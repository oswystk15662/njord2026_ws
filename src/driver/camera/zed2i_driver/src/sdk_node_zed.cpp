#include "zed2i_driver/message_utils.hpp"
#include "zed2i_driver/detection_message_utils.hpp"
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
#include "zed2i_driver/gpu_depth_median.hpp"
#include "zed2i_driver/perception_logic.hpp"
#include "zed2i_driver/tensor_rt_detector.hpp"
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
#include "zed2i_driver/ground_video_streamer.hpp"
#endif

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sl/Camera.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <limits>
#include <string>
#include <stdexcept>
#include <vector>
#include <mutex>
#include <optional>

#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace zed2i_driver
{

namespace
{

sl::RESOLUTION parse_camera_resolution(const std::string & value)
{
  if (value == "HD2K") return sl::RESOLUTION::HD2K;
  if (value == "HD1080") return sl::RESOLUTION::HD1080;
  if (value == "HD720") return sl::RESOLUTION::HD720;
  if (value == "VGA") return sl::RESOLUTION::VGA;
  throw std::invalid_argument("camera_resolution must be one of HD2K, HD1080, HD720, or VGA");
}

// Per-row [x_start, x_end) column bounds of the human-FOV-like elliptical
// region of interest. Precomputing this once (per camera-info change) lets
// every per-frame consumer (RGB, depth, point cloud) skip whole outside
// spans with a memset/fill instead of testing every pixel against the
// ellipse equation.
struct EllipseRowSpans
{
  std::vector<int> x_start;
  std::vector<int> x_end;  // exclusive
};

EllipseRowSpans compute_ellipse_row_spans(
  int width, int height, double cx, double cy, double a, double b)
{
  EllipseRowSpans spans;
  spans.x_start.assign(static_cast<size_t>(height), 0);
  spans.x_end.assign(static_cast<size_t>(height), 0);
  a = std::max(a, 1e-6);
  b = std::max(b, 1e-6);
  for (int y = 0; y < height; ++y) {
    const double dy = (static_cast<double>(y) - cy) / b;
    const double term = 1.0 - dy * dy;
    if (term < 0.0) {
      continue;
    }
    const double dx = a * std::sqrt(term);
    int x0 = static_cast<int>(std::ceil(cx - dx));
    int x1 = static_cast<int>(std::floor(cx + dx)) + 1;
    x0 = std::clamp(x0, 0, width);
    x1 = std::clamp(x1, 0, width);
    if (x1 < x0) {
      x1 = x0;
    }
    spans.x_start[static_cast<size_t>(y)] = x0;
    spans.x_end[static_cast<size_t>(y)] = x1;
  }
  return spans;
}

// Blacks out the columns outside the precomputed per-row span of a BGRA8
// image (4 bytes/pixel), leaving the inside untouched.
void apply_ellipse_mask_bgra(cv::Mat & image, const EllipseRowSpans & spans)
{
  for (int y = 0; y < image.rows; ++y) {
    auto * row = image.ptr<std::uint8_t>(y);
    const int x0 = spans.x_start[static_cast<size_t>(y)];
    const int x1 = spans.x_end[static_cast<size_t>(y)];
    if (x0 > 0) {
      std::memset(row, 0, static_cast<size_t>(x0) * 4);
    }
    if (x1 < image.cols) {
      std::memset(
        row + static_cast<size_t>(x1) * 4, 0,
        static_cast<size_t>(image.cols - x1) * 4);
    }
  }
}

// Invalidates (NaN) the columns outside the precomputed per-row span of a
// 32FC1 depth image. NaN matches the depth invalid convention already
// checked by depth_to_point_cloud_msg (std::isfinite), so the point-cloud
// builder automatically skips masked-out pixels with no further changes.
void apply_ellipse_mask_depth(cv::Mat & depth, const EllipseRowSpans & spans)
{
  constexpr float kInvalid = std::numeric_limits<float>::quiet_NaN();
  for (int y = 0; y < depth.rows; ++y) {
    auto * row = depth.ptr<float>(y);
    const int x0 = spans.x_start[static_cast<size_t>(y)];
    const int x1 = spans.x_end[static_cast<size_t>(y)];
    std::fill(row, row + x0, kInvalid);
    std::fill(row + x1, row + depth.cols, kInvalid);
  }
}

}  // namespace

class SdkNode : public rclcpp::Node
{
public:
  explicit SdkNode(const rclcpp::NodeOptions & options)
  : Node("zed2i_sdk_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    left_frame_id_ = declare_parameter<std::string>("left_frame_id", "zed2i_left_camera_frame");
    right_frame_id_ = declare_parameter<std::string>("right_frame_id", "zed2i_right_camera_frame");
    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", left_frame_id_);
    framerate_ = declare_parameter<int>("framerate", 15);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    depth_min_m_ = declare_parameter<double>("depth_min_m", 0.3);
    depth_max_m_ = declare_parameter<double>("depth_max_m", 20.0);
    engine_path_ = declare_parameter<std::string>("engine_path", "");
    fov_ellipse_enable_ = declare_parameter<bool>("fov_ellipse_enable", false);
    fov_ellipse_cx_ratio_ = declare_parameter<double>("fov_ellipse_cx_ratio", 0.5);
    fov_ellipse_cy_ratio_ = declare_parameter<double>("fov_ellipse_cy_ratio", 0.5);
    fov_ellipse_a_ratio_ = declare_parameter<double>("fov_ellipse_a_ratio", 0.5);
    fov_ellipse_b_ratio_ = declare_parameter<double>("fov_ellipse_b_ratio", 0.5);
    const auto camera_resolution = declare_parameter<std::string>("camera_resolution", "HD720");
    const bool enable_ground_video = declare_parameter<bool>("enable_ground_video", false);
    const auto ground_video_codec = declare_parameter<std::string>("ground_video_codec", "jpeg");
    ground_video_config_.width = declare_parameter<int>("ground_video_width", 480);
    ground_video_config_.height = declare_parameter<int>("ground_video_height", 360);
    ground_video_config_.fps = declare_parameter<double>("ground_video_fps", 4.0);
    ground_video_config_.jpeg_quality = declare_parameter<int>("ground_video_jpeg_quality", 70);
    ground_video_config_.host = declare_parameter<std::string>("ground_video_host", "");
    ground_video_config_.port = declare_parameter<int>("ground_video_port", 5600);
    ground_video_config_.max_pending_frames = declare_parameter<int>("ground_video_max_pending_frames", 1);
    ground_video_config_.mtu = declare_parameter<int>("ground_video_mtu", 1200);
    const bool enable_gpu_perception = declare_parameter<bool>("enable_gpu_perception", false);
    if (enable_gpu_perception) {
#ifndef ZED2I_DRIVER_HAS_GPU_PERCEPTION
      RCLCPP_FATAL(
        get_logger(),
        "enable_gpu_perception=true requires a build with ZED, CUDA, and TensorRT support.");
      rclcpp::shutdown();
      return;
#else
      if (engine_path_.empty()) {
        RCLCPP_FATAL(get_logger(), "enable_gpu_perception=true requires a non-empty engine_path");
        rclcpp::shutdown();
        return;
      }
      confidence_threshold_ = declare_parameter<double>("confidence_threshold", 0.25);
      max_detections_ = declare_parameter<int>("max_detections", 32);
      detection_topic_ = declare_parameter<std::string>("detection_topic", "/buoy_detections_3d");
      output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
      publish_debug_detections_ = declare_parameter<bool>("publish_debug_detections", false);
      depth_center_ratio_ = declare_parameter<double>("depth_center_ratio", 0.5);
      depth_sample_stride_ = declare_parameter<int>("depth_sample_stride", 2);
      min_valid_depth_samples_ = declare_parameter<int>("min_valid_depth_samples", 16);
      enable_virtual_wall_ = declare_parameter<bool>("enable_virtual_wall", true);
      virtual_wall_topic_ = declare_parameter<std::string>(
        "virtual_wall_topic", "/virtual_obstacles");
      wall_frame_ = declare_parameter<std::string>("wall_frame", "map");
      channel_heading_rad_ = declare_parameter<double>("channel_heading_rad", NAN);
      wall_radius_m_ = declare_parameter<double>("wall_radius_m", 2.0);
      wall_points_per_full_circle_ = declare_parameter<int>("wall_points_per_full_circle", 40);
      lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/livox/lidar");
      lidar_max_age_sec_ = declare_parameter<double>("lidar_max_age_sec", 0.15);
      lidar_cluster_tolerance_m_ = declare_parameter<double>("lidar_cluster_tolerance_m", 0.15);
      lidar_min_cluster_points_ = declare_parameter<int>("lidar_min_cluster_points", 5);
      lidar_max_cluster_points_ = declare_parameter<int>("lidar_max_cluster_points", 5000);
      detector_ = std::make_unique<TensorRtDetector>(engine_path_, confidence_threshold_, max_detections_);
      // Canonical output for Nav2-side perception consumers.  This must not
      // depend on the optional debug-detection setting.
      detection_pub_ = create_publisher<njord_interfaces::msg::BuoyDetectionArray>(
        detection_topic_, rclcpp::QoS(10));
      if (enable_virtual_wall_) {
        virtual_wall_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
          virtual_wall_topic_, rclcpp::QoS(10));
      }
      lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(lidar_mutex_);
          latest_lidar_ = std::move(message);
        });
#endif
    }

    rclcpp::QoS image_qos(rclcpp::KeepLast(5));
    image_qos.best_effort();
    image_qos.durability_volatile();

    left_image_pub_ = create_publisher<sensor_msgs::msg::Image>("left/image_rect", image_qos);
    right_image_pub_ = create_publisher<sensor_msgs::msg::Image>("right/image_rect", image_qos);
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("left/camera_info", image_qos);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("right/camera_info",
        image_qos);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("depth/image", image_qos);
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("points", image_qos);

    sl::InitParameters init_params;
    try {
      init_params.camera_resolution = parse_camera_resolution(camera_resolution);
    } catch (const std::invalid_argument & error) {
      RCLCPP_FATAL(get_logger(), "%s", error.what());
      rclcpp::shutdown();
      return;
    }
    init_params.camera_fps = framerate_;
    init_params.depth_mode = sl::DEPTH_MODE::QUALITY;
    init_params.coordinate_units = sl::UNIT::METER;

    const auto error = camera_.open(init_params);
    if (error != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_FATAL(get_logger(), "Failed to open ZED camera: %s", sl::toString(error).c_str());
      rclcpp::shutdown();
      return;
    }

    const auto info = camera_.getCameraInformation();
    const auto calibration = info.camera_configuration.calibration_parameters;
    width_ = static_cast<int>(info.camera_configuration.resolution.width);
    height_ = static_cast<int>(info.camera_configuration.resolution.height);
    fx_ = calibration.left_cam.fx;
    fy_ = calibration.left_cam.fy;
    cx_ = calibration.left_cam.cx;
    cy_ = calibration.left_cam.cy;
    baseline_m_ = std::abs(calibration.stereo_transform.getTranslation().tx);
    if (baseline_m_ > 10.0) {
      baseline_m_ *= 0.001;
    }
    ellipse_cx_ = fov_ellipse_cx_ratio_ * width_;
    ellipse_cy_ = fov_ellipse_cy_ratio_ * height_;
    ellipse_a_ = fov_ellipse_a_ratio_ * width_;
    ellipse_b_ = fov_ellipse_b_ratio_ * height_;
    if (fov_ellipse_enable_) {
      ellipse_row_spans_ = compute_ellipse_row_spans(
        width_, height_, ellipse_cx_, ellipse_cy_, ellipse_a_, ellipse_b_);
    }
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    if (detector_) {
      left_gpu_ = sl::Mat(width_, height_, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
      depth_gpu_ = sl::Mat(width_, height_, sl::MAT_TYPE::F32_C1, sl::MEM::GPU);
    }
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
    if (enable_ground_video && ground_video_codec == "jpeg") {
      GroundVideoConfig config;
      config.source_width = width_;
      config.source_height = height_;
      config.width = ground_video_config_.width;
      config.height = ground_video_config_.height;
      config.fps = ground_video_config_.fps;
      config.jpeg_quality = ground_video_config_.jpeg_quality;
      config.host = ground_video_config_.host;
      config.port = ground_video_config_.port;
      config.max_pending_frames = ground_video_config_.max_pending_frames;
      config.mtu = ground_video_config_.mtu;
      config.fov_ellipse_enable = fov_ellipse_enable_;
      config.fov_ellipse_cx_ratio = fov_ellipse_cx_ratio_;
      config.fov_ellipse_cy_ratio = fov_ellipse_cy_ratio_;
      config.fov_ellipse_a_ratio = fov_ellipse_a_ratio_;
      config.fov_ellipse_b_ratio = fov_ellipse_b_ratio_;
      try {
        ground_video_streamer_ = std::make_unique<GroundVideoStreamer>(config);
        if (!left_gpu_.isInit()) {
          left_gpu_ = sl::Mat(width_, height_, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Ground-video streaming disabled: %s", error.what());
      }
    }
    if (enable_ground_video && ground_video_codec != "jpeg") {
      RCLCPP_ERROR(get_logger(), "Ground-video streaming disabled: only JPEG is supported");
    }
#else
    if (enable_ground_video) {
      RCLCPP_ERROR(
        get_logger(),
        "Ground-video streaming disabled: this build requires ZED, CUDA, nvJPEG, and GStreamer appsrc.");
    }
#endif

    const auto period_ms = std::max(1, 1000 / std::max(1, framerate_));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&SdkNode::grab_and_publish, this));
  }

  ~SdkNode() override
  {
    camera_.close();
  }

private:
  std::optional<geometry_msgs::msg::PointStamped> lidar_fallback_position(
    const Detection2D & detection, const rclcpp::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      cloud = latest_lidar_;
    }
    if (!cloud || cloud->header.frame_id.empty()) {
      return std::nullopt;
    }
    const auto age = std::abs((stamp - rclcpp::Time(cloud->header.stamp)).seconds());
    if (age > lidar_max_age_sec_) {
      return std::nullopt;
    }

    geometry_msgs::msg::TransformStamped cloud_to_camera;
    try {
      cloud_to_camera = tf_buffer_.lookupTransform(
        left_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping LiDAR fallback: %s -> %s TF unavailable: %s",
        cloud->header.frame_id.c_str(), left_frame_id_.c_str(), error.what());
      return std::nullopt;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr candidates(new pcl::PointCloud<pcl::PointXYZ>());
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*cloud, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) {
          continue;
        }
        geometry_msgs::msg::PointStamped point_in;
        point_in.header = cloud->header;
        point_in.point.x = *x;
        point_in.point.y = *y;
        point_in.point.z = *z;
        geometry_msgs::msg::PointStamped point_camera;
        tf2::doTransform(point_in, point_camera, cloud_to_camera);
        if (point_camera.point.z <= 0.0) {
          continue;
        }
        const auto u = fx_ * point_camera.point.x / point_camera.point.z + cx_;
        const auto v = fy_ * point_camera.point.y / point_camera.point.z + cy_;
        if (point_in_central_bbox_roi(
            static_cast<float>(u), static_cast<float>(v), detection,
            static_cast<float>(depth_center_ratio_))) {
          candidates->push_back({
            static_cast<float>(point_camera.point.x),
            static_cast<float>(point_camera.point.y),
            static_cast<float>(point_camera.point.z)});
        }
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping LiDAR fallback: PointCloud2 lacks xyz fields: %s", error.what());
      return std::nullopt;
    }
    if (candidates->size() < static_cast<size_t>(lidar_min_cluster_points_)) {
      return std::nullopt;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(candidates);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(lidar_cluster_tolerance_m_);
    extraction.setMinClusterSize(lidar_min_cluster_points_);
    extraction.setMaxClusterSize(lidar_max_cluster_points_);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(candidates);
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);
    if (clusters.empty()) {
      return std::nullopt;
    }
    const auto center_u = (detection.x1 + detection.x2) * 0.5F;
    const auto center_v = (detection.y1 + detection.y2) * 0.5F;
    float best_distance = std::numeric_limits<float>::infinity();
    Eigen::Vector4f selected;
    bool found = false;
    for (const auto & cluster : clusters) {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*candidates, cluster.indices, centroid);
      const auto u = static_cast<float>(fx_) * centroid.x() / centroid.z() + static_cast<float>(cx_);
      const auto v = static_cast<float>(fy_) * centroid.y() / centroid.z() + static_cast<float>(cy_);
      const auto distance = (u - center_u) * (u - center_u) + (v - center_v) * (v - center_v);
      if (distance < best_distance) {
        best_distance = distance;
        selected = centroid;
        found = true;
      }
    }
    if (!found) {
      return std::nullopt;
    }
    geometry_msgs::msg::PointStamped camera_point;
    camera_point.header.stamp = stamp;
    camera_point.header.frame_id = left_frame_id_;
    camera_point.point.x = selected.x();
    camera_point.point.y = selected.y();
    camera_point.point.z = selected.z();
    try {
      const auto camera_to_output = tf_buffer_.lookupTransform(
        output_frame_, left_frame_id_, stamp, rclcpp::Duration::from_seconds(0.05));
      geometry_msgs::msg::PointStamped output_point;
      tf2::doTransform(camera_point, output_point, camera_to_output);
      return output_point;
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Skipping LiDAR fallback: %s -> %s TF unavailable: %s",
        left_frame_id_.c_str(), output_frame_.c_str(), error.what());
      return std::nullopt;
    }
  }

  template<typename MessageT>
  static bool has_subscribers(const std::shared_ptr<rclcpp::Publisher<MessageT>> & publisher)
  {
    return publisher->get_subscription_count() > 0 ||
           publisher->get_intra_process_subscription_count() > 0;
  }

  static cv::Mat sl_mat_to_cv_bgra_view(const sl::Mat & mat)
  {
    const auto width = static_cast<int>(mat.getWidth());
    const auto height = static_cast<int>(mat.getHeight());
    return cv::Mat(
      height, width, CV_8UC4, mat.getPtr<sl::uchar1>(sl::MEM::CPU),
      mat.getStepBytes(sl::MEM::CPU));
  }

  // Cheap per-detection ellipse membership test used to gate GPU perception
  // output. width_/height_ and the ellipse_* members are fixed after the
  // camera opens, so this is a handful of flops per detection.
  bool ellipse_contains(double x, double y) const
  {
    const double dx = (x - ellipse_cx_) / std::max(ellipse_a_, 1e-6);
    const double dy = (y - ellipse_cy_) / std::max(ellipse_b_, 1e-6);
    return dx * dx + dy * dy <= 1.0;
  }

  static cv::Mat sl_mat_to_cv_depth_view(const sl::Mat & mat)
  {
    const auto width = static_cast<int>(mat.getWidth());
    const auto height = static_cast<int>(mat.getHeight());
    return cv::Mat(
      height, width, CV_32FC1, mat.getPtr<sl::float1>(sl::MEM::CPU),
      mat.getStepBytes(sl::MEM::CPU));
  }

  void grab_and_publish()
  {
    bool run_gpu = false;
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    run_gpu = static_cast<bool>(detector_);
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
    run_gpu = run_gpu || static_cast<bool>(ground_video_streamer_);
#endif
    const bool publish_left = has_subscribers(left_image_pub_);
    const bool publish_right = has_subscribers(right_image_pub_);
    const bool publish_left_info = has_subscribers(left_info_pub_);
    const bool publish_right_info = has_subscribers(right_info_pub_);
    const bool publish_depth = has_subscribers(depth_pub_);
    const bool publish_points = publish_pointcloud_ && has_subscribers(pointcloud_pub_);

    if (!publish_left && !publish_right && !publish_left_info && !publish_right_info &&
      !publish_depth && !publish_points && !run_gpu)
    {
      return;
    }

    sl::RuntimeParameters runtime_params;
    if (camera_.grab(runtime_params) != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to grab a ZED frame");
      return;
    }
    const auto stamp = now();
    if (publish_left_info) {
      left_info_pub_->publish(make_camera_info_msg(
          width_, height_, fx_, fy_, cx_, cy_, 0.0, left_frame_id_, stamp));
    }
    if (publish_right_info) {
      right_info_pub_->publish(make_camera_info_msg(
          width_, height_, fx_, fy_, cx_, cy_, baseline_m_, right_frame_id_, stamp));
    }

    bool left_published = false;
#if defined(ZED2I_DRIVER_HAS_GPU_PERCEPTION) || defined(ZED2I_DRIVER_HAS_GROUND_VIDEO)
    if (run_gpu) {
      const auto image_error = camera_.retrieveImage(left_gpu_, sl::VIEW::LEFT, sl::MEM::GPU);
      if (image_error != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Failed to retrieve the left ZED image on GPU: %s",
          sl::toString(image_error).c_str());
      } else {
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
        // Select the ground frame before any GPU-to-CPU download or DDS publish.
        // submit() only enqueues a latest-wins D2D copy; encoding and networking
        // continue on the streamer's worker thread.
        if (ground_video_streamer_) {
          ground_video_streamer_->submit(
            left_gpu_.getPtr<sl::uchar1>(sl::MEM::GPU), left_gpu_.getStepBytes(sl::MEM::GPU),
            width_, height_);
        }
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
        if (detector_) {
          camera_.retrieveMeasure(depth_gpu_, sl::MEASURE::DEPTH, sl::MEM::GPU);
          const auto * pointer = left_gpu_.getPtr<sl::uchar1>(sl::MEM::GPU);
          try {
            const auto detections = detector_->infer(
              pointer, left_gpu_.getStepBytes(sl::MEM::GPU), width_, height_, nullptr);
            const auto depth_pointer = depth_gpu_.getPtr<sl::float1>(sl::MEM::GPU);
            std::vector<PositionedDetection> positioned;
            positioned.reserve(detections.size());
            for (const auto & detection : detections) {
              if (fov_ellipse_enable_) {
                const auto center_u = (detection.x1 + detection.x2) * 0.5F;
                const auto center_v = (detection.y1 + detection.y2) * 0.5F;
                if (!ellipse_contains(center_u, center_v)) {
                  continue;
                }
              }
              PositionedDetection item{detection, nan_position(), PositionSource::kNone};
              const auto roi = central_depth_roi(
                detection, static_cast<float>(depth_center_ratio_), width_, height_);
              const auto depth = depth_median_gpu(
                depth_pointer, depth_gpu_.getStepBytes(sl::MEM::GPU), width_, height_,
                roi.x0, roi.y0, roi.x1, roi.y1, depth_sample_stride_,
                static_cast<float>(depth_min_m_), static_cast<float>(depth_max_m_),
                min_valid_depth_samples_, nullptr);
              if (std::isfinite(depth)) {
                geometry_msgs::msg::PointStamped camera_point;
                camera_point.header.stamp = stamp;
                camera_point.header.frame_id = left_frame_id_;
                const auto u = (detection.x1 + detection.x2) * 0.5F;
                const auto v = (detection.y1 + detection.y2) * 0.5F;
                camera_point.point.x = (u - cx_) * depth / fx_;
                camera_point.point.y = (v - cy_) * depth / fy_;
                camera_point.point.z = depth;
                try {
                  const auto transform = tf_buffer_.lookupTransform(
                    output_frame_, left_frame_id_, stamp, rclcpp::Duration::from_seconds(0.05));
                  geometry_msgs::msg::PointStamped output_point;
                  tf2::doTransform(camera_point, output_point, transform);
                  item.position_base = {
                    static_cast<float>(output_point.point.x),
                    static_cast<float>(output_point.point.y),
                    static_cast<float>(output_point.point.z)};
                  item.source = PositionSource::kZedDepth;
                } catch (const tf2::TransformException & error) {
                  RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Skipping buoy position: %s -> %s TF unavailable: %s",
                    left_frame_id_.c_str(), output_frame_.c_str(), error.what());
                }
              }
              if (item.source == PositionSource::kNone) {
                if (const auto fallback = lidar_fallback_position(detection, stamp)) {
                  item.position_base = {
                    static_cast<float>(fallback->point.x),
                    static_cast<float>(fallback->point.y),
                    static_cast<float>(fallback->point.z)};
                  item.source = PositionSource::kLidarFused;
                }
              }
              positioned.push_back(item);
            }
            std_msgs::msg::Header header;
            header.stamp = stamp;
            header.frame_id = output_frame_;
            if (detection_pub_) {
              detection_pub_->publish(to_detection_array(positioned, header));
            }
            if (virtual_wall_pub_) {
              std::vector<PositionedDetection> wall_detections;
              wall_detections.reserve(positioned.size());
              for (const auto & item : positioned) {
                if (item.source == PositionSource::kNone) {
                  continue;
                }
                try {
                  geometry_msgs::msg::PointStamped output_point;
                  output_point.header = header;
                  output_point.point.x = item.position_base[0];
                  output_point.point.y = item.position_base[1];
                  output_point.point.z = item.position_base[2];
                  const auto transform = tf_buffer_.lookupTransform(
                    wall_frame_, output_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
                  geometry_msgs::msg::PointStamped wall_point;
                  tf2::doTransform(output_point, wall_point, transform);
                  auto wall_item = item;
                  wall_item.position_base = {
                    static_cast<float>(wall_point.point.x),
                    static_cast<float>(wall_point.point.y),
                    static_cast<float>(wall_point.point.z)};
                  wall_detections.push_back(wall_item);
                } catch (const tf2::TransformException & error) {
                  RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000,
                    "Skipping virtual wall: %s -> %s TF unavailable: %s",
                    output_frame_.c_str(), wall_frame_.c_str(), error.what());
                }
              }
              auto wall_header = header;
              wall_header.frame_id = wall_frame_;
              virtual_wall_pub_->publish(to_virtual_wall_cloud(
                wall_detections, wall_header, wall_frame_,
                static_cast<float>(channel_heading_rad_), static_cast<float>(wall_radius_m_),
                wall_points_per_full_circle_));
            }
          } catch (const std::exception & error) {
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 2000, "GPU perception stopped: %s", error.what());
            detector_.reset();
          }
        }
#endif
        if (publish_left) {
          const auto download_error = left_gpu_.updateCPUfromGPU();
          if (download_error == sl::ERROR_CODE::SUCCESS) {
            auto left_bgra = sl_mat_to_cv_bgra_view(left_gpu_);
            if (fov_ellipse_enable_) {
              apply_ellipse_mask_bgra(left_bgra, ellipse_row_spans_);
            }
            left_image_pub_->publish(
              mat_to_image_msg(left_bgra, "bgra8", left_frame_id_, stamp));
            left_published = true;
          } else {
            RCLCPP_WARN_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Failed to download the left ZED image after ground-video submission: %s",
              sl::toString(download_error).c_str());
          }
        }
      }
    }
#endif

    if (publish_left && !left_published) {
      sl::Mat left;
      camera_.retrieveImage(left, sl::VIEW::LEFT);
      auto left_bgra = sl_mat_to_cv_bgra_view(left);
      if (fov_ellipse_enable_) {
        apply_ellipse_mask_bgra(left_bgra, ellipse_row_spans_);
      }
      left_image_pub_->publish(mat_to_image_msg(left_bgra, "bgra8", left_frame_id_, stamp));
    }

    if (publish_right) {
      sl::Mat right;
      camera_.retrieveImage(right, sl::VIEW::RIGHT);
      auto right_bgra = sl_mat_to_cv_bgra_view(right);
      if (fov_ellipse_enable_) {
        apply_ellipse_mask_bgra(right_bgra, ellipse_row_spans_);
      }
      right_image_pub_->publish(mat_to_image_msg(right_bgra, "bgra8", right_frame_id_, stamp));
    }

    if (publish_depth || publish_points) {
      sl::Mat depth;
      camera_.retrieveMeasure(depth, sl::MEASURE::DEPTH);
      // This is a non-owning view. Each requested ROS output copies it directly into its
      // final message allocation before the SDK buffer expires.
      auto depth_m = sl_mat_to_cv_depth_view(depth);
      if (fov_ellipse_enable_) {
        // NaN matches the existing invalid-depth convention, so the point
        // cloud loop below (which already tests std::isfinite) needs no
        // further change to skip masked-out pixels.
        apply_ellipse_mask_depth(depth_m, ellipse_row_spans_);
      }
      if (publish_depth) {
        depth_pub_->publish(mat_to_image_msg(depth_m, "32FC1", depth_frame_id_, stamp));
      }
      if (!publish_points) {
        return;
      }
      pointcloud_pub_->publish(
        depth_to_point_cloud_msg(
          depth_m, fx_, fy_, cx_, cy_, 1, depth_min_m_, depth_max_m_, depth_frame_id_, stamp));
    }
  }

  sl::Camera camera_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string depth_frame_id_;
  int framerate_;
  bool publish_pointcloud_;
  double depth_min_m_;
  double depth_max_m_;
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double baseline_m_;
  std::string engine_path_;
  bool fov_ellipse_enable_{false};
  double fov_ellipse_cx_ratio_{0.5};
  double fov_ellipse_cy_ratio_{0.5};
  double fov_ellipse_a_ratio_{0.5};
  double fov_ellipse_b_ratio_{0.5};
  double ellipse_cx_{0.0};
  double ellipse_cy_{0.0};
  double ellipse_a_{0.0};
  double ellipse_b_{0.0};
  EllipseRowSpans ellipse_row_spans_;
  struct GroundVideoParameterStorage
  {
    int source_width{};
    int source_height{};
    int width{640};
    int height{360};
    double fps{5.0};
    int jpeg_quality{70};
    std::string host;
    int port{5600};
    int max_pending_frames{1};
    int mtu{1200};
  } ground_video_config_;
#if defined(ZED2I_DRIVER_HAS_GPU_PERCEPTION) || defined(ZED2I_DRIVER_HAS_GROUND_VIDEO)
  sl::Mat left_gpu_;
#endif
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
  std::unique_ptr<TensorRtDetector> detector_;
  sl::Mat depth_gpu_;
  std::string detection_topic_;
  std::string output_frame_;
  double confidence_threshold_{};
  int max_detections_{};
  bool publish_debug_detections_{};
  double depth_center_ratio_{};
  int depth_sample_stride_{};
  int min_valid_depth_samples_{};
  bool enable_virtual_wall_{};
  std::string virtual_wall_topic_;
  std::string wall_frame_;
  double channel_heading_rad_{};
  double wall_radius_m_{};
  int wall_points_per_full_circle_{};
  rclcpp::Publisher<njord_interfaces::msg::BuoyDetectionArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr virtual_wall_pub_;
  std::string lidar_topic_;
  double lidar_max_age_sec_{};
  double lidar_cluster_tolerance_m_{};
  int lidar_min_cluster_points_{};
  int lidar_max_cluster_points_{};
  std::mutex lidar_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_lidar_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
#endif
#ifdef ZED2I_DRIVER_HAS_GROUND_VIDEO
  std::unique_ptr<GroundVideoStreamer> ground_video_streamer_;
#endif

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace zed2i_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed2i_driver::SdkNode)
