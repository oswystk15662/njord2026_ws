#include "zed2i_driver/message_utils.hpp"
#include "zed2i_driver/detection_message_utils.hpp"
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
#include "zed2i_driver/tensor_rt_detector.hpp"
#endif

#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sl/Camera.hpp>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <memory>
#include <limits>
#include <string>
#include <vector>

namespace zed2i_driver
{

class SdkNode : public rclcpp::Node
{
public:
  explicit SdkNode(const rclcpp::NodeOptions & options)
  : Node("zed2i_sdk_node", options)
  {
    left_frame_id_ = declare_parameter<std::string>("left_frame_id", "zed2i_left_camera_frame");
    right_frame_id_ = declare_parameter<std::string>("right_frame_id", "zed2i_right_camera_frame");
    depth_frame_id_ = declare_parameter<std::string>("depth_frame_id", left_frame_id_);
    framerate_ = declare_parameter<int>("framerate", 15);
    publish_pointcloud_ = declare_parameter<bool>("publish_pointcloud", true);
    depth_min_m_ = declare_parameter<double>("depth_min_m", 0.3);
    depth_max_m_ = declare_parameter<double>("depth_max_m", 20.0);
    engine_path_ = declare_parameter<std::string>("engine_path", "");
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
      detector_ = std::make_unique<TensorRtDetector>(engine_path_, confidence_threshold_, max_detections_);
      detection_pub_ = create_publisher<njord_interfaces::msg::BuoyDetectionArray>(
        detection_topic_, rclcpp::QoS(10));
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
    init_params.camera_resolution = sl::RESOLUTION::HD720;
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
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    if (detector_) {
      left_gpu_ = sl::Mat(width_, height_, sl::MAT_TYPE::U8_C4, sl::MEM::GPU);
      depth_gpu_ = sl::Mat(width_, height_, sl::MAT_TYPE::F32_C1, sl::MEM::GPU);
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

    if (publish_left) {
      sl::Mat left;
      camera_.retrieveImage(left, sl::VIEW::LEFT);
      const auto left_bgra = sl_mat_to_cv_bgra_view(left);
      left_image_pub_->publish(mat_to_image_msg(left_bgra, "bgra8", left_frame_id_, stamp));
    }
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
    if (run_gpu) {
      camera_.retrieveImage(left_gpu_, sl::VIEW::LEFT, sl::MEM::GPU);
      camera_.retrieveMeasure(depth_gpu_, sl::MEASURE::DEPTH, sl::MEM::GPU);
      const auto * pointer = left_gpu_.getPtr<sl::uchar1>(sl::MEM::GPU);
      try {
        const auto detections = detector_->infer(
          pointer, left_gpu_.getStepBytes(sl::MEM::GPU), width_, height_, nullptr);
        std_msgs::msg::Header header;
        header.stamp = stamp;
        header.frame_id = output_frame_;
        std::vector<PositionedDetection> positioned;
        positioned.reserve(detections.size());
        for (const auto & detection : detections) {
          positioned.push_back({detection, nan_position(), PositionSource::kNone});
        }
        detection_pub_->publish(to_detection_array(positioned, header));
      } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "GPU perception stopped: %s", error.what());
        detector_.reset();
      }
    }
#endif

    if (publish_right) {
      sl::Mat right;
      camera_.retrieveImage(right, sl::VIEW::RIGHT);
      const auto right_bgra = sl_mat_to_cv_bgra_view(right);
      right_image_pub_->publish(mat_to_image_msg(right_bgra, "bgra8", right_frame_id_, stamp));
    }

    if (publish_depth || publish_points) {
      sl::Mat depth;
      camera_.retrieveMeasure(depth, sl::MEASURE::DEPTH);
      // This is a non-owning view. Each requested ROS output copies it directly into its
      // final message allocation before the SDK buffer expires.
      const auto depth_m = sl_mat_to_cv_depth_view(depth);
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
#ifdef ZED2I_DRIVER_HAS_GPU_PERCEPTION
  std::unique_ptr<TensorRtDetector> detector_;
  sl::Mat left_gpu_;
  sl::Mat depth_gpu_;
  std::string detection_topic_;
  std::string output_frame_;
  double confidence_threshold_{};
  int max_detections_{};
  rclcpp::Publisher<njord_interfaces::msg::BuoyDetectionArray>::SharedPtr detection_pub_;
#endif

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
};

}  // namespace zed2i_driver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(zed2i_driver::SdkNode)
