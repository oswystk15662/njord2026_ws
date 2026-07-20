#include "zed2i_driver/message_utils.hpp"

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
#include <string>

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
    sl::RuntimeParameters runtime_params;
    if (camera_.grab(runtime_params) != sl::ERROR_CODE::SUCCESS) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to grab a ZED frame");
      return;
    }

    sl::Mat left;
    sl::Mat right;
    sl::Mat depth;
    camera_.retrieveImage(left, sl::VIEW::LEFT);
    camera_.retrieveImage(right, sl::VIEW::RIGHT);
    camera_.retrieveMeasure(depth, sl::MEASURE::DEPTH);

    const auto stamp = now();
    // These cv::Mat objects are non-owning views. The ROS message conversion copies each
    // SDK buffer once into its final message allocation before the sl::Mat objects expire.
    const auto left_bgra = sl_mat_to_cv_bgra_view(left);
    const auto right_bgra = sl_mat_to_cv_bgra_view(right);
    const auto depth_m = sl_mat_to_cv_depth_view(depth);

    left_image_pub_->publish(mat_to_image_msg(left_bgra, "bgra8", left_frame_id_, stamp));
    right_image_pub_->publish(mat_to_image_msg(right_bgra, "bgra8", right_frame_id_, stamp));
    left_info_pub_->publish(make_camera_info_msg(width_, height_, fx_, fy_, cx_, cy_, 0.0,
        left_frame_id_, stamp));
    right_info_pub_->publish(
      make_camera_info_msg(width_, height_, fx_, fy_, cx_, cy_, baseline_m_, right_frame_id_,
        stamp));
    depth_pub_->publish(mat_to_image_msg(depth_m, "32FC1", depth_frame_id_, stamp));

    if (publish_pointcloud_) {
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
