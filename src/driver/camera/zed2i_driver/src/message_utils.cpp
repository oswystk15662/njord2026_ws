#include "zed2i_driver/message_utils.hpp"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace zed2i_driver
{
namespace
{

int bytes_per_pixel(const cv::Mat & image)
{
  return static_cast<int>(image.elemSize());
}

}  // namespace

sensor_msgs::msg::Image::UniquePtr mat_to_image_msg(
  const cv::Mat & image,
  const std::string & encoding,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (image.empty()) {
    throw std::runtime_error("cannot convert an empty cv::Mat to sensor_msgs::msg::Image");
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->height = static_cast<uint32_t>(image.rows);
  msg->width = static_cast<uint32_t>(image.cols);
  msg->encoding = encoding;
  msg->is_bigendian = false;
  msg->step = static_cast<uint32_t>(image.cols * bytes_per_pixel(image));
  msg->data.resize(static_cast<size_t>(msg->step) * msg->height);
  if (image.isContinuous() && image.step == msg->step) {
    std::memcpy(msg->data.data(), image.data, msg->data.size());
  } else {
    for (int row = 0; row < image.rows; ++row) {
      std::memcpy(
        msg->data.data() + static_cast<size_t>(row) * msg->step,
        image.ptr(row), msg->step);
    }
  }
  return msg;
}

sensor_msgs::msg::CameraInfo make_camera_info_msg(
  int width,
  int height,
  double fx,
  double fy,
  double cx,
  double cy,
  double baseline_m,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  sensor_msgs::msg::CameraInfo msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.width = static_cast<uint32_t>(width);
  msg.height = static_cast<uint32_t>(height);
  msg.distortion_model = "plumb_bob";
  msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
  msg.k = {
    fx, 0.0, cx,
    0.0, fy, cy,
    0.0, 0.0, 1.0};
  msg.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0};
  msg.p = {
    fx, 0.0, cx, -fx * baseline_m,
    0.0, fy, cy, 0.0,
    0.0, 0.0, 1.0, 0.0};
  return msg;
}

sensor_msgs::msg::PointCloud2::UniquePtr depth_to_point_cloud_msg(
  const cv::Mat & depth_m,
  double fx,
  double fy,
  double cx,
  double cy,
  int stride,
  double depth_min_m,
  double depth_max_m,
  const std::string & frame_id,
  const rclcpp::Time & stamp)
{
  if (depth_m.empty() || depth_m.type() != CV_32FC1) {
    throw std::runtime_error("depth_to_point_cloud_msg expects a non-empty CV_32FC1 image");
  }

  stride = std::max(1, stride);
  auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
  msg->header.stamp = stamp;
  msg->header.frame_id = frame_id;
  msg->height = 1;
  msg->is_bigendian = false;
  msg->is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(*msg);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  const auto point_rows = static_cast<size_t>((depth_m.rows + stride - 1) / stride);
  const auto point_columns = static_cast<size_t>((depth_m.cols + stride - 1) / stride);
  const auto max_points = point_rows * point_columns;
  modifier.resize(max_points);

  size_t point_count = 0;

  for (int v = 0; v < depth_m.rows; v += stride) {
    const float * row = depth_m.ptr<float>(v);
    for (int u = 0; u < depth_m.cols; u += stride) {
      const float z = row[u];
      if (!std::isfinite(z) || z < depth_min_m || z > depth_max_m) {
        continue;
      }

      const float x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
      const float y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);
      auto * point_data = msg->data.data() + point_count * msg->point_step;
      std::memcpy(point_data + msg->fields[0].offset, &x, sizeof(x));
      std::memcpy(point_data + msg->fields[1].offset, &y, sizeof(y));
      std::memcpy(point_data + msg->fields[2].offset, &z, sizeof(z));
      ++point_count;
    }
  }

  msg->width = static_cast<uint32_t>(point_count);
  msg->row_step = msg->width * msg->point_step;
  msg->data.resize(msg->row_step);

  return msg;
}

}  // namespace zed2i_driver
