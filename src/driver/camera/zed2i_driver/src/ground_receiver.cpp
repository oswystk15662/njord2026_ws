#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

namespace
{

class GroundReceiver : public rclcpp::Node
{
public:
  GroundReceiver()
  : Node("ground_receiver")
  {
    port_ = declare_parameter<int>("port", 5600);
    jitter_latency_ms_ = declare_parameter<int>("jitter_latency_ms", 50);
    topic_ = declare_parameter<std::string>("topic", "/ground_video/image/compressed");

    if (port_ < 1 || port_ > 65535 || jitter_latency_ms_ < 0 || topic_.empty()) {
      throw std::invalid_argument("invalid ground-video receiver configuration");
    }

    publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      topic_, rclcpp::SensorDataQoS());

    gst_init(nullptr, nullptr);
    GError * error = nullptr;
    const std::string pipeline_description =
      "udpsrc port=" + std::to_string(port_) +
      " caps=application/x-rtp,media=video,encoding-name=JPEG,payload=26"
      " ! rtpjitterbuffer latency=" + std::to_string(jitter_latency_ms_) +
      " drop-on-latency=true"
      " ! rtpjpegdepay"
      " ! appsink name=sink emit-signals=true sync=false max-buffers=1 drop=true";
    pipeline_ = gst_parse_launch(pipeline_description.c_str(), &error);
    if (error != nullptr || pipeline_ == nullptr) {
      const std::string message = error == nullptr ? "unknown GStreamer error" : error->message;
      if (error != nullptr) {
        g_error_free(error);
      }
      throw std::runtime_error("could not create ground-video receiver pipeline: " + message);
    }

    appsink_ = GST_APP_SINK(gst_bin_get_by_name(GST_BIN(pipeline_), "sink"));
    if (appsink_ == nullptr) {
      throw std::runtime_error("could not find ground-video appsink element");
    }
    g_signal_connect(appsink_, "new-sample", G_CALLBACK(&GroundReceiver::on_new_sample), this);

    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
      throw std::runtime_error("could not start ground-video receiver pipeline");
    }
    RCLCPP_INFO(
      get_logger(), "Receiving RTP/JPEG on UDP port %d and publishing locally to %s",
      port_, topic_.c_str());
  }

  ~GroundReceiver() override
  {
    if (pipeline_ != nullptr) {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
    }
    if (appsink_ != nullptr) {
      gst_object_unref(appsink_);
    }
    if (pipeline_ != nullptr) {
      gst_object_unref(pipeline_);
    }
  }

private:
  static GstFlowReturn on_new_sample(GstAppSink * sink, gpointer user_data)
  {
    auto * receiver = static_cast<GroundReceiver *>(user_data);
    GstSample * sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr) {
      return GST_FLOW_ERROR;
    }

    GstBuffer * buffer = gst_sample_get_buffer(sample);
    GstMapInfo map{};
    if (buffer == nullptr || !gst_buffer_map(buffer, &map, GST_MAP_READ)) {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    sensor_msgs::msg::CompressedImage message;
    message.header.stamp = receiver->get_clock()->now();
    message.format = "jpeg";
    message.data.assign(map.data, map.data + map.size);
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
    receiver->publisher_->publish(std::move(message));
    return GST_FLOW_OK;
  }

  int port_{};
  int jitter_latency_ms_{};
  std::string topic_;
  GstElement * pipeline_{nullptr};
  GstAppSink * appsink_{nullptr};
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_;
};

}  // namespace

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundReceiver>());
  rclcpp::shutdown();
  return 0;
}
