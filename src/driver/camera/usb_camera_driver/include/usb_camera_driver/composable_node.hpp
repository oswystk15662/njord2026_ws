#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp> // Required for composable nodes
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp> // For cv::VideoCapture
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>

#include <string>
#include <thread> // For running capture in a separate thread
#include <chrono> // For std::chrono

static GstFlowReturn on_new_sample(GstElement *sink, gpointer user_data);

class USBCameraDriverNode : public rclcpp::Node
{
public:
    // Constructor accepts rclcpp::NodeOptions
    USBCameraDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~USBCameraDriverNode();

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_publisher_;

private:
    std::string camera_device_;
    int image_width_;
    int image_height_;
    int framerate_;
    std::string camera_frame_id_;
    std::string compressed_image_topic_name_;
    std::string camera_info_topic_name_;
    std::string gstreamer_pipeline_;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    std::thread capture_thread_; // Thread for camera capture

    void capture_and_publish();
    
};

