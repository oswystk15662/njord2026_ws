#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <thread>
#include <chrono>
#include <atomic>

class USBCameraDriverMJPGNode : public rclcpp::Node
{
public:
    USBCameraDriverMJPGNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~USBCameraDriverMJPGNode();
private:
    // Parameters
    std::string camera_device_;
    int image_width_;
    int image_height_;
    int framerate_;
    std::string camera_frame_id_;
    std::string image_topic_name_;
    std::string camera_info_topic_name_;

    // OpenCV
    cv::VideoCapture cap_;

    // ROS 2 Publishers and Messages
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    // Thread for camera capture
    std::thread capture_thread_;
    std::atomic<bool> is_running_; // Flag to control the thread loop

    // Thread function
    void capture_and_publish();
};