#include "usb_camera_driver/via_mjpg_streamer.hpp"

USBCameraDriverMJPGNode::USBCameraDriverMJPGNode(const rclcpp::NodeOptions& options)
: Node("usb_camera_driver_node", options), is_running_(true)
{
    RCLCPP_INFO(this->get_logger(), "USB Camera Driver Node has been started.");

    // Declare and get parameters
    this->declare_parameter<std::string>("camera_device", "/dev/video0");
    this->declare_parameter<int>("image_width", 640);
    this->declare_parameter<int>("image_height", 480);
    this->declare_parameter<int>("framerate", 30);
    this->declare_parameter<std::string>("camera_frame_id", "camera_link");
    this->declare_parameter<std::string>("image_topic_name", "image_raw");
    this->declare_parameter<std::string>("camera_info_topic_name", "camera_info");

    camera_device_ = this->get_parameter("camera_device").as_string();
    image_width_ = this->get_parameter("image_width").as_int();
    image_height_ = this->get_parameter("image_height").as_int();
    framerate_ = this->get_parameter("framerate").as_int();
    camera_frame_id_ = this->get_parameter("camera_frame_id").as_string();
    image_topic_name_ = this->get_parameter("image_topic_name").as_string();
    camera_info_topic_name_ = this->get_parameter("camera_info_topic_name").as_string();

    RCLCPP_INFO(this->get_logger(), "Camera Device: %s", camera_device_.c_str());
    RCLCPP_INFO(this->get_logger(), "Resolution: %dx%d", image_width_, image_height_);
    RCLCPP_INFO(this->get_logger(), "Framerate: %d fps", framerate_);
    RCLCPP_INFO(this->get_logger(), "Camera Frame ID: %s", camera_frame_id_.c_str());

    cap_.open(camera_device_);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera: %s", camera_device_.c_str());
        return;
    }
    
    // Set camera properties
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
    cap_.set(cv::CAP_PROP_FPS, framerate_);

    RCLCPP_INFO(this->get_logger(), "Camera opened successfully.");

    // QoS settings
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // Create publishers
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_name_, qos_profile);
    camera_info_publisher_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_name_, qos_profile);

    // Initialize camera info message
    camera_info_msg_.header.frame_id = camera_frame_id_;
    camera_info_msg_.width = image_width_;
    camera_info_msg_.height = image_height_;
    camera_info_msg_.k = {1.0, 0.0, static_cast<double>(image_width_) / 2.0,
                            0.0, 1.0, static_cast<double>(image_height_) / 2.0,
                            0.0, 0.0, 1.0};
    camera_info_msg_.p = {1.0, 0.0, static_cast<double>(image_width_) / 2.0, 0.0,
                            0.0, 1.0, static_cast<double>(image_height_) / 2.0, 0.0,
                            0.0, 0.0, 1.0, 0.0};
    camera_info_msg_.distortion_model = "plumb_bob";

    // Start camera capture in a separate thread
    capture_thread_ = std::thread(&USBCameraDriverMJPGNode::capture_and_publish, this);
}

USBCameraDriverMJPGNode::~USBCameraDriverMJPGNode()
{
    is_running_ = false; // Set flag to stop the thread loop
    if (capture_thread_.joinable()) {
        capture_thread_.join(); // Wait for the thread to finish
    }
    RCLCPP_INFO(this->get_logger(), "USB Camera Driver Node has been stopped.");
}

void USBCameraDriverMJPGNode::capture_and_publish()
{
    RCLCPP_INFO(this->get_logger(), "Starting capture loop.");

    cv::Mat frame;
    cv::Mat rgb_frame;
    rclcpp::Rate loop_rate(framerate_); // Control the loop rate to match the framerate

    while (rclcpp::ok() && is_running_) {
        if (!cap_.read(frame)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to read a frame from the camera.");
            break;
        }

        // Convert the image from BGR to RGB (OpenCV default is BGR, ROS is RGB)
        cv::cvtColor(frame, rgb_frame, cv::COLOR_BGR2RGB);

        // Get the current time for the message header
        auto current_time = this->get_clock()->now();

        // Create the Image message using cv_bridge
        std_msgs::msg::Header header;
        header.stamp = current_time;
        header.frame_id = camera_frame_id_;
        auto image_msg = cv_bridge::CvImage(header, "rgb8", rgb_frame).toImageMsg();

        // Publish the Image message
        image_publisher_->publish(*image_msg);

        // Update and publish the CameraInfo message
        camera_info_msg_.header.stamp = current_time;
        camera_info_publisher_->publish(camera_info_msg_);

        loop_rate.sleep();
    }
    
    // Cleanup
    cap_.release();
    RCLCPP_INFO(this->get_logger(), "Capture loop has been stopped.");
}

RCLCPP_COMPONENTS_REGISTER_NODE(USBCameraDriverMJPGNode)