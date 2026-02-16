#include "arducam_ros2_cpp/stereo.hpp"


ArduCamNode::ArduCamNode() 
  : Node("arducam_node")
    {
        // Parameters
        this->declare_parameter<std::string>("left_cam_info_file", "left_cam_info.yaml");
        this->declare_parameter<std::string>("right_cam_info_file", "right_cam_info.yaml");
        this->declare_parameter<int>("width", 1920);
        this->declare_parameter<int>("height", 1080);
        this->declare_parameter<std::string>("frame_id", "cam0");

        left_cam_info_file  = this->get_parameter("left_cam_info_file").as_string();
        right_cam_info_file = this->get_parameter("right_cam_info_file").as_string();
        width  = this->get_parameter("width").as_int();
        height = this->get_parameter("height").as_int();
        frame_id = this->get_parameter("frame_id").as_string();

        pub_caminfo = true;

        RCLCPP_INFO(this->get_logger(), "Initiating video feed.");
        cap.open("udp://172.17.0.1:8080", cv::CAP_FFMPEG);
        RCLCPP_INFO(this->get_logger(), "Video feed successfully initiated.");

        load_camera_info();

        // Publishers
        left_cam_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "left/camera_info", rclcpp::SensorDataQoS());
        right_cam_info_pub = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "right/camera_info", rclcpp::SensorDataQoS());

        left_img_pub = this->create_publisher<sensor_msgs::msg::Image>(
            "left/image_raw", rclcpp::SensorDataQoS());
        right_img_pub = this->create_publisher<sensor_msgs::msg::Image>(
            "right/image_raw", rclcpp::SensorDataQoS());

        double fps = 15.0;
        timer = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / fps)),
            std::bind(&ArduCamNode::run, this));
    }

void ArduCamNode::load_camera_info()
  {
      try {
          YAML::Node left = YAML::LoadFile(left_cam_info_file);
          YAML::Node right = YAML::LoadFile(right_cam_info_file);

          fill_camera_info(left, left_cam_info_msg);
          fill_camera_info(right, right_cam_info_msg);

          RCLCPP_INFO(this->get_logger(), "Loaded camera info files.");
        }
      catch (...) {
          RCLCPP_WARN(this->get_logger(), "Could not load camera parameters");
          pub_caminfo = false;
        }
  }

void ArduCamNode::fill_camera_info(const YAML::Node &node,
    sensor_msgs::msg::CameraInfo &msg)    
{
    msg.width = node["image_width"].as<int>();
    msg.height = node["image_height"].as<int>();
    msg.distortion_model = node["distortion_model"].as<std::string>();

    auto k = node["camera_matrix"]["data"].as<std::vector<double>>();
    std::copy(k.begin(), k.end(), msg.k.begin());

    msg.d = node["distortion_coefficients"]["data"].as<std::vector<double>>();

    auto r = node["rectification_matrix"]["data"].as<std::vector<double>>();
    std::copy(r.begin(), r.end(), msg.r.begin());

    auto p = node["projection_matrix"]["data"].as<std::vector<double>>();
    std::copy(p.begin(), p.end(), msg.p.begin());
}
  
void ArduCamNode::run()
{
    cv::Mat frame;
    if (!cap.read(frame))
        return;

    auto capture_time = this->now();

    std::string encoding =
        (frame.channels() >= 3) ? "bgr8" : "mono8";

    int width = frame.cols;
    int height = frame.rows;

    cv::Mat left_img  = frame(cv::Rect(0, 0, width / 2, height));
    cv::Mat right_img = frame(cv::Rect(width / 2, 0, width / 2, height));

    auto left_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), encoding, left_img).toImageMsg();
    auto right_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), encoding, right_img).toImageMsg();

    left_msg->header.frame_id = frame_id;
    right_msg->header.frame_id = frame_id;
    left_msg->header.stamp = capture_time;
    right_msg->header.stamp = capture_time;

    left_img_pub->publish(*left_msg);
    right_img_pub->publish(*right_msg);

    if (pub_caminfo) {
        left_cam_info_msg.header.stamp = capture_time;
        right_cam_info_msg.header.stamp = capture_time;
        left_cam_info_msg.header.frame_id = frame_id;
        right_cam_info_msg.header.frame_id = frame_id;

        left_cam_info_pub->publish(left_cam_info_msg);
        right_cam_info_pub->publish(right_cam_info_msg);
    }
  }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArduCamNode>();
  RCLCPP_INFO(node->get_logger(), "arducam_node has started");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
