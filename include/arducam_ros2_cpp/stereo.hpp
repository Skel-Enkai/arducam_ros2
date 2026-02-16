#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class ArduCamNode : public  rclcpp::Node 
{
  public:
    ArduCamNode();
  
  private:
    std::string left_cam_info_file;
    std::string right_cam_info_file;
    std::string frame_id;
    int width;
    int height;
    bool pub_caminfo;

    cv::VideoCapture cap;

    sensor_msgs::msg::CameraInfo left_cam_info_msg;
    sensor_msgs::msg::CameraInfo right_cam_info_msg;

    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_cam_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_cam_info_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_img_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_img_pub;

    rclcpp::TimerBase::SharedPtr timer;

    void load_camera_info();
    void fill_camera_info(const YAML::Node &node,
                          sensor_msgs::msg::CameraInfo &msg);
    void run();
};



