#ifndef SERVER_HPP
#define SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces_ros2/srv/interface.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class ServerWay : public rclcpp::Node
{
public:
    ServerWay();
    void handle_request(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                        std::shared_ptr<interfaces_ros2::srv::Interface::Response> response);
    void send_img(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                  std::shared_ptr<interfaces_ros2::srv::Interface::Response> response);
    void send_video(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                    std::shared_ptr<interfaces_ros2::srv::Interface::Response> response);
    void send_camera(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                    std::shared_ptr<interfaces_ros2::srv::Interface::Response> response);
    void send_transform(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                    std::shared_ptr<interfaces_ros2::srv::Interface::Response> response);

private:
    rclcpp::Service<interfaces_ros2::srv::Interface>::SharedPtr service_;
    std::unordered_map<int, cv::VideoCapture> video_captures_; // 保存每个视频的捕获状态

    int cnt_ = 0;
};

#endif // SERVER_HPP
