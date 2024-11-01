#ifndef CLIENTWAY_HPP
#define CLIENTWAY_HPP

#include "rclcpp/rclcpp.hpp"
#include "interfaces_ros2/srv/interface.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <fstream>

class ClientWay : public rclcpp::Node
{
public:

    ClientWay(std::shared_ptr<rclcpp::Client<interfaces_ros2::srv::Interface>> client);
    void send_request(std::string type,int nameID,std::string from,std::string to);//发送请求
    void handle_img(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future);//处理图片
    void handle_video(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future);//处理视频
    void handle_transform(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future);//处理平移和位移向量
    void handle_camera(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future);//处理相机内参和畸变系数
    void get_input();//读取终端输入

private:
    std::shared_ptr<rclcpp::Client<interfaces_ros2::srv::Interface>> client_;
    std::atomic<bool> new_requested_;
    int current_video_id_;

    double gimtranslation_[3];
    double gimrotation_[4];
    double camtranslation_[3];
    double camrotation_[4];
};

#endif // CLIENTWAY_HPP
