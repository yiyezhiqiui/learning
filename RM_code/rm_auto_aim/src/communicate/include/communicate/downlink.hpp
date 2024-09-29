#ifndef DOWNLINK_HPP_
#define DOWNLINK_HPP_

#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription_base.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>

#include "communicate/msg/serial_info.hpp"
#include "communicate/uplink.hpp"

class Downlink : public rclcpp::Node {
public:
    Downlink(Uplink* uplink);

    /**
     * @brief 订阅左小云台控制话题回调函数
     */
    void LeftPTZCB(const communicate::msg::SerialInfo::SharedPtr msg);

    /**
     * @brief 订阅右小云台控制话题回调函数
     */
    void RightPTZCB(const communicate::msg::SerialInfo::SharedPtr msg);

    /**
     * @brief 订阅大云台控制话题回调函数
     */
    void MainPTZCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅底盘速度控制话题回调函数
     */
    void ChassisVelCB(const geometry_msgs::msg::Twist::SharedPtr msg);

    /**
     * @brief 订阅比赛交互控制话题回调函数
     */
    void InteractionCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅车体模块控制话题回调函数
     */
    void MoudleCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

private:
    Uplink* uplink_;
    bool debug;  // 是否开启调试模式

    std::string LeftPTZSubTopic;      // 左小云台控制订阅话题
    std::string RightPTZSubTopic;     // 右小云台控制订阅话题
    std::string MainPTZSubTopic;      // 大云台控制订阅话题
    std::string ChassisVelSubTopic;   // 底盘控制订阅话题
    std::string InteractionSubTopic;  // 比赛交互控制订阅话题
    std::string MoudleSubTopic;       // 车体模块控制订阅话题

    rclcpp::Subscription<communicate::msg::SerialInfo>::SharedPtr LeftPTZ_sub;
    rclcpp::Subscription<communicate::msg::SerialInfo>::SharedPtr RightPTZ_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr MainPTZ_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr ChassisVel_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr Interaction_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr Moudle_sub;
};
#endif
