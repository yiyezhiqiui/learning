#ifndef _STATUS_MANAGEMENT_H_
#define _STATUS_MANAGEMENT_H_

#include <cmath>
#include <communicate/msg/detail/guard_status__struct.hpp>
#include <communicate/msg/detail/ignore_classes__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

#include "communicate/msg/guard_status.hpp"
#include "communicate/msg/ignore_classes.hpp"

class StatusManagement: public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     */
    StatusManagement();

    /**
     * @brief 析构函数
     */
    ~StatusManagement();

    /**
     * @brief 订阅当前位置话题回调函数
     */
    void positionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅导航目的地话题回调函数
     */
    void moveCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅自瞄模式切换话题回调函数
     */
    void autoAimCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    /**
     * @brief 订阅忽略装甲板类型话题回调函数
     */
    void ignoreClassesCallback(const communicate::msg::IgnoreClasses::SharedPtr msg);

    /**
     * @brief 发布哨兵状态
     */
    void publishGuardStatus();

private:
    int mode_;                                //自瞄模式 0：自瞄 1：符
    int nav_status_;                          //导航状态 0：未运行 1：导航中
    double goal_x_;                           //导航目的地x坐标 -1：无
    double goal_y_;                           //导航目的地y坐标 -1：无
    std::vector<std::string> ignore_classes_; //忽略装甲板类型

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr move_sub_;          //导航目的地订阅者
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr auto_aim_sub_;        //自瞄模式切换订阅者
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr position_sub_;      //当前位置订阅者
    rclcpp::Subscription<communicate::msg::IgnoreClasses>::SharedPtr ignore_classes_sub_; //忽略装甲板类型订阅者

    rclcpp::Publisher<communicate::msg::GuardStatus>::SharedPtr guard_status_pub_; //哨兵状态发布者
};
#endif