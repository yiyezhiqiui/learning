#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

//初始化游戏数据
class gameDataInit : public BT::RosTopicSubNode<std_msgs::msg::Int32MultiArray> {
public:
    gameDataInit(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Int32MultiArray>& msg) override;

private:
    int game_time;             // 游戏时间
    int game_economy;          // 游戏经济
    int state_purchase;        // 发射机构状态
    int remaining_ammunition;  // 剩余弹丸数量
};



