#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>

#include "behaviortree_interfaces/msg/all_robot_hp.hpp"

namespace behaviortree
{
    // 血量数据初始化
    class allHpDataInit : public BT::RosTopicSubNode<behaviortree_interfaces::msg::AllRobotHP>
    {
    public:
        allHpDataInit(
            const std::string &name,
            const BT::NodeConfig &conf,
            const BT::RosNodeParams &params);
        static BT::PortsList providedPorts();

        // behaviortree_interfaces::msg::AllRobotHP  自定义消息类型 存放双方的血量信息
        BT::NodeStatus onTick(const std::shared_ptr<behaviortree_interfaces::msg::AllRobotHP> &msg) override;
    };
}