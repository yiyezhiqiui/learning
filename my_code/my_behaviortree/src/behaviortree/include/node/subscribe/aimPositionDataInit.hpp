#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>

namespace behaviortree
{
    // 云台手指定坐标初始化
    class aimPositionDataInit : public BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>
    {
    public:
        aimPositionDataInit(
            const std::string &name,
            const BT::NodeConfig &conf,
            const BT::RosNodeParams &params);
        static BT::PortsList providedPorts();
        BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray> &msg) override;

    private:
        // 云台手指定的坐标
        std::vector<float> aim_position_;
    };

}