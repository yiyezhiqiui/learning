#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>

namespace behaviortree
{
    // 哨兵坐标初始化
    class sentryPositionDataInit : public BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>
    {
    public:
        sentryPositionDataInit(
            const std::string &name,
            const BT::NodeConfig &conf,
            const BT::RosNodeParams &params);
        static BT::PortsList providedPorts();
        BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray> &msg) override;

    private:
        // 哨兵当前位置
        std::vector<float> sentry_position_;
    };
}
