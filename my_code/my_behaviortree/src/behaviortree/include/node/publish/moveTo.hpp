#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>

namespace behaviortree
{
    // 获取指定位置
    class moveTo : public BT::RosTopicPubNode<std_msgs::msg::Float32MultiArray>
    {
    public:
        moveTo(const std::string &name, const BT::NodeConfig &conf, const BT::RosNodeParams &params);
        static BT::PortsList providedPorts();
        bool setMessage(std_msgs::msg::Float32MultiArray &msg);
    };
}
