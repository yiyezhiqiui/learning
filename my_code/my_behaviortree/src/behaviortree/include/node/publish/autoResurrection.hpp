#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>

namespace behaviortree
{
    // 自动复活
    class autoResurrection : public BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray>
    {
    public:
        autoResurrection(
            const std::string &name,
            const BT::NodeConfig &conf,
            const BT::RosNodeParams &params);
        static BT::PortsList providedPorts();
        bool setMessage(std_msgs::msg::Int32MultiArray &msg);
    };

}
