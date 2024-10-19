#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>


namespace behaviortree
{
    class IsDistanceOK : public BT::SimpleConditionNode
    {
    public:
        IsDistanceOK(const std::string &name, const BT::NodeConfiguration &config);
        static BT::PortsList providedPorts();
        BT::NodeStatus CheckDistance();
    };
}