# pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>

#include "behaviortree_interfaces/msg/all_robot_hp.hpp"

namespace behaviortree
{
    class IsOutpostOk : public BT::SimpleConditionNode
    {
    public:
    IsOutpostOk(
        const std::string& name,
        const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus CheckOutpostStatus();
    };
}