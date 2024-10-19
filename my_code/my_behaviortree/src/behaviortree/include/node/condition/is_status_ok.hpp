#pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>

#include "behaviortree_interfaces/msg/sentry_status.hpp"

namespace behaviortree
{
    class IsStatusOk : public BT::SimpleConditionNode
    {
    public:
        IsStatusOk(
            const std::string &name,
            const BT::NodeConfiguration &config);
        static BT::PortsList providedPorts();
        BT::NodeStatus CheckSentryStatus();
    };
}