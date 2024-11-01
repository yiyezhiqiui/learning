#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace behaviortree
{
    class IsGameTime : public BT::SimpleConditionNode
    {
    public:
    IsGameTime(const std::string &name, const BT::NodeConfiguration &config);
    BT::PortsList providedPorts();
    BT::NodeStatus CheckGameTime();
    };
}