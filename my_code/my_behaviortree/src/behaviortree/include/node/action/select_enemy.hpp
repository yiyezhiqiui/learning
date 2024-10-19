#pragma once 

#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/action_node.h>

#include "behaviortree_interfaces/msg/enemy_information.hpp"
#include "behaviortree_interfaces/msg/first_attack_enemy.hpp"

namespace behaviortree
{
    class SelectEnemy : public BT::SyncActionNode
    {
    public:
    SelectEnemy(const std::string& name, const BT::NodeConfiguration& config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    };
} // namespace behaviortree