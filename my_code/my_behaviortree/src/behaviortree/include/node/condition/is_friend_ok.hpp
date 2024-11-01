# pragma once

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp/condition_node.h>

#include "behaviortree_interfaces/msg/all_robot_hp.hpp"

namespace behaviortree
{
    class IsFriendOk : public BT::SimpleConditionNode
    {
        IsFriendOk(
            const std::string& name, 
            const BT::NodeConfiguration& config);
        static BT::PortsList providedPorts();
        BT::NodeStatus CheckFriendStatus();
        void calculateAverageHP(const behaviortree_interfaces::msg::AllRobotHP &msg,int& redHp,int& blueHp);
    };
}