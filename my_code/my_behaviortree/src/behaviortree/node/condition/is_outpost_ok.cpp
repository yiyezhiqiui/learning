#include "node/condition/is_outpost_ok.hpp"
#include "node/subscribe/gameDataInit.hpp"

namespace behaviortree
{
    IsOutpostOk::IsOutpostOk(const std::string &name, const BT::NodeConfiguration &config)
        : SimpleConditionNode(name, std::bind(&IsOutpostOk::CheckOutpostStatus, this), config)
    {
    }

    BT::PortsList IsOutpostOk::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<behaviortree_interfaces::msg::AllRobotHP>("all_robot_hp"));
        ports_list.insert(BT::InputPort<int>("team_color"));
        ports_list.insert(BT::InputPort<int>("outpost_hp_threshold"));
        return ports_list;
    }

    BT::NodeStatus IsOutpostOk::CheckOutpostStatus()
    {
        auto all_robot_hp = getInput<behaviortree_interfaces::msg::AllRobotHP>("all_robot_hp");
        auto team_color = getInput<int>("team_color");
        auto outpost_hp_threshold = getInput<int>("outpost_hp_threshold");
        if (!all_robot_hp)
        {
            throw BT::RuntimeError("missing required input [all_robot_hp]: ", all_robot_hp.error());
            return BT::NodeStatus::FAILURE;
        }
        if (!team_color)
        {
            throw BT::RuntimeError("missing required input [team_color]: ", team_color.error());
            return BT::NodeStatus::FAILURE;
        }
        if (!outpost_hp_threshold)
        {
            throw BT::RuntimeError("missing required input [outpost_hp_threshold]: ", outpost_hp_threshold.error());
            return BT::NodeStatus::FAILURE;
        }
        
        int outpost_hp = 0;
        outpost_hp = (team_color.value() == TeamColor::RED)
                         ? all_robot_hp->red_outpost_hp
                         : all_robot_hp->blue_outpost_hp;

        if (outpost_hp <= outpost_hp_threshold.value())
        {
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            return BT::NodeStatus::SUCCESS;
        }
    }
}