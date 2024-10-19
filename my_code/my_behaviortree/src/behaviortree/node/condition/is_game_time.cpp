#include "node/condition/is_game_time.hpp"

namespace behaviortree
{
    IsGameTime::IsGameTime(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SimpleConditionNode(name,std::bind(IsGameTime::CheckGameTime,this) ,config)
    {
    }
    BT::PortsList IsGameTime::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<int>("game_time"));
        ports_list.insert(BT::InputPort<int>("low_remain_time"));
        ports_list.insert(BT::InputPort<int>("high_remain_time"));
        return ports_list;
    }
    BT::NodeStatus IsGameTime::CheckGameTime()
    {
        auto game_time = getInput<int>("game_time");
        if (!game_time)
        {
            throw BT::RuntimeError("missing required input [game_time]: ", game_time.error());
            return BT::NodeStatus::FAILURE;
        }
        auto low_remain_time = getInput<int>("low_remain_time");
        if (!low_remain_time)
        {
            throw BT::RuntimeError("missing required input [low_remain_time]: ", low_remain_time.error());
            return BT::NodeStatus::FAILURE;
        }
        auto high_remain_time = getInput<int>("high_remain_time");
        if (!high_remain_time)
        {
            throw BT::RuntimeError("missing required input [high_remain_time]: ", high_remain_time.error());
            return BT::NodeStatus::FAILURE;
        }

        if (game_time.value() <= high_remain_time.value() && game_time.value() >= low_remain_time.value())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
}