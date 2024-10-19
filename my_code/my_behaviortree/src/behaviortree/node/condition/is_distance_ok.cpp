#include "node/condition/is_distance_ok.hpp"
#include "behaviortree_interfaces/msg/first_attack_enemy.hpp"

namespace behaviortree
{
    IsDistanceOK::IsDistanceOK(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SimpleConditionNode(name,std::bind(IsDistanceOK::CheckDistance,this) ,config)
    {
    }
    
    BT::PortsList IsDistanceOK::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<behaviortree_interfaces::msg::FirstAttackEnemy>("first_attack_enemy"));
        ports_list.insert(BT::InputPort<int>("low_distance"));
        ports_list.insert(BT::InputPort<int>("high_distance"));
        return ports_list;
    }
    BT::NodeStatus IsDistanceOK::CheckDistance()
    {
        auto first_attack_enemy = getInput<behaviortree_interfaces::msg::FirstAttackEnemy>("first_attack_enemy");
        if(!first_attack_enemy)
        {
            throw BT::RuntimeError("missing required input [first_attack_enemy]: ", first_attack_enemy.error());
            return BT::NodeStatus::FAILURE;
        }
        auto low_distance = getInput<int>("low_distance");
        if(!low_distance)
        {
            throw BT::RuntimeError("missing required input [low_distance]: ", low_distance.error());
            return BT::NodeStatus::FAILURE;
        }
        auto high_distance = getInput<int>("high_distance");
        if(!high_distance)
        {
            throw BT::RuntimeError("missing required input [high_distance]: ", high_distance.error());
            return BT::NodeStatus::FAILURE;
        }

        if(first_attack_enemy->distance>=low_distance.value() &&first_attack_enemy->distance<=high_distance.value())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
}