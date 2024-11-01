#include "node/condition/is_detect_enemy.hpp"

namespace behaviortree
{
    IsDeteceEnemy::IsDeteceEnemy(const std::string &name, const BT::NodeConfiguration &config) 
    : BT::SimpleConditionNode(name,std::bind(IsDeteceEnemy::DetectEnemyStatus,this) ,config)
    {
    }

    BT::PortsList IsDeteceEnemy::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<int>("enemy_status"));
        return ports_list;
    }
    BT::NodeStatus IsDeteceEnemy::DetectEnemyStatus()
    {
        auto enemy_status = getInput<int>("enemy_status");
        if(!enemy_status)
        {
            throw BT::RuntimeError("error reading port [enemy_status]:", enemy_status.error());
            return BT::NodeStatus::FAILURE;
        }
        return enemy_status.value() == 0 ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }
}