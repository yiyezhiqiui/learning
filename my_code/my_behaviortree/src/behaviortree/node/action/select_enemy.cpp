#include "node/action/select_enemy.hpp"

namespace behaviortree
{
    SelectEnemy::SelectEnemy(const std::string& name, const BT::NodeConfiguration& config) :
        BT::SyncActionNode(name, config)
    {
    }
    BT::PortsList SelectEnemy::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<std::vector<behaviortree_interfaces::msg::EnemyInformation>>("enemy_list"));
        return ports_list;
    }
}//namespace behaviortree