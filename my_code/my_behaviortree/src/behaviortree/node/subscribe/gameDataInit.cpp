#include "node/subscribe/gameDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

gameDataInit::gameDataInit(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Int32MultiArray>(name, conf, params) 
{
    std::cout << "gIntDataInit works" << std::endl;
}

BT::PortsList gameDataInit::providedPorts() 
{
    BT::PortsList ports_list;
    ports_list.insert(BT::OutputPort<int>("game_time"));
    ports_list.insert(BT::OutputPort<int>("game_economy"));
    ports_list.insert(BT::OutputPort<int>("state_purchase"));
    ports_list.insert(BT::OutputPort<int>("remaining_ammunition"));
    return ports_list;
}

BT::NodeStatus gameDataInit::onTick(const std::shared_ptr<std_msgs::msg::Int32MultiArray>& msg) 
{
    if (msg == nullptr) {
        return BT::NodeStatus::FAILURE;
    }
    game_time = (msg->data)[0];
    game_economy = (msg->data)[1];
    state_purchase = (msg->data)[5];
    remaining_ammunition = (msg->data)[6];
    setOutput("game_time", game_time);
    setOutput("game_economy", game_economy);
    setOutput("state_purchase", state_purchase);
    setOutput("remaining_ammunition", remaining_ammunition);
    return BT::NodeStatus::SUCCESS;
}