#include "node/subscribe/aimPositionDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

aimPositionDataInit::aimPositionDataInit(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>(name, conf, params) 
{
    std::cout << "alFloatDataInit works" << std::endl;
}

BT::PortsList aimPositionDataInit::providedPorts() 
{
    BT::PortsList ports_list;
    ports_list.insert(BT::OutputPort<float>("aim_position_x"));
    ports_list.insert(BT::OutputPort<float>("aim_position_y"));
    return ports_list;
}

BT::NodeStatus aimPositionDataInit::onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray>& msg) 
{
    if (msg == nullptr) {
        return BT::NodeStatus::FAILURE;
    }
    aim_position_x = (msg->data)[0];
    aim_position_y = (msg->data)[1];
    setOutput("aim_position_x", aim_position_x);
    setOutput("aim_position_y", aim_position_y);
    return BT::NodeStatus::SUCCESS;
}
