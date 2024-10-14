#include "node/subscribe/sentryPositionDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

sentryPositionDataInit::sentryPositionDataInit(
    const std::string& name,
    const BT::NodeConfig& conf,
    const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>(name, conf, params) 
{
    std::cout << "pFloatDataInit works" << std::endl;
}

BT::PortsList sentryPositionDataInit::providedPorts() 
{
    BT::PortsList ports_list;
    ports_list.insert(BT::OutputPort<float>("position_x"));
    ports_list.insert(BT::OutputPort<float>("position_y"));
    return ports_list;
}

BT::NodeStatus sentryPositionDataInit::onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray>& msg) 
{
    if (msg == nullptr) {
        return BT::NodeStatus::FAILURE;
    }
    position_x = (msg->data)[0];
    position_y = (msg->data)[1];
    setOutput("position_x", position_x);
    setOutput("position_y", position_y);
    return BT::NodeStatus::SUCCESS;
}
