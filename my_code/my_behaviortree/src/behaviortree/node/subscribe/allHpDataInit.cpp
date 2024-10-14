#include "node/subscribe/allHpDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

allHpDataInit::allHpDataInit(
    const std::string &name,
    const BT::NodeConfiguration &conf,
    const BT::RosNodeParams &params)
    : BT::RosTopicSubNode<behaviortree_interfaces::msg::AllRobotHP>(name, conf, params)
{
    std::cout << "hpIntFloatDataInit works" << std::endl;
}

BT::PortsList allHpDataInit::providedPorts()
{
    BT::PortsList ports_list;
    ports_list.insert(BT::InputPort<behaviortree_interfaces::msg::AllRobotHP>("message"));
    return ports_list;
}

BT::NodeStatus allHpDataInit::onTick(const std::shared_ptr<behaviortree_interfaces::msg::AllRobotHP>& msg)
{
    if(msg == nullptr)
    {
        return BT::NodeStatus::FAILURE;
    }
    setOutput("message", *msg);
    return BT::NodeStatus::SUCCESS;
}
