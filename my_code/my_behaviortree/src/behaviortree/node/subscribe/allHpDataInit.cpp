#include "node/subscribe/allHpDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    allHpDataInit::allHpDataInit(
        const std::string &name,
        const BT::NodeConfiguration &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicSubNode<behaviortree_interfaces::msg::AllRobotHP>(name, conf, params)
    {
        std::cout << "allHpDataInit works" << std::endl;
    }

    BT::PortsList allHpDataInit::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::OutputPort<behaviortree_interfaces::msg::AllRobotHP>("all_robot_hp"));
        return ports_list;
    }

    BT::NodeStatus allHpDataInit::onTick(const std::shared_ptr<behaviortree_interfaces::msg::AllRobotHP> &msg)
    {
        if (msg == nullptr)
        {
            return BT::NodeStatus::FAILURE;
        }
        setOutput("all_robot_hp", *msg);
        return BT::NodeStatus::SUCCESS;
    }
}
