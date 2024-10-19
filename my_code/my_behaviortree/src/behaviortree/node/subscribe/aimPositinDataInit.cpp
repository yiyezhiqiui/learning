#include "node/subscribe/aimPositionDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    aimPositionDataInit::aimPositionDataInit(
        const std::string &name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>(name, conf, params)
    {
        std::cout << "aimPositionDataInit works" << std::endl;
    }

    BT::PortsList aimPositionDataInit::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::OutputPort<std::vector<float>>("aim_position"));
        return ports_list;
    }

    BT::NodeStatus aimPositionDataInit::onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray> &msg)
    {
        if (msg == nullptr)
        {
            return BT::NodeStatus::FAILURE;
        }
        aim_position_[0] = (msg->data)[0];
        aim_position_[1] = (msg->data)[1];
        setOutput("aim_position",aim_position_);
        return BT::NodeStatus::SUCCESS;
    }

}