#include "node/subscribe/sentryPositionDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    sentryPositionDataInit::sentryPositionDataInit(
        const std::string &name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray>(name, conf, params)
    {
        std::cout << "sentryPositionDataInit works" << std::endl;
    }

    BT::PortsList sentryPositionDataInit::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::OutputPort<std::vector<float>>("sentry_position"));
        return ports_list;
    }

    BT::NodeStatus sentryPositionDataInit::onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray> &msg)
    {
        if (msg == nullptr)
        {
            return BT::NodeStatus::FAILURE;
        }
        sentry_position_[0]= msg->data[0];
        sentry_position_[1]= msg->data[1];
        setOutput("sentry_position", sentry_position_);
        return BT::NodeStatus::SUCCESS;
    }

}
