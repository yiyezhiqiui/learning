#include "node/subscribe/gameDataInit.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    gameDataInit::gameDataInit(
        const std::string &name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicSubNode<std_msgs::msg::Int32MultiArray>(name, conf, params)
    {
        std::cout << "gameDataInit works" << std::endl;
    }

    BT::PortsList gameDataInit::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::OutputPort<int>("game_time"));
        ports_list.insert(BT::OutputPort<int>("game_economy"));
        ports_list.insert(BT::OutputPort<int>("sentry_heat"));
        ports_list.insert(BT::OutputPort<int>("remaining_ammunition"));
        ports_list.insert(BT::OutputPort<int>("team_color"));
        return ports_list;
    }

    BT::NodeStatus gameDataInit::onTick(const std::shared_ptr<std_msgs::msg::Int32MultiArray> &msg)
    {
        if (msg == nullptr)
        {
            return BT::NodeStatus::FAILURE;
        }
        game_time_ = (msg->data)[0];
        game_economy_ = (msg->data)[1];
        sentry_heat_ = (msg->data)[2];
        remaining_ammunition_ = (msg->data)[3];
        team_color_ = (msg->data)[4];
        
        setOutput("game_time", game_time_);
        setOutput("game_economy", game_economy_);
        setOutput("sentry_heat", sentry_heat_);
        setOutput("remaining_ammunition", remaining_ammunition_);
        setOutput("team_color", team_color_);
        return BT::NodeStatus::SUCCESS;
    }
}