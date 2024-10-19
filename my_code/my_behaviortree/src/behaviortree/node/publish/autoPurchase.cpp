#include "node/publish/autoPurchase.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    autoPurchase::autoPurchase(
        const std::string &name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray>(name, conf, params)
    {
        std::cout << "autoPurchase works!!!" << std::endl;
    }

    BT::PortsList autoPurchase::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<int>("game_economy"));
        ports_list.insert(BT::InputPort<int>("remaining_ammunition"));
        return ports_list;
    }

    bool autoPurchase::setMessage(std_msgs::msg::Int32MultiArray &msg)
    {
        auto game_economy = getInput<int>("game_economy");
        if (!game_economy)
        {
            throw BT::RuntimeError("missing required input [game_economy]: ", game_economy.error());
            return false;
        }
        auto remaining_ammunition = getInput<int>("remaining_ammunition");
        if (!remaining_ammunition)
        {
            throw BT::RuntimeError("missing required input [remaining_ammunition]: ",remaining_ammunition.error());
            return false;
        }
        
        std::vector<int> data;
        if (game_economy.value() > 300 && remaining_ammunition.value() < 50)
        {
            data.push_back(2);
            data.push_back(50);
        }
        else
        {
            data.push_back(0);
            data.push_back(0);
        }
        msg.data = data;
        return true;
    }
}
