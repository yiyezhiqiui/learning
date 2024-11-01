#include "node/publish/autoResurrection.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    autoResurrection::autoResurrection(
        const std::string &name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray>(name, conf, params)
    {
        std::cout << "autoResurrection works!!!" << std::endl;
    }

    BT::PortsList autoResurrection::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<int>("game_economy"));
        ports_list.insert(BT::InputPort<int>("game_time"));
        ports_list.insert(BT::InputPort<int>("state_guard"));
        return ports_list;
    }

    bool autoResurrection::setMessage(std_msgs::msg::Int32MultiArray &msg)
    {
        auto game_economy = getInput<int>("game_economy");
        if (!game_economy)
        {
            throw BT::RuntimeError("missing required input [game_economy]: ", game_economy.error());
            return false;
        }
        auto game_time = getInput<int>("game_time");
        if (!game_time)
        {
            throw BT::RuntimeError("missing required input [game_time]: ", game_time.error());
            return false;
        }
        auto state_guard = getInput<int>("state_guard");
        if (!state_guard)
        {
            throw BT::RuntimeError("missing required input [state_guard]: ", state_guard.error());
            return false;
        }

        std::vector<int> data;
        if (state_guard.value() == 0 && game_economy.value() > 500 && game_time.value() > 300)
        {
            data.push_back(1);
            data.push_back(1);
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
