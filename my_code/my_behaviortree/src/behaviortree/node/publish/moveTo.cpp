#include "node/publish/moveTo.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    moveTo::moveTo(
        const std::string &name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicPubNode<std_msgs::msg::Float32MultiArray>(name, conf, params)
    {
        std::cout << "moveTo works!!!" << std::endl;
    }

    BT::PortsList moveTo::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<float>("destination_location_x"));
        ports_list.insert(BT::InputPort<float>("destination_location_y"));
        return ports_list;
    }

    bool moveTo::setMessage(std_msgs::msg::Float32MultiArray &msg)
    {
        auto destination_location_x = getInput<float>("destination_location_x");
        if (!destination_location_x)
        {
            throw BT::RuntimeError("missing required input [destination_location_x]: ",destination_location_x.error());
            return false;
        }
        auto destination_location_y = getInput<float>("destination_location_y");
        if (!destination_location_y)
        {
            throw BT::RuntimeError("missing required input [destination_location_y]: ",destination_location_y.error());
            return false;
        }
        std::vector<float> data;
        data.push_back(destination_location_x.value());
        data.push_back(destination_location_y.value());
        msg.data = data;
        return true;
    }
}
