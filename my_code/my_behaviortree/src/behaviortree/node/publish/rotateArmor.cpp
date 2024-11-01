#include "node/publish/rotateArmor.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

namespace behaviortree
{
    rotateArmor::rotateArmor(
        const std::string &name,
        const BT::NodeConfig &conf,
        const BT::RosNodeParams &params)
        : BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray>(name, conf, params)
    {
        std::cout << "rotateArmor works!!!" << std::endl;
    }

    BT::PortsList rotateArmor::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<int>("gimbal_spin"));
        return ports_list;
    }

    bool rotateArmor::setMessage(std_msgs::msg::Int32MultiArray &msg)
    {
        auto gimbal_spin = getInput<int>("gimbal_spin");
        if (!gimbal_spin)
        {
            throw BT::RuntimeError("missing required input [gimbal_spin]: ", gimbal_spin.error());
            return false;
        }
        std::vector<int> data;
        if (gimbal_spin.value() == true)
        {
            data.push_back(1);
        }
        else
        {
            data.push_back(0);
        }
        msg.data = data;
        return true;
    }
}
