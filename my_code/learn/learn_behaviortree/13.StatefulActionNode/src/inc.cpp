#include "inc.hpp"

numcount::numcount(const std::string &name, const BT::NodeConfiguration &config)
    : BT::StatefulActionNode(name, config)
{
}

BT::NodeStatus numcount::onStart()
{
    num_count = 0;
    return BT::NodeStatus::RUNNING;
}

void numcount::onHalted()
{
    // nothing to do here...
    // std::cout << "MoveAroundAction interrupted" << '\n';
}

BT::NodeStatus numcount::onRunning()
{
    std::cout << "num_count: " << num_count << std::endl;
    if (num_count <= 5)
    {
        num_count++;
        return BT::NodeStatus::RUNNING;
    }
    else
    {
        return BT::NodeStatus::SUCCESS;
    }
}

// SaySometing
SaySomething::SaySomething(const std::string &name, const BT::NodeConfig &config)
    : SyncActionNode(name, config)
{
}

BT::PortsList SaySomething::providedPorts()
{
    // This action has a single input port called "message"
    return {BT::InputPort<std::string>("message")};
}

BT::NodeStatus SaySomething::tick()
{
    BT::Expected<std::string> msg = getInput<std::string>("message");
    // Check if expected is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ",
                               msg.error());
    }
    // use the method value() to extract the valid message.
    std::cout<<"SaySomething count:  "<<count<<std::endl;
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}
