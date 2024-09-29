#include "../include/tree.hpp"
// Saysometing
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
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// ThinkWhatToSay
ThinkWhatToSay::ThinkWhatToSay(const std::string &name, const BT::NodeConfig &config)
    : SyncActionNode(name, config)
{
}

BT::PortsList ThinkWhatToSay::providedPorts()
{
    return {BT::OutputPort<std::string>("text")};
}

BT::NodeStatus ThinkWhatToSay::tick()
{
    // the output may change at each tick(). Here we keep it simple.
    setOutput("text", "The answer is 42");
    return BT::NodeStatus::SUCCESS;
}