#include "../include/tree.hpp"

// SaySomething
SaySomething::SaySomething(const std::string &name, const BT::NodeConfig &config)
    : SyncActionNode(name, config)
{
}

BT::PortsList SaySomething::providedPorts()
{
    // This action has two input ports: "message" and "test_out"
    return {BT::InputPort<std::string>("message"),
            BT::InputPort<std::string>("test_out"),
            BT::InputPort<std::string>("test_in", "789")};
}

BT::NodeStatus SaySomething::tick()
{
    BT::Expected<std::string> msg = getInput<std::string>("message");
    BT::Expected<std::string> msg_test = getInput<std::string>("test_out");
    BT::Expected<std::string> msg_test_in = getInput<std::string>("test_in");

    // 检查 message 端口是否有值
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message]: ", msg.error());
    }

    // 检查 test_out 端口是否有值
    if (!msg_test)
    {
        std::cerr << "Warning: missing optional input [test_out]: " << msg_test.error() << std::endl;
        msg_test = "default"; // 使用默认值或其他处理方式
    }

    // 直接从黑板中获取 test_in 的值，自动使用默认值
   if (!msg_test_in)
    {
        std::cerr << "Error: unable to get input or default value for [test_in]: " << msg_test_in.error() << std::endl;
        throw BT::RuntimeError("Error: unable to get input or default value for [test_in]");
    }


    // 正常输出已获取的输入
    std::cout << "Robot says: " << msg.value() << std::endl;
    std::cout << "Robot says: " << msg_test.value() << std::endl;
    std::cout << "Robot says moren: " << msg_test_in.value() << std::endl;

    return BT::NodeStatus::SUCCESS;
}
// ThinkWhatToSay
ThinkWhatToSay::ThinkWhatToSay(const std::string &name, const BT::NodeConfig &config)
    : SyncActionNode(name, config)
{
}

BT::PortsList ThinkWhatToSay::providedPorts()
{
    // This action has two output ports: "text" and "test"
    return {BT::OutputPort<std::string>("text"), BT::OutputPort<std::string>("test")};
}

BT::NodeStatus ThinkWhatToSay::tick()
{
    // The output may change at each tick(). Here we keep it simple.
    setOutput("text", "The answer is 42");
    setOutput("test", "TEST");

    return BT::NodeStatus::SUCCESS;
}