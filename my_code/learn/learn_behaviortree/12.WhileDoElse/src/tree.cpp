#include "../include/tree.hpp"

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
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// JudgeMent
JudgeMent::JudgeMent(const std::string &name, const BT::NodeConfig &config)
    : SyncActionNode(name, config)
{
}

BT::PortsList JudgeMent::providedPorts()
{
    return {BT::InputPort<int>("judge_num")};
}

BT::NodeStatus JudgeMent::tick()
{
    BT::Expected<int> num = getInput<int>("judge_num");
    if (num == 1)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

// SaySometingInt
SaySomethingInt::SaySomethingInt(const std::string &name, const BT::NodeConfig &config)
    : SyncActionNode(name, config)
{
}

BT::PortsList SaySomethingInt::providedPorts()
{
    // This action has a single input port called "message"
    return {BT::InputPort<int>("message_num")};
}

BT::NodeStatus SaySomethingInt::tick()
{
    BT::Expected<int> msg = getInput<int>("message_num");
    // Check if expected is valid. If not, throw its error
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [message_num]: ",
                               msg.error());
    }
    // use the method value() to extract the valid message.
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

// Init
Init::Init(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config)
{
}

BT::PortsList Init::providedPorts()
{
    return {BT::InputPort<int>("judge_num_in"),
            BT::OutputPort<int>("judge_num")};
}

BT::NodeStatus Init::tick()
{
    BT::Expected<int> msg = getInput<int>("judge_num_in");
    if (!msg)
    {
        throw BT::RuntimeError("missing required input [judge_num_in]: ",
                               msg.error());
    }
    setOutput("judge_num", msg.value());
    return BT::NodeStatus::SUCCESS;
}


//RateController
RateController::RateController(const std::string & name, const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf), first_time_(false)
{
    double hz = 1.0;
    getInput("hz", hz);
    period_ = 1.0 / hz;
}

BT::NodeStatus RateController::tick()
{
    if (status() == BT::NodeStatus::IDLE) {
        start_ = std::chrono::high_resolution_clock::now();
        first_time_ = true;
        std::cout << "RateController: First tick or reset" << std::endl;
    }

    setStatus(BT::NodeStatus::RUNNING);

    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = now - start_;
    typedef std::chrono::duration<float> float_seconds;
    auto seconds = std::chrono::duration_cast<float_seconds>(elapsed);

    if (first_time_ || (child_node_->status() == BT::NodeStatus::RUNNING) ||
        seconds.count() >= period_) {
        first_time_ = false;
        std::cout << "RateController: Executing child tick" << std::endl;
        const BT::NodeStatus child_state = child_node_->executeTick();

        switch (child_state) {
            case BT::NodeStatus::RUNNING:
                return BT::NodeStatus::RUNNING;

            case BT::NodeStatus::SUCCESS:
                start_ = std::chrono::high_resolution_clock::now();  // Reset the timer
                std::cout << "RateController: Child succeeded" << std::endl;
                return BT::NodeStatus::SUCCESS;

            case BT::NodeStatus::FAILURE:
            default:
                std::cout << "RateController: Child failed" << std::endl;
                return BT::NodeStatus::FAILURE;
        }
    }

    return status();
}
