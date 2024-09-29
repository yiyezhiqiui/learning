#include "../include/tree.hpp"

// MoveBaseAction
BT::NodeStatus MoveBaseAction::onStart()
{
    if (!getInput<Pose2D>("goal", goal_))
    {
        throw BT::RuntimeError("missing required input [goal]");
    }
    printf("[ MoveBase: SEND REQUEST ]. goal: x=%f y=%f theta=%f\n",
           goal_.x, goal_.y, goal_.theta);

    // We use this counter to simulate an action that takes a certain
    // amount of time to be completed (200 ms)
    completion_time_ = chr::system_clock::now() + chr::milliseconds(220);

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseAction::onRunning()
{
    // Pretend that we are checking if the reply has been received
    // you don't want to block inside this function too much time.
    std::this_thread::sleep_for(chr::milliseconds(10));

    // Pretend that, after a certain amount of time,
    // we have completed the operation
    if (chr::system_clock::now() >= completion_time_)
    {
        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

void MoveBaseAction::onHalted()
{
    printf("[ MoveBase: ABORTED ]");
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
    std::cout << "Robot says: " << msg.value() << std::endl;
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CheckBattery()
{

  std::cout << "[ Battery: OK ]" << std::endl;
  return BT::NodeStatus::SUCCESS;
}