#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <stdio.h>
// We want to use this custom type
struct Position2D
{
  double x;
  double y;
};

// Template specialization to converts a string to Position2D.
namespace BT
{
  template <>
  inline Position2D BT::convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
      throw RuntimeError("invalid input)");
    }
    else
    {
      Position2D output;
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      return output;
    }
  }
} // end namespace BT

class CalculateGoal : public BT::SyncActionNode
{
public:
  CalculateGoal(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<Position2D>("goal")};
  }

  BT::NodeStatus tick() override
  {
    Position2D mygoal = {1.1, 2.3};
    setOutput<Position2D>("goal", mygoal);
    return BT::NodeStatus::SUCCESS;
  }
};

class PrintTarget : public BT::SyncActionNode
{
public:
  PrintTarget(const std::string &name, const BT::NodeConfig &config) : SyncActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    // Optionally, a port can have a human readable description
    const char *description = "Simply print the goal on console...";
    return {BT::InputPort<Position2D>("target", description)};
  }

  BT::NodeStatus tick() override
  {
    auto res = getInput<Position2D>("target");
    if (!res)
    {
      throw BT::RuntimeError("error reading port [target]:", res.error());
    }
    Position2D target = res.value();
    printf("Target positions: [ %.1f, %.1f ]\n", target.x, target.y);
    return BT::NodeStatus::FAILURE;
  }
};