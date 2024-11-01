#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

// Custom type
struct Pose2D
{
    double x, y, theta;
};

namespace chr = std::chrono;

class MoveBaseAction : public BT::StatefulActionNode
{
public:
    // Any TreeNode with ports must have a constructor with this signature
    MoveBaseAction(const std::string &name, const BT::NodeConfig &config)
        : StatefulActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<Pose2D>("goal")};
    }

    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Pose2D goal_;
    chr::system_clock::time_point completion_time_;
};

class SaySomething : public BT::SyncActionNode
{
public:
    SaySomething(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

BT::NodeStatus CheckBattery();
namespace BT
{
    template <>
    inline Pose2D BT::convertFromString(StringView str)
    {
        // We expect real numbers separated by semicolons
        auto parts = splitString(str, ';');
        if (parts.size() != 3)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            Pose2D output;
            output.x = convertFromString<double>(parts[0]);
            output.y = convertFromString<double>(parts[1]);
            return output;
        }
    }
}