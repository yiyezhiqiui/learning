#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

// Action_A has a different constructor than the default one.
class Action_A: public BT::SyncActionNode
{

public:
    // additional arguments passed to the constructor
    Action_A(const std::string& name, const BT::NodeConfig& config,
             int arg_int, std::string arg_str):
        SyncActionNode(name, config),
        arg1_(arg_int),
        arg2_(arg_str) {}

    // this example doesn't require any port
    static BT::PortsList providedPorts() { return {}; }

    // tick() can access the private members
    BT::NodeStatus tick() override;

private:
    int arg1_;
    std::string arg2_;
};

class Action_B: public BT::SyncActionNode
{

public:
    // The constructor looks as usual.
    Action_B(const std::string& name, const BT::NodeConfig& config):
        BT::SyncActionNode(name, config) {}

    // We want this method to be called ONCE and BEFORE the first tick()
    void initialize(int arg_int, const std::string& arg_str)
    {
        _arg1 = arg_int;
        _arg2 = arg_str;
    }

    // this example doesn't require any port
    static BT::PortsList providedPorts() { return {}; }

    // tick() can access the private members
    BT::NodeStatus tick() override;

private:
    int _arg1;
    std::string _arg2;
};