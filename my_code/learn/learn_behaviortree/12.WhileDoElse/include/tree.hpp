#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

class SaySomething : public BT::SyncActionNode
{
public:
    SaySomething(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class JudgeMent :  public BT::SyncActionNode
{
    public:
    JudgeMent(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class SaySomethingInt : public BT::SyncActionNode
{
public:
    SaySomethingInt(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class Init : public BT::SyncActionNode
{
    public:
    Init(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class RateController : public BT::DecoratorNode
{
public:
  RateController(const std::string & name, const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts() { return {BT::InputPort<double>("hz", 10.0, "Rate")}; }

private:
  BT::NodeStatus tick() override;

  std::chrono::time_point<std::chrono::high_resolution_clock> start_;
  double period_;
  bool first_time_;
};
