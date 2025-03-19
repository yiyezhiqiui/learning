#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

class SaySomething : public BT::SyncActionNode {
public:
  SaySomething(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};
class ThinkWhatToSay : public BT::SyncActionNode {
public:
  ThinkWhatToSay(const std::string &name, const BT::NodeConfig &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
};