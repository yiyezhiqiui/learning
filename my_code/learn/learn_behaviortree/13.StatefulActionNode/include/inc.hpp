#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>

class numcount : public BT::StatefulActionNode
{
public:
    numcount(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("num")};
    }
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
private:
    int num_count;
    int num;
};

class SaySomething : public BT::SyncActionNode
{
public:
    SaySomething(const std::string &name, const BT::NodeConfig &config);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
private:
    int count=0;
};

#endif // RM_BEHAVIOR_TREE__PLUGINS__ACTION__MOVE_AROUND_HPP_