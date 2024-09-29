#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
class Obs;

struct Pointxy
{
    int x;
    int y;
};

class Robot
{
public:
    int x_step=1;
    int y_step=1;
    Pointxy position={0,0};
    BT::NodeStatus move_X(Obs obs);
    BT::NodeStatus move_Y(Obs obs);
};
class Obs
{
    public:
    Pointxy target={0,0};
    void Set_target(int x,int y);
    BT::NodeStatus x_is_arrive(int x_now);
    BT::NodeStatus y_is_arrive(int y_now);

};

BT::NodeStatus PrintPosition(Robot& robot,Obs& obs);