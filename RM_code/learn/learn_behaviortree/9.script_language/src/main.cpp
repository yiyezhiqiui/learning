#include <behaviortree_cpp/bt_factory.h>

// file that contains the custom nodes definitions
#include "../include/tree.hpp"

enum Color
{
    RED = 1,
    BLUE = 2,
    GREEN = 3
};

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<SaySomething>("SaySomething");

    factory.registerScriptingEnums<Color>();
    //factory.registerScriptingEnum("THE_ANSWER", 42);

    auto tree = factory.createTreeFromFile("src/mytree.xml");
    tree.tickWhileRunning();
    return 0;
}

