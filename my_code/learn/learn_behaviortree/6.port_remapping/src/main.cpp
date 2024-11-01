#include "../include/tree.hpp"
int main()
{
    BT::BehaviorTreeFactory factory;

    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MoveBaseAction>("MoveBase");

    auto tree = factory.createTreeFromFile("src/mytree.xml");
    
    // Keep ticking until the end
    tree.tickWhileRunning();

    // let's visualize some information about the current state of the blackboards.
    std::cout << "\n------ First BB ------" << std::endl;
    tree.subtrees[0]->blackboard->debugMessage();
    std::cout << "\n------ Second BB------" << std::endl;
    tree.subtrees[1]->blackboard->debugMessage();

    return 0;
}

/* Expected output:

------ First BB ------
move_result (std::string)
move_goal (Pose2D)

------ Second BB------
[result] remapped to port of parent tree [move_result]
[target] remapped to port of parent tree [move_goal]

*/