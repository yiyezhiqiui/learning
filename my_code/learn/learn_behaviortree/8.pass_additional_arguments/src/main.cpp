#include "../include/tree.hpp"

int main()
{
    BT::BehaviorTreeFactory factory;

    // Register as usual, but we still need to initialize
    factory.registerNodeType<Action_B>("Action_B");

    // Create the whole tree. Instances of Action_B are not initialized yet
    auto tree = factory.createTreeFromText("src/mytree.xml");

    // visitor will initialize the instances of
    auto visitor = [](BT::TreeNode *node)
    {
        if (auto action_B_node = dynamic_cast<Action_B *>(node))
        {
            action_B_node->initialize(69, "interesting_value");
        }
    };

    // Apply the visitor to ALL the nodes of the tree
    tree.applyVisitor(visitor);
}