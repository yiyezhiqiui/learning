#include "../include/tree.hpp"

int main()
{
  BT::BehaviorTreeFactory factory;

  CrossDoor cross_door;
  cross_door.registerNodes(factory);

  // In this example a single XML contains multiple <BehaviorTree>
  // To determine which one is the "main one", we should first register
  // the XML and then allocate a specific tree, using its ID

  auto tree = factory.createTreeFromFile("src/mytree.xml");

  // helper function to print the tree
  printTreeRecursively(tree.rootNode());

  tree.tickWhileRunning();

  return 0;
}

