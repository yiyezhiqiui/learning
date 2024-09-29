#include <behaviortree_cpp/bt_factory.h>

// file that contains the custom nodes definitions
#include "../include/tree.hpp"

int main()
{  
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("Saysomething");
  factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");

  auto blackboard = BT::Blackboard::create();
  auto tree = factory.createTreeFromFile("src/mytree.xml",blackboard);
  tree.tickWhileRunning();
  return 0;
}

/*  Expected output:
  Robot says: hello
  Robot says: The answer is 42
*/