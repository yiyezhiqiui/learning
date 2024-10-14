#include "../include/inc.hpp"

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<numcount>("numcount");
    factory.registerNodeType<SaySomething>("SaySomething");

    auto tree = factory.createTreeFromFile("src/mytree.xml");

    auto blackboard = BT::Blackboard::create();

    // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
    std::cout << "--- ticking1\n";
    auto status = tree.tickOnce();
    std::cout << "--- status1:外面 " << toStr(status) << "\n\n";

    int i = 0;
    while (1)
    {
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.

        tree.sleep(std::chrono::milliseconds(1000));

        std::cout << "--- ticking\n";
        status = tree.tickOnce();
        std::cout << "--- status1: " <<status << "\n\n";
        i++;
    }

    return 0;
}