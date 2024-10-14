#include "../include/tree.hpp"

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<SaySomething>("SaySomething");

    auto tree = factory.createTreeFromFile("src/mytree.xml");

    // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
    std::cout << "--- ticking1\n";
    auto status = tree.tickOnce();
    std::cout << "--- status1: " << toStr(status) << "\n\n";

    while (status == BT::NodeStatus::RUNNING)
    {
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(std::chrono::milliseconds(100));

        std::cout << "--- ticking\n";
        status = tree.tickOnce();
        std::cout << "--- status: " << toStr(status) << "\n\n";
    }

    return 0;
}