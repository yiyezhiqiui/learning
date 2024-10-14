#include "../include/tree.hpp"

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<JudgeMent>("JudgeMent");
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<SaySomethingInt>("SaySomethingInt");
    factory.registerNodeType<RateController>("RateController");
    factory.registerNodeType<Init>("Init");

    auto tree = factory.createTreeFromFile("src/mytree.xml");

    auto blackboard = BT::Blackboard::create();

    // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
    std::cout << "--- ticking1\n";
    auto status = tree.tickOnce();
    std::cout << "--- status1: " << toStr(status) << "\n\n";

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
        std::cout << "--- "<<status << "\n\n";
        i++;
    }

    return 0;
}