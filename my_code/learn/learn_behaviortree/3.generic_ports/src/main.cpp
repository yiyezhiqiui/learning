#include "../include/tree.hpp"

int main()
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<PrintTarget>("PrintTarget");

    // todo: 通过xml文本创建一个行为树
    auto tree = factory.createTreeFromFile("src/mytree.xml");
    while (true)
    {
        // * 树运行
        BT::NodeStatus status = tree.tickOnce();
        std::cout << "Tree status: " << status << std::endl;
        if (status == BT::NodeStatus::SUCCESS)
        {
            std::cout << "calcuate complicated" << std::endl;
            return 0;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待100毫秒
        }
        std::cout << "------------------" << std::endl;
    }

    return 0;
}