#include "../include/move.hpp"

#include <iostream>

int main()
{
    BT::BehaviorTreeFactory factory;
    Robot myRobot;
    Obs myObs;
    myObs.Set_target(2, 5);

    // 注册条件节点
    factory.registerSimpleCondition("isXArrived", std::bind(&Obs::x_is_arrive, &myObs, std::ref(myRobot.position.x)));
    factory.registerSimpleCondition("isYArrived", std::bind(&Obs::y_is_arrive, &myObs, std::ref(myRobot.position.y)));

    // 注册动作节点
    factory.registerSimpleAction("moveX", std::bind(&Robot::move_X, &myRobot, std::ref(myObs)));
    factory.registerSimpleAction("moveY", std::bind(&Robot::move_Y, &myRobot, std::ref(myObs)));
    factory.registerSimpleAction("PrintPosition", std::bind(PrintPosition, std::ref(myRobot), std::ref(myObs)));

    // todo: 通过xml文本创建一个行为树
    auto tree = factory.createTreeFromFile("../src/mytree.xml");
    while (true)
    {
        // * 树运行
        BT::NodeStatus status = tree.tickOnce();
        std::cout << "Tree status: " << status << std::endl;
        if (status == BT::NodeStatus::SUCCESS)
        {
            std::cout<<"robot arrived target"<<std::endl;
            return 0;
        }
        else
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 等待100毫秒
        }
        std::cout << "------------------" << std::endl;
    }
}
