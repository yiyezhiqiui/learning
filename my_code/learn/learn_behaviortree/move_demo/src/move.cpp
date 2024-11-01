#include "../include/move.hpp"

BT::NodeStatus Robot::move_X(Obs obs)
{
    if (position.x < obs.target.x)
    {
        std::cout << "Moving along the x axis: " << position.x << std::endl;
        position.x += x_step;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

BT::NodeStatus Robot::move_Y(Obs obs)
{
    if (position.y < obs.target.y)
    {
        std::cout << "Moving along the y axis: " << position.y << std::endl;
        position.y += y_step;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

void Obs::Set_target(int x, int y)
{
    target.x = x;
    target.y = y;
    std::cout << "Target set successfully" << std::endl;
}

BT::NodeStatus Obs::x_is_arrive(int x_now)
{
    if (x_now < target.x)
    {
        std::cout << "X not arrived" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        std::cout << "X arrived" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
}

BT::NodeStatus Obs::y_is_arrive(int y_now)
{
    if (y_now < target.y)
    {
        std::cout << "Y not arrived" << std::endl;
        return BT::NodeStatus::FAILURE;
    }
    else
    {
        std::cout << "Y arrived" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
}

BT::NodeStatus PrintPosition(Robot &robot, Obs &obs)
{
    if (robot.position.x == obs.target.x && robot.position.y == obs.target.y)
    {
        std::cout << "Current position: " << robot.position.x << ", " << robot.position.y << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        std::cout << "Current position: " << robot.position.x << ", " << robot.position.y << std::endl;
        return BT::NodeStatus::FAILURE;
    }
}