#include "node/action/move_around.hpp"

#include <random>
#include <rclcpp/clock.hpp>
#include <chrono>

namespace behaviortree
{
    MoveAround::MoveAround(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config), Node("move_around_node")
    {
    }

    BT::PortsList MoveAround::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<std::vector<double>>("position"));
        ports_list.insert(BT::InputPort<int>("expected_move_goal_count"));
        ports_list.insert(BT::InputPort<int>("expected_distance"));
        return ports_list;
    }
    BT::NodeStatus MoveAround::onStart()
    {
        position_[0] = 0;
        position_[1] = 0;

        expected_move_goal_count_ = 0;
        expected_distance_ = 0;
        goal_count_ = 0;

        if (!getInput("position", position_))
        {
            return BT::NodeStatus::FAILURE;
        }
        // 获取参数：期望的点位数量
        if (!getInput("expected_move_goal_count", expected_move_goal_count_))
        {
            // std::cout << "missing required input [expected_nearby_goal_count]" << '\n';
            return BT::NodeStatus::FAILURE;
        }

        // 获取参数：期望的距离
        if (!getInput("expected_distance", expected_distance_))
        {
            // std::cout << "missing required input [expected_dis]" << '\n';
            return BT::NodeStatus::FAILURE;
        }

        if (expected_move_goal_count_ <= 0)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }
    BT::NodeStatus MoveAround::onRunning()
    {

        if (expected_move_goal_count_ == goal_count_)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            goal_count_++;
            generatePoints(position_, expected_distance_, nearby_random_point_);
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 这是不太文明的做法...
            return BT::NodeStatus::RUNNING;
        }
    }
    void MoveAround::onHalted()
    {
        // nothing to do here...
        // std::cout << "MoveAroundAction interrupted" << '\n';
    }
    void MoveAround::generatePoints(
        std::vector<double> location, double distance,
        std::vector<double> &nearby_random_point)
    {
        // 创建随机数生成器
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(0, 2 * M_PI);

        // 生成随机角度
        double angle = dis(gen);
        nearby_random_point[0] = location[0] + distance * sin(angle);
        nearby_random_point[1] = location[1] + distance * cos(angle);
    }
}