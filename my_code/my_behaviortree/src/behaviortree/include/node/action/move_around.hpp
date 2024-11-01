#pragma once

#include <vector>
#include <rclcpp/node.hpp>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace behaviortree
{
    class MoveAround : public BT::StatefulActionNode,rclcpp::Node
    {
    public:
        MoveAround(const std::string &name, const BT::NodeConfiguration &config);
        static BT::PortsList providedPorts();

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;

        void generatePoints(
            std::vector<double> location, double distance,
            std::vector<double> &nearby_random_point);


    private:
        int goal_count_;
        int expected_move_goal_count_;
        float expected_distance_;
        std::vector<double> position_;
        std::vector<double> nearby_random_point_;
    };
}
