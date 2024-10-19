#include "node/condition/is_friend_ok.hpp"

namespace behaviortree
{
    IsFriendOk::IsFriendOk(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SimpleConditionNode(name, std::bind(&IsFriendOk::CheckFriendStatus, this), config)
    {
    }

    BT::PortsList IsFriendOk::providedPorts()
    {
        BT::PortsList ports_list;
        ports_list.insert(BT::InputPort<int>("team_color"));
        ports_list.insert(BT::InputPort<int>("all_robot_hp"));
        return ports_list;
    }

    BT::NodeStatus IsFriendOk::CheckFriendStatus()
    {
        int team_average_hp = 0;
        int enemy_average_hp = 0;
        auto all_robot_hp = getInput<behaviortree_interfaces::msg::AllRobotHP>("all_robot_hp");
        auto team_color = getInput<std::string>("team_color");

        if (!all_robot_hp)
        {
            throw BT::RuntimeError("missing required input [all_robot_hp]: ", all_robot_hp.error());
            return BT::NodeStatus::FAILURE;
        }
        if (!team_color)
        {
            throw BT::RuntimeError("missing required input [team_color]: ", team_color.error());
            return BT::NodeStatus::FAILURE;
        }

        if (team_color.value() != "red" && team_color.value() != "blue")
        {
            std::cerr << "Unknown color: " << team_color->c_str() << '\n';
            return BT::NodeStatus::FAILURE;
        }

        if (team_color.value() == "red")
        {
            calculateAverageHP(all_robot_hp.value(), team_average_hp, enemy_average_hp);
        }
        else
        {
            calculateAverageHP(all_robot_hp.value(), enemy_average_hp, team_average_hp);
        }
        // clang-format on

        if (team_average_hp > enemy_average_hp)
        {
            // std::cout << "我方血量优势" << '\n';
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            // std::cout << "我方血量劣势" << '\n';
            return BT::NodeStatus::FAILURE;
        }
    }

    void IsFriendOk::calculateAverageHP(const behaviortree_interfaces::msg::AllRobotHP &msg, int &redHp, int &blueHp)
    {
        redHp = (msg.red_1_robot_hp + msg.red_3_robot_hp + msg.red_4_robot_hp + msg.red_7_robot_hp) / 4;
        blueHp = (msg.blue_1_robot_hp + msg.blue_3_robot_hp + msg.blue_4_robot_hp + msg.blue_7_robot_hp) / 4;
    }

}