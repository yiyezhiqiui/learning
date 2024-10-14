#ifndef _BEHAVIORTREE_H_
#define _BEHAVIORTREE_H_

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/tree_node.h>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_options.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/detail/int32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <unistd.h>
#include <vector>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"

//初始化游戏数据
class gIntDataInit : public BT::RosTopicSubNode<std_msgs::msg::Int32MultiArray> {
public:
    gIntDataInit(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Int32MultiArray>& msg) override;

private:
    int game_time;             // 游戏时间
    int game_economy;          // 游戏经济
    int state_outpost;         // 前哨站血量
    int state_guard;           // 哨兵血量
    int state_base;            // 基地血量
    int state_purchase;        // 发射机构状态
    int remaining_ammunition;  // 剩余弹丸数量
};

//位置初始化 待完善
class alFloatDataInit : public BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray> {
public:
    alFloatDataInit(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray>& msg) override;

private:
    float artificial_location_x;
    float artificial_location_y;
};

//位置初始化 待完善
class pFloatDataInit : public BT::RosTopicSubNode<std_msgs::msg::Float32MultiArray> {
public:
    pFloatDataInit(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    BT::NodeStatus onTick(const std::shared_ptr<std_msgs::msg::Float32MultiArray>& msg) override;

private:
    float position_x;
    float position_y;
};

//自动复活
class autoResurrection : public BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray> {
public:
    autoResurrection(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    bool setMessage(std_msgs::msg::Int32MultiArray& msg);
};

//自动购买弹药
class autoPurchase : public BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray> {
public:
    autoPurchase(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    bool setMessage(std_msgs::msg::Int32MultiArray& msg);
};

//前哨站血量判断
class stateOutpostJudgement : public BT::SyncActionNode {
public:
    stateOutpostJudgement(const std::string& name, const BT::NodeConfig& conf);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};


class rotateArmor : public BT::RosTopicPubNode<std_msgs::msg::Int32MultiArray> {
public:
    rotateArmor(
        const std::string& name,
        const BT::NodeConfig& conf,
        const BT::RosNodeParams& params);
    bool setMessage(std_msgs::msg::Int32MultiArray& msg);
};

//发射机构状态判断
class purchaseJudgement : public BT::SyncActionNode {
public:
    purchaseJudgement(const std::string& name, const BT::NodeConfig& conf);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

//获取指定位置
class moveTo : public BT::RosTopicPubNode<std_msgs::msg::Float32MultiArray> {
public:
    moveTo(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);
    static BT::PortsList providedPorts();
    bool setMessage(std_msgs::msg::Float32MultiArray& msg);
};

//哨兵血量判断
class guardJudgement : public BT::SyncActionNode {
public:
    guardJudgement(const std::string& name, const BT::NodeConfig& conf);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

//位置判断 待完善
class artificialControlJudgement : public BT::SyncActionNode {
public:
    artificialControlJudgement(const std::string& name, const BT::NodeConfig& conf);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

//游戏时间判断
class gameTimeJudgement : public BT::SyncActionNode {
public:
    gameTimeJudgement(const std::string& name, const BT::NodeConfig& conf);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
#endif