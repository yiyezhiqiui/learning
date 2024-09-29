#include "communicate/downlink.hpp"

#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>

Downlink::Downlink(Uplink* uplink)
    : Node("communicate_subscription"),
      uplink_(uplink) {
    this->LeftPTZSubTopic = "/shoot_info/left";
    this->RightPTZSubTopic = "/shoot_info/right";
    this->MainPTZSubTopic = "/behaviortree/main";
    this->ChassisVelSubTopic = "cmd_vel";
    this->InteractionSubTopic = "/behaviortree/interaction";
    this->MoudleSubTopic = "/behaviortree/moudle";

    this->debug = this->declare_parameter("debug", false);

    LeftPTZ_sub = this->create_subscription<communicate::msg::SerialInfo>(
        LeftPTZSubTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&Downlink::LeftPTZCB, this, std::placeholders::_1));
    RightPTZ_sub = this->create_subscription<communicate::msg::SerialInfo>(
        RightPTZSubTopic,
        rclcpp::SensorDataQoS(),
        std::bind(&Downlink::RightPTZCB, this, std::placeholders::_1));
    MainPTZ_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        MainPTZSubTopic,
        10,
        std::bind(&Downlink::MainPTZCB, this, std::placeholders::_1));
    ChassisVel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        ChassisVelSubTopic,
        10,
        std::bind(&Downlink::ChassisVelCB, this, std::placeholders::_1));
    Interaction_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        InteractionSubTopic,
        10,
        std::bind(&Downlink::InteractionCB, this, std::placeholders::_1));
    Moudle_sub = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        MoudleSubTopic,
        10,
        std::bind(&Downlink::MoudleCB, this, std::placeholders::_1));
}

void Downlink::LeftPTZCB(const communicate::msg::SerialInfo::SharedPtr msg) {
    if (this->debug) {
        return;
    }
    LeftptzControlBuffer tmp;
    tmp.find_bool = msg->is_find.data;
    tmp.yaw = msg->euler[0];
    tmp.pitch = msg->euler[1];
    tmp.distance = msg->distance;
    uplink_->Send(0xA0, &tmp);
}

void Downlink::RightPTZCB(const communicate::msg::SerialInfo::SharedPtr msg) {
    if (this->debug) {
        return;
    }
    RightptzControlBuffer tmp;
    tmp.find_bool = msg->is_find.data;
    tmp.yaw = msg->euler[0];
    tmp.pitch = msg->euler[1];
    tmp.distance = msg->distance;
    uplink_->Send(0xA1, &tmp);
}

void Downlink::MainPTZCB(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (this->debug) {
        return;
    }
    MainptzControlBuffer tmp;
    tmp.yaw = msg->data[0];
    uplink_->Send(0xA2, &tmp);
}

void Downlink::ChassisVelCB(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (this->debug) {
        return;
    }
    ChassisControlBuffer tmp;
    tmp.vx = msg->linear.x;
    tmp.vy = msg->linear.y;
    uplink_->Send(0xA3, &tmp);
}

void Downlink::InteractionCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (this->debug) {
        return;
    }
    InteractionControlBuffer tmp;
    tmp.type = msg->data[0];
    tmp.content = msg->data[1];
    uplink_->Send(0xA4, &tmp);
}

void Downlink::MoudleCB(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (this->debug) {
        return;
    }
    MoudleControlBuffer tmp;
    tmp.type = msg->data[0];
    tmp.content = msg->data[1];
    uplink_->Send(0xA5, &tmp);
}