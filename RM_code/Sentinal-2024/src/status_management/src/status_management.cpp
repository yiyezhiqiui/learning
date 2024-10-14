#include "status_management/status_management.h"

StatusManagement::StatusManagement():
    Node("status_management") {
    // 初始化状态
    this->mode_ = 0;
    this->nav_status_ = 0;
    this->goal_x_ = -1;
    this->goal_y_ = -1;
    this->ignore_classes_ = { "negative" };
    // 配置订阅者和发布者
    this->move_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("behaviortree/move", 10, std::bind(&StatusManagement::moveCallback, this, std::placeholders::_1));
    this->auto_aim_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("communicate/autoaim", 10, std::bind(&StatusManagement::autoAimCallback, this, std::placeholders::_1));
    this->position_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("communicate/position/exrobot", 10, std::bind(&StatusManagement::positionCallback, this, std::placeholders::_1));
    this->ignore_classes_sub_ = this->create_subscription<communicate::msg::IgnoreClasses>("/detector/ignore_classes", 10, std::bind(&StatusManagement::ignoreClassesCallback, this, std::placeholders::_1));
    this->guard_status_pub_ = this->create_publisher<communicate::msg::GuardStatus>("management/guardstatus", 10);
};

StatusManagement::~StatusManagement() {
    // 释放资源
    this->move_sub_.reset();
    this->auto_aim_sub_.reset();
    this->position_sub_.reset();
    this->ignore_classes_sub_.reset();
    this->guard_status_pub_.reset();
};

void StatusManagement::positionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (abs(msg->data[2] - this->goal_x_) < 0.1 && abs(msg->data[3] - this->goal_y_) < 0.1) {
        this->nav_status_ = 0;
    }
    this->publishGuardStatus();
}

void StatusManagement::moveCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    this->goal_x_ = msg->data[0];
    this->goal_y_ = msg->data[1];
    this->nav_status_ = 1;
}

void StatusManagement::autoAimCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    this->mode_ = msg->data[1];
    if (this->mode_ == 1) {
        this->ignore_classes_ = { "negative" };
    }
}

void StatusManagement::ignoreClassesCallback(const communicate::msg::IgnoreClasses::SharedPtr msg) {
    this->ignore_classes_ = msg->ignore_classes;
}

void StatusManagement::publishGuardStatus() {
    communicate::msg::GuardStatus msg;
    msg.mode = this->mode_;
    msg.nav_status = this->nav_status_;
    msg.ignore_classes = this->ignore_classes_;
    this->guard_status_pub_->publish(msg);
}