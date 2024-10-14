#include "goal_client/goal_client.hpp"
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <nav2_msgs/action/detail/navigate_to_pose__struct.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>

goalClient::goalClient():
    Node("goal_client_node") {
    navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
    navigation_action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    goal_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "behaviortree/move",
        1,
        std::bind(&goalClient::goal_callback, this, std::placeholders::_1)
    );
}

void goalClient::goal_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.header.stamp = rclcpp::Clock().now();
    pose.header.frame_id = "map";
    pose.pose.position.x = msg->data[0];
    pose.pose.position.y = msg->data[1];
    pose.pose.position.z = 0.0;
    pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(msg->data[2]);

    std::cout << "Start navigation" << std::endl;
    startNavigation(pose);
}

void goalClient::startNavigation(geometry_msgs::msg::PoseStamped pose) {
    auto is_action_server_ready =
        navigation_action_client->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready) {
        RCLCPP_ERROR(
            this->get_logger(),
            "FollowWaypoints action server is not available."
            " Is the initial pose set?"
        );
        return;
    }

    navigation_goal_.pose = pose;

    auto send_goal_options =
        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [](auto) {};

    auto future_goal_handle =
        navigation_action_client->async_send_goal(navigation_goal_, send_goal_options);
    std::cout << "Publish End" << std::endl;   
}