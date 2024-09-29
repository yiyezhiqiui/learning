#include <rclcpp/client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_util/geometry_utils.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/detail/float32_multi_array__struct.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class goalClient: public rclcpp::Node {
public:
    goalClient();

private:
    void goal_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void startNavigation(geometry_msgs::msg::PoseStamped pose);

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_goal_handle_;
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr goal_sub;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_action_client;
};