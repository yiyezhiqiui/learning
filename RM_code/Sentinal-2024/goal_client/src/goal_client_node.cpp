#include <goal_client/goal_client.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<goalClient>());
    rclcpp::shutdown();
}