#include <rclcpp/executors.hpp>

#include "status_management/status_management.h"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<StatusManagement>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}