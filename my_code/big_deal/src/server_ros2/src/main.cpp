#include "rclcpp/rclcpp.hpp"
#include "../include/server.hpp"
#include <thread>

class Server : public rclcpp::Node
{
public:
    Server() : Node("server_work"), server_(std::make_shared<ServerWay>())
    {
        // 创建处理图像请求的服务
        server_img = this->create_service<interfaces_ros2::srv::Interface>(
            "get_request",
            std::bind(&ServerWay::handle_request, server_.get(), std::placeholders::_1, std::placeholders::_2));

    }

    ~Server()
    {
    }

private:
    std::shared_ptr<ServerWay> server_;

    // 创建服务变量
    rclcpp::Service<interfaces_ros2::srv::Interface>::SharedPtr server_img;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto server_node = std::make_shared<Server>();

    // 在主线程中 spin 以处理主线程中的回调
    rclcpp::spin(server_node);

    rclcpp::shutdown();
    return 0;
}
