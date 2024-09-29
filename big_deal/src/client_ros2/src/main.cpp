#include "../include/client.hpp"

class Client : public rclcpp::Node
{
public:
    Client() : Node("client_work")
    {
        // 初始化客户端，用于发送请求
        client_ = this->create_client<interfaces_ros2::srv::Interface>("get_request");

        // 实例化 ClientWay 并传入客户端
        client_way_ = std::make_shared<ClientWay>(client_);

        // 在独立线程中运行 get_input
        input_thread_ = std::thread(&ClientWay::get_input, client_way_);
    }

    ~Client()
    {
        // 在析构函数中等待输入线程完成
        if (input_thread_.joinable())
        {
            input_thread_.join();
        }
    }

private:
    rclcpp::Client<interfaces_ros2::srv::Interface>::SharedPtr client_;
    std::shared_ptr<ClientWay> client_way_;  // 保存 ClientWay 的实例
    std::thread input_thread_;  // 用于运行 get_input 的独立线程
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Client>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
