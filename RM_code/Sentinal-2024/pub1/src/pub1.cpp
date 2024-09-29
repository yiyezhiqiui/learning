#include <chrono>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>

using namespace std::chrono_literals;
class Pub1: public rclcpp::Node {
public:
    Pub1(): Node("pub1_node") {
        pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("behaviortree/move", 10);
        timer_ = this->create_wall_timer(1000ms, std::bind(&Pub1::timer_callback, this));
    }

private:
    void timer_callback() {
        std::vector<float> data;
        data.push_back(6.0);
        data.push_back(6.0);
        std_msgs::msg::Float32MultiArray msg;
        msg.data = data;
        pub->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Pub1>());
    rclcpp::shutdown();
}