#include <cstdint>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <unistd.h>

#include "communicate/downlink.hpp"
#include "communicate/uplink.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    uint32_t baud_rate = 115200; // 波特率

    drivers::serial_driver::FlowControl fc = drivers::serial_driver::FlowControl ::NONE;
    drivers::serial_driver::Parity pt = drivers::serial_driver::Parity::NONE;
    drivers::serial_driver::StopBits sb = drivers::serial_driver::StopBits::ONE;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

    Uplink uplink(*device_config);
    //communicate.init(*device_config);
    auto node = std::make_shared<Downlink>(&uplink);

    std::thread thread_recv([&uplink] {
        while (rclcpp::ok()) {
            uplink.Recv();
            // std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    });
    thread_recv.detach();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}