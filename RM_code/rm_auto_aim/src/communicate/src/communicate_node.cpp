#include <cstdint>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <unistd.h>

#include "communicate/downlink.hpp"
#include "communicate/uplink.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::string serial_name = "/dev/ttyACM0";  // 串口位置
    //std::string serial_name = "/dev/serial/by-path/pci-0000:00:14.0-usb-0:2:1.0";
    uint32_t baud_rate = 115200;  // 波特率

    drivers::serial_driver::FlowControl fc = drivers::serial_driver::FlowControl ::NONE;
    drivers::serial_driver::Parity pt = drivers::serial_driver::Parity::NONE;
    drivers::serial_driver::StopBits sb = drivers::serial_driver::StopBits::ONE;
    std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config =
        std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);

    Uplink uplink(serial_name, *device_config);
    //communicate.init(serial_name, *device_config);
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