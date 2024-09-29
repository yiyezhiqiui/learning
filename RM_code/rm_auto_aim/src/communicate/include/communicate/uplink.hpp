#ifndef _UPLINK_HPP_
#define _UPLINK_HPP_

#include "protocol.hpp"
#include <chrono>
#include <communicate/srv/mode_switch.hpp>
#include <cstdint>
#include <functional>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <string>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

class Uplink: public rclcpp::Node {
private:
    Message datasend; // 发送消息
    Message datarecv; // 接收消息
    Message temp;     // 缓存
    enum { AUTOAIM = 0,
           ROBOTPOSITION,
           EXROBOTPOSITION,
           GAME,
           REDROBITHP,
           BLUEROBITHP,
           BUILDINGHP,
           LEFTGYRO,
           RIGHTGYRO,
           GYRO
    };                                                                   // 功能ID
    void (*PublishFunc[10])(uint8_t*, rclcpp::PublisherBase::SharedPtr); // 发布消息函数指针

    rclcpp::PublisherBase::SharedPtr pub_[12]; // 发布者

    std::unique_ptr<IoContext> owned_ctx_;
    std::unique_ptr<drivers::serial_driver::SerialDriver> stm32_serial; // 串口

    void InitPublisher(); // 初始化发布者
    bool debug;           // 是否开启调试模式
    int enemy_team_color;
    int mode;
    int rune_flag;
    rclcpp::Client<communicate::srv::ModeSwitch>::SharedPtr client;

public:
    /**
     * @brief 默认构造函数
     */
    Uplink();
    /**
     * @brief Uplink 构造函数
     * @param serial_name 串口路径
     * @param device_config 串口配置
     */
    Uplink(std::string serial_name, drivers::serial_driver::SerialPortConfig device_config);
    /**
     * @brief 向下位机发送数据
     * @template T 数据类型
     * @param type 消息类型
     * @param buffer 数据
     */
    template<typename T>
    void Send(uint8_t type, T* buffer) {
        this->datasend.start = 's';
        this->datasend.end = 'e';
        this->datasend.type = type;
        memcpy(this->datasend.buffer, (uint8_t*)buffer, sizeof(T));
        auto data = toVector(this->datasend);
        try {
            stm32_serial->port()->send(data);
        } catch (const std::exception& e) {
            std::cout << "!!!!!!!!消息发送失败!!!!!!!!" << std::endl;
            std::cout << e.what() << std::endl;
            exit(-2);
        }
    }
    /**
     * @brief 从下位机接收消息
     */
    void Recv();
    /**
     * @brief 初始化对象, 设置并打开串口, 消息缓冲区置零
     * @param serial_name 串口路径
     * @param device_config 串口配置
     */
    void Init(std::string serial_name, drivers::serial_driver::SerialPortConfig device_config);

    /**
     * @brief 重开串口
     */
    bool ReopenPort();

    /**
     * @brief 发布自瞄假信息话题
     * @param pub 发布者
     */
    void PublishAutoaimDefault(
        rclcpp::PublisherBase::SharedPtr pub
    );

    /**
     * @brief 发布默认tf树假信息话题
     * @param pub 发布者
     */
    void PublishGyroDefault(
        rclcpp::PublisherBase::SharedPtr pub
    );

    /**
     * @brief 发布自瞄信息信息话题
     * @param pub 发布者
     * @param buffer 发送数据
     */
    void PublishAutoaim(
        uint8_t* buffer,
        rclcpp::PublisherBase::SharedPtr pub1,
        rclcpp::PublisherBase::SharedPtr pub2,
        rclcpp::PublisherBase::SharedPtr pub3,
        rclcpp::PublisherBase::SharedPtr pub4
    );

    int reopen_count = 0;
};

/**
 * @brief 发布我方机器人位置信息话题(1 2 3 号机器人)
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);

/**
 * @brief 发布我方机器人位置信息话题(4 5 7 号机器人)
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishExRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);

/**
 * @brief 发布比赛信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishGameInfo(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);

/**
 * @brief 发布红方机器人血量信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishRedRobotHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);

/**
 * @brief 发布蓝方机器人血量信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishBlueRobotHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);

/**
 * @brief 发布建筑血量信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishBuildingHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
);

#endif