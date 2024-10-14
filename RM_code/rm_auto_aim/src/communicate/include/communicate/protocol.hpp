#ifndef PROTOCOL_HPP
#define PROTOCOL_HPP

#include <cstdint>
#include <iostream>
#include <vector>

#define BYTE1(x) (((x) >> 0) & 0xFF)   // 第 1 字节
#define BYTE2(x) (((x) >> 8) & 0xFF)   // 第 2 字节
#define BYTE3(x) (((x) >> 16) & 0xFF)  // 第 3 字节
#define BYTE4(x) (((x) >> 24) & 0xFF)  // 第 4 字节

#pragma pack(1)            // 字对齐
typedef struct Protocol_s  // 通信协议
{
    uint8_t start;       // 帧头，取 's’     0 - 1
    uint8_t type;        // 消息类型         1 - 2
                         /* type 取值：
                         *       上位机发下位机
                         *      0xA0     左小云台控制
                         *      0xA1     右小云台控制
                         *      0xA2     大云台控制
                         *      0xA3     底盘控制
                         *      0xA4     比赛交互控制
                         *      0xA5     车体模块控制
                         *       下位机发上位机
                         *      0xB0     自瞄信息
                         *      0xB1     我方机器人位置信息(1 2 3 号机器人)
                         *      0xB2     我方机器人位置信息(4 5 7 号机器人)
                         *      0xB3     比赛信息
                         *      0xB4     红方机器人血量信息
                         *      0xB5     蓝方机器人血量信息
                         *      0xB6     建筑血量信息
                         */
    uint8_t buffer[29];  // 数据            2 - 31
    uint8_t end;         // 帧尾，取 'e'    31 - 32
} Message;
#pragma pack()

inline Message fromVector(const std::vector<uint8_t>& data) {
    Message packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t*>(&packet));
    return packet;
}

inline std::vector<uint8_t> toVector(const Message& data) {
    std::vector<uint8_t> packet(sizeof(Message));
    std::copy(
        reinterpret_cast<const uint8_t*>(&data),
        reinterpret_cast<const uint8_t*>(&data) + sizeof(Message),
        packet.begin());
    return packet;
}

#pragma pack(1)
// 上位机发下位机
typedef struct LeftptzControl_s  // 左小云台控制发送数据
{
    char find_bool;   // 追踪
    float yaw;        // 偏航角
    float pitch;      // 俯仰角
    float distance;    // 距离
} LeftptzControlBuffer;

typedef struct RightptzControl_s  // 右小云台控制发送数据
{
    char find_bool; // 追踪
    float yaw;        // 偏航角
    float pitch;      // 俯仰角
    float distance;    // 距离
} RightptzControlBuffer;

typedef struct MainptzControl_s  // 大云台控制发送数据
{
    float yaw;  // 偏航角
} MainptzControlBuffer;

typedef struct ChassisControl_s  // 底盘控制发送数据
{
    float vx;  // x 轴方向速度
    float vy;  // y 轴方向速度
} ChassisControlBuffer;

typedef struct InteractionControl_s  // 比赛交互控制发送数据
{
    int type;     // 类型  0：无  1：买活  2：买弹丸
    int content;  // 具体内容
} InteractionControlBuffer;

typedef struct MoudleControl_s  // 车体模块控制发送数据
{
    int type;     // 类型  0：无  1：小陀螺  2：左小云台单连发控制  3：右小云台单连发控制
    int content;  // 具体内容
} MoudleControlBuffer;

// 下位机发上位机
typedef struct AutoaimFeedback_s  // 自瞄信息反馈接收数据
{
    char enemy_team_color;  // 敌方颜色 'r':红色 'b':蓝色
    char mode;              // 模式 ‘a’：自瞄 'r'：符
    char rune_flag;         // 符模式 '0'：不可激活 '1'：小符 '2':大符
    float left_yaw;         // 偏航角 左云台
    float left_pitch;       // 俯仰角
    float right_yaw;        // 偏航角 右云台
    float right_pitch;      // 俯仰角
    float yaw;              // 偏航角 大云台
} AutoaimFeedbackBuffer;

typedef struct RobotPositionFeedback_s  // 我方机器人位置反馈接收数据(1 2 3 号机器人)
{
    float standard_1_x;  // 己方 1 号英雄机器人位置 x 轴坐标
    float standard_1_y;  // 己方 1 号英雄机器人位置 y 轴坐标
    float standard_2_x;  // 己方 2 号工程机器人位置 x 轴坐标
    float standard_2_y;  // 己方 2 号工程机器人位置 y 轴坐标
    float standard_3_x;  // 己方 3 号步兵机器人位置 x 轴坐标
    float standard_3_y;  // 己方 3 号步兵机器人位置 y 轴坐标
} RobotPositionFeedbackBuffer;

typedef struct ExRobotPositionFeedback_s  // 我方机器人位置反馈接收数据(4 5 7 号机器人)
{
    float standard_4_x;  // 己方 4 号步兵机器人位置 x 轴坐标
    float standard_4_y;  // 己方 4 号步兵机器人位置 y 轴坐标
    float standard_5_x;  // 己方 5 号步兵机器人位置 x 轴坐标
    float standard_5_y;  // 己方 5 号步兵机器人位置 y 轴坐标
    float standard_7_x;  // 己方 7 号哨兵机器人位置 x 轴坐标
    float standard_7_y;  // 己方 7 号哨兵机器人位置 y 轴坐标
} ExRobotPositionFeedbackBuffer;

typedef struct GameFeedback_s  // 比赛反馈接收数据
{
    int enemy_team_color;    // 敌方颜色 0：红 1：蓝
    int game_progress;       // 比赛阶段
    int game_time;           // 比赛时间
    int game_economy;        // 比赛经济
    int allowance_capacity;  // 允许发弹量
    int left_purchase;       // 左发射机构状态
    int right_purchase;      // 右发射机构状态
} GameFeedbackBuffer;

typedef struct RedRobotHPFeedback_s  //红方机器人血量反馈接收数据
{
    int red_1_robot_HP;  // 红 1 英雄机器人血量
    int red_2_robot_HP;  // 红 2 工程机器人血量
    int red_3_robot_HP;  // 红 3 步兵机器人血量
    int red_4_robot_HP;  // 红 4 步兵机器人血量
    int red_5_robot_HP;  // 红 5 步兵机器人血量
    int red_7_robot_HP;  // 红 7 哨兵机器人血量
} RedRobotHPFeedbackBuffer;

typedef struct BlueRobotHPFeedback_s  //蓝方机器人血量反馈接收数据
{
    int blue_1_robot_HP;  // 蓝 1 英雄机器人血量
    int blue_2_robot_HP;  // 蓝 2 工程机器人血量
    int blue_3_robot_HP;  // 蓝 3 步兵机器人血量
    int blue_4_robot_HP;  // 蓝 4 步兵机器人血量
    int blue_5_robot_HP;  // 蓝 5 步兵机器人血量
    int blue_7_robot_HP;  // 蓝 7 哨兵机器人血量
} BlueRobotHPFeedbackBuffer;

typedef struct BuildingHPFeedback_s  //建筑血量反馈接收数据
{
    int red_outpost_HP;   // 红方前哨站血量
    int red_base_HP;      // 红方基地血量
    int blue_outpost_HP;  // 蓝方前哨站血量
    int blue_base_HP;     // 蓝方基地血量
} BuildingHPFeedbackBuffer;
#pragma pack()
#endif
