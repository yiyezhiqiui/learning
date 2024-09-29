# 与下位机通讯节点代码
## 文件结构
```
.
├── include/communicate
│   ├── uplink.hpp
│   ├── downlink.hpp
│   └── protocol.hpp
│
├── msg
│   ├── GuardStatus.msg
│   ├── IgnoreClasses.msg
│   └── SerialInfo.msg
│
├── config
│   └── config.yaml
│
├── launch
│   └── launch.py
│
├── shell
│   └── run.bash
│
├── src
│   ├── uplink.cpp
│   ├── downlinnk.cpp
│   └── communicate_node.cpp
│
└── reset.bash
```
### 下位机传给上位机的话题及内容
#### /communicate/autoaim
```
// 话题数据类型：std_msgs::msg::Int32MultiArray
int enemy_team_color;  // 敌方颜色，蓝色为 1
int mode;              // 模式 0：自瞄 1：符
int rune_flag;         // 符模式 0：不可激活 1：小符 2:大符
```
#### /communicate/gyro/left 
```
// 话题数据类型：sensor_msgs::msg::JointState
// 左云台
.header.stamp            // 时间戳 
.position[0];            // 偏航角
.position[1];            // 俯仰角
```
#### /communicate/gyro/right 
```
// 话题数据类型：sensor_msgs::msg::JointState
// 右云台
.header.stamp            // 时间戳 
.position[0];            // 偏航角
.position[1];            // 俯仰角
```
#### /communicate/chassis
```
// 话题数据类型：std_msgs::msg::Float32MultiArray
float vx;              // x 轴方向速度
float vy;              // y 轴方向速度
```
#### /communicate/displacement
```
// 话题数据类型：std_msgs::msg::Float32MultiArray
float yaw;             // 偏航角
float x;               // x 轴方向位移
float y;               // y 轴方向位移
```
#### /communicate/position/robot
```
// 话题数据类型：std_msgs::msg::Float32MultiArray
float standard_1_x;    // 己方 1 号英雄机器人位置 x 轴坐标
float standard_1_y;    // 己方 1 号英雄机器人位置 y 轴坐标
float standard_2_x;    // 己方 2 号工程机器人位置 x 轴坐标
float standard_2_y;    // 己方 2 号工程机器人位置 y 轴坐标
float standard_3_x;    // 己方 3 号步兵机器人位置 x 轴坐标
float standard_3_y;    // 己方 3 号步兵机器人位置 y 轴坐标
```
#### /communicate/position/exrobot
```
// 话题数据类型：std_msgs::msg::Float32MultiArray
float standard_4_x;    // 己方 4 号步兵机器人位置 x 轴坐标
float standard_4_y;    // 己方 4 号步兵机器人位置 y 轴坐标
float standard_5_x;    // 己方 5 号步兵机器人位置 x 轴坐标
float standard_5_y;    // 己方 5 号步兵机器人位置 y 轴坐标
float standard_7_x;    // 己方 7 号哨兵机器人位置 x 轴坐标
float standard_7_y;    // 己方 7 号哨兵机器人位置 y 轴坐标
```
#### /communicate/game
```
// 话题数据类型：std_msgs::msg::Int32MultiArray
int enemy_team_color;  // 敌方颜色，蓝色为 1
int game_time;         // 比赛时间
int game_economy;      // 比赛经济
int allowance_capacity;// 允许发弹量
int left_purchase;     // 左发射机构状态
int right_purchase;    // 右发射机构状态
```
#### /communicate/hp/redrobot
```
// 话题数据类型：std_msgs::msg::Int32MultiArray
int red_1_robot_HP;    // 红 1 英雄机器人血量
int red_2_robot_HP;    // 红 2 工程机器人血量
int red_3_robot_HP;    // 红 3 步兵机器人血量
int red_4_robot_HP;    // 红 4 步兵机器人血量
int red_5_robot_HP;    // 红 5 步兵机器人血量
int red_7_robot_HP;    // 红 7 哨兵机器人血量
```
#### /communicate/hp/bluerobot
```
// 话题数据类型：std_msgs::msg::Int32MultiArray
int blue_1_robot_HP;   // 蓝 1 英雄机器人血量
int blue_2_robot_HP;   // 蓝 2 工程机器人血量
int blue_3_robot_HP;   // 蓝 3 步兵机器人血量
int blue_4_robot_HP;   // 蓝 4 步兵机器人血量
int blue_5_robot_HP;   // 蓝 5 步兵机器人血量
int blue_7_robot_HP;   // 蓝 7 哨兵机器人血量
```
#### /communicate/hp/building
```
// 话题数据类型：std_msgs::msg::Int32MultiArray
int red_outpost_HP;    // 红方前哨站血量
int red_base_HP;       // 红方基地血量
int blue_outpost_HP;   // 蓝方前哨站血量
int blue_base_HP;      // 蓝方基地血量
```
### 上位机传给上位机的话题及内容
#### /shoot_info/left
```
// 话题数据类型：communicate::msg::SerialInfo
// 左云台
float yaw;             // 偏航角
float pitch;           // 俯仰角
char find_bool         // 追踪
char shoot_bool        // 开火
```
#### /shoot_info/right
```
// 话题数据类型：communicate::msg::SerialInfo
// 右云台
float yaw;             // 偏航角
float pitch;           // 俯仰角
char find_bool         // 追踪
char shoot_bool        // 开火
```
#### cmd_vel
```
// 话题数据类型：std_msgs::msg::Float32MultiArray
float vx;              // x 轴方向速度
float vy;              // y 轴方向速度
```
#### /behaviortree/interaction
```
// 话题数据类型：std_msgs::msg::Int32MultiArray
int type;              // 类型  0：无  1：买活  2：买弹丸
int content;           // 具体内容
```
#### /behaviortree/moudle
```
// 话题数据类型：std_msgs::msg::Int32MultiArray
int type;              // 类型  0：无  1：小陀螺  2：左小云台单连发控制  3：右小云台单连发控制
int content;           // 具体内容
```