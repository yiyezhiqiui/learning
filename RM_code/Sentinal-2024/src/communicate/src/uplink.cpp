#include "communicate/uplink.hpp"

#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>
#include <std_msgs/msg/detail/int32_multi_array__struct.hpp>

#include "communicate/protocol.hpp"

Uplink::Uplink(
    drivers::serial_driver::SerialPortConfig device_config
):
    Node("communicate_publisher"),
    owned_ctx_ { new IoContext(2) },
    stm32_serial { new drivers::serial_driver::SerialDriver(*owned_ctx_) } {
    this->debug_ = this->declare_parameter("debug", false);
    this->serial_name_ = this->declare_parameter("serial_name", "/dev/ttyUSB0");
    stm32_serial->init_port(serial_name_, device_config);

    this->InitPublisher();

    try {
        stm32_serial->port()->open();
        if (stm32_serial->port()->is_open()) {
            std::cout << "!!!!!!!!打开串口成功!!!!!!!!" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "!!!!!!!!打开串口失败!!!!!!!!" << std::endl;
        std::cout << e.what() << std::endl;
        if (!this->debug_) {
            if (ReopenPort() == false) {
                exit(-1);
                RCLCPP_ERROR(this->get_logger(), "Reopen port failed");
            } else {
                RCLCPP_(this->get_logger(), "Reopen port success");
            }
        }
    }
}

bool Uplink::ReopenPort() {
    if (reopen_count++ > 5) {
        return false;
    }

    RCLCPP_WARN(
        this->get_logger(),
        "Attempting to reopen port"
    );

    try {
        if (stm32_serial->port()->is_open()) {
            stm32_serial->port()->close();
        }
        stm32_serial->port()->open();
        RCLCPP_(
            this->get_logger(),
            "Successfully reopened port"
        );
    } catch (const std::exception& ex) {
        RCLCPP_WARN(
            this->get_logger(),
            "Error while reopening port: %s",
            ex.what()
        );
        if (rclcpp::ok()) {
            rclcpp::sleep_for(std::chrono::seconds(1));
            return ReopenPort();
        }
    }
    return true;
}

void Uplink::Recv() {
    if (this->debug_) {
        PublishGyroDefault(pub_[GYRO]);
        return;
    }
    try {
        std::vector<uint8_t> data(sizeof(Message));
        stm32_serial->port()->receive(data);
        this->datarecv = fromVector(data);
    } catch (const std::exception& e) {
        std::cout << "!!!!!!!!消息接收失败!!!!!!!!" << std::endl;
        std::cout << e.what() << std::endl;
        exit(-3);
    }

    if (this->datarecv.start != 's' || this->datarecv.end != 'e' || this->datarecv.type < 0xB0 || this->datarecv.type > 0xB6) {
        // for (int i = 0; i < 32; i++)
        //     std::cout << (int)(((uint8_t*)&this->datarecv)[i]) << " ";
        // std::cout << std::endl;
        std::cout << "!!!!!!!!数据包校验失败!!!!!!!!" << std::endl;
        // for (int i = 0; i < 32; i++) {
        //     std::cout << (int)(((uint8_t*)&this->datarecv)[i]) << " ";
        // }
        // std::cout << std::endl;
        // std::cout << "start:" << (char)(this->datarecv.start) << std::endl
        //           << "end:" << (char)(this->datarecv.end) << std::endl
        //           << "type:" << std::hex << (int)this->datarecv.type << std::endl;
        exit(-1);
    } else {
        // std::cout << "yes" << std::endl;
        this->datarecv.type -= 0xB0;
        if (this->datarecv.type == 0) {
            this->PublishGyro(this->datarecv.buffer, this->pub_[this->datarecv.type]);
        } else if (this->datarecv.type == 3) {
            PublishGame(this->datarecv.buffer, this->pub_[this->datarecv.type], this->pub_[AUTOAIM]);
        } else {
            this->PublishFunc[this->datarecv.type](this->datarecv.buffer, this->pub_[this->datarecv.type]);
        }
    }
}

void Uplink::Init(
    drivers::serial_driver::SerialPortConfig device_config
) {
    this->debug_ = this->declare_parameter("debug", false);
    this->serial_name_ = this->declare_parameter("serial_name", "/dev/ttyUSB0");
    stm32_serial->init_port(serial_name_, device_config);

    this->InitPublisher();

    try {
        stm32_serial->port()->open();
        if (stm32_serial->port()->is_open()) {
            std::cout << "!!!!!!!!打开串口成功!!!!!!!!" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "!!!!!!!!打开串口失败!!!!!!!!" << std::endl;
        std::cout << e.what() << std::endl;
        if (!this->debug_) {
            if (ReopenPort() == false) {
                exit(-1);
                RCLCPP_ERROR(this->get_logger(), "Reopen port failed");
            } else {
                RCLCPP_(this->get_logger(), "Reopen port success");
            }
        }
    }
}

void Uplink::InitPublisher() {
    std::string topic[12];

    topic[GYRO] = "/communicate/gyro";
    topic[ROBOTPOSITION] = "/communicate/position/robot";
    topic[EXROBOTPOSITION] = "/communicate/position/exrobot";
    topic[GAME] = "/communicate/game";
    topic[REDROBITHP] = "/communicate/hp/redrobot";
    topic[BLUEROBITHP] = "/communicate/hp/bluerobot";
    topic[BUILDINGHP] = "/communicate/hp/building";
    topic[SHOOTSTAUTS] = "/communicate/shootstauts";
    topic[ENEMYROBOTPOSITION] = "/communicate/position/enemyrobot";
    topic[ENEMYEXROBOTPOSITION] = "/communicate/position/enemyexrobot";
    topic[AUTOAIM] = "/";

    this->pub_[GYRO] = this->create_publisher<sensor_msgs::msg::JointState>(topic[GYRO], rclcpp::SensorDataQoS());
    this->pub_[ROBOTPOSITION] = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic[ROBOTPOSITION], rclcpp::SensorDataQoS());
    this->pub_[EXROBOTPOSITION] = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic[EXROBOTPOSITION], rclcpp::SensorDataQoS());
    this->pub_[GAME] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[GAME], rclcpp::SensorDataQoS());
    this->pub_[REDROBITHP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[REDROBITHP], rclcpp::SensorDataQoS());
    this->pub_[BLUEROBITHP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[BLUEROBITHP], rclcpp::SensorDataQoS());
    this->pub_[BUILDINGHP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[BUILDINGHP], rclcpp::SensorDataQoS());
    this->pub_[SHOOTSTAUTS] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[SHOOTSTAUTS], rclcpp::SensorDataQoS());
    this->pub_[ENEMYROBOTPOSITION] = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic[ENEMYROBOTPOSITION], rclcpp::SensorDataQoS());
    this->pub_[ENEMYEXROBOTPOSITION] = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic[ENEMYEXROBOTPOSITION], rclcpp::SensorDataQoS());
    this->pub_[AUTOAIM] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[AUTOAIM], rclcpp::SensorDataQoS());

    this->PublishFunc[ROBOTPOSITION] = &PublishRobotPosition;
    this->PublishFunc[EXROBOTPOSITION] = &PublishExRobotPosition;
    this->PublishFunc[REDROBITHP] = &PublishRedRobotHP;
    this->PublishFunc[BLUEROBITHP] = &PublishBlueRobotHP;
    this->PublishFunc[BUILDINGHP] = &PublishBuildingHP;
    this->PublishFunc[SHOOTSTAUTS] = &PublishShootStauts;
    this->PublishFunc[ENEMYROBOTPOSITION] = &PublishEnemyRobotPosition;
    this->PublishFunc[ENEMYEXROBOTPOSITION] = &PublishEnemyExRobotPosition;
}

/**
 * @brief 发布默认tf树假信息话题
 * @param pub 发布者
 */
void Uplink::PublishGyroDefault(
    rclcpp::PublisherBase::SharedPtr pub
) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.position.push_back(0);
    msg.position.push_back(0);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub);
    pub_completed->publish(msg);
}

/**
 * @brief 发布陀螺仪信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void Uplink::PublishGyro(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    GyroFeedback_s* data = (GyroFeedback_s*)(buffer);
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.position.push_back(data->yaw);
    msg.position.push_back(data->pitch);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub);
    pub_completed->publish(msg);
}

/**
 * @brief 发布我方机器人位置信息话题(1 2 3 号机器人)
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    RobotPositionFeedbackBuffer* data = (RobotPositionFeedbackBuffer*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->standard_1_x);
    msg.data.push_back(data->standard_1_y);
    msg.data.push_back(data->standard_2_x);
    msg.data.push_back(data->standard_2_y);
    msg.data.push_back(data->standard_3_x);
    msg.data.push_back(data->standard_3_y);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}
/**
 * @brief 发布我方机器人位置信息话题(4 5 7 号机器人)
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishExRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    ExRobotPositionFeedbackBuffer* data = (ExRobotPositionFeedbackBuffer*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->standard_4_x);
    msg.data.push_back(data->standard_4_y);
    msg.data.push_back(data->standard_5_x);
    msg.data.push_back(data->standard_5_y);
    msg.data.push_back(data->standard_7_x);
    msg.data.push_back(data->standard_7_y);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}
/**
 * @brief 发布比赛信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishGame(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub1,
    rclcpp::PublisherBase::SharedPtr pub2
) {
    GameFeedbackBuffer* data = (GameFeedbackBuffer*)buffer;
    std_msgs::msg::Int32MultiArray msg_g;
    std_msgs::msg::Int32MultiArray msg_a;
    msg_g.data.push_back(data->enemy_team_color);
    msg_g.data.push_back(data->game_time);
    msg_g.data.push_back(data->game_economy);
    msg_g.data.push_back(data->allowance_capacity);
    msg_g.data.push_back(data->left_purchase);
    msg_g.data.push_back(data->right_purchase);
    msg_a.data.push_back(data->enemy_team_color);
    msg_a.data.push_back(0);
    msg_a.data.push_back(0);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub1_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub1);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub2_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub2);
    pub1_completed->publish(msg_g);
    pub2_completed->publish(msg_a);
}
/**
 * @brief 发布红方机器人血量信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishRedRobotHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    RedRobotHPFeedbackBuffer* data = (RedRobotHPFeedbackBuffer*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->red_1_robot_HP);
    msg.data.push_back(data->red_2_robot_HP);
    msg.data.push_back(data->red_3_robot_HP);
    msg.data.push_back(data->red_4_robot_HP);
    msg.data.push_back(data->red_5_robot_HP);
    msg.data.push_back(data->red_7_robot_HP);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}
/**
 * @brief 发布蓝方机器人血量信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishBlueRobotHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    BlueRobotHPFeedbackBuffer* data = (BlueRobotHPFeedbackBuffer*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->blue_1_robot_HP);
    msg.data.push_back(data->blue_2_robot_HP);
    msg.data.push_back(data->blue_3_robot_HP);
    msg.data.push_back(data->blue_4_robot_HP);
    msg.data.push_back(data->blue_5_robot_HP);
    msg.data.push_back(data->blue_7_robot_HP);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}
/**
 * @brief 发布建筑血量信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishBuildingHP(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    BuildingHPFeedbackBuffer* data = (BuildingHPFeedbackBuffer*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->red_outpost_HP);
    msg.data.push_back(data->red_base_HP);
    msg.data.push_back(data->blue_outpost_HP);
    msg.data.push_back(data->blue_base_HP);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

/**
 * @brief 发布发射状态量信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishShootStauts(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    ShootStautsBuffer* data = (ShootStautsBuffer*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->left_real_heat);
    msg.data.push_back(data->right_real_heat);
    msg.data.push_back(data->left_bullet_speed);
    msg.data.push_back(data->right_bullet_speed);
    msg.data.push_back(data->game_progress);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
}

/**
 * @brief 发布敌方机器人位置信息话题(1 2 3 号机器人)
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishEnemyRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    EnemyRobotPositionFeedbackBuffer* data = (EnemyRobotPositionFeedbackBuffer*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->enemy1_x_position);
    msg.data.push_back(data->enemy1_y_position);
    msg.data.push_back(data->enemy2_x_position);
    msg.data.push_back(data->enemy2_y_position);
    msg.data.push_back(data->enemy3_x_position);
    msg.data.push_back(data->enemy3_y_position);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}

/**
 * @brief 发布敌方机器人位置信息话题(4 5 7 号机器人)
 * @param pub 发布者
 * @param buffer 发送数据
 */
void PublishEnemyExRobotPosition(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    EnemyExRobotPositionFeedbackBuffer* data = (EnemyExRobotPositionFeedbackBuffer*)buffer;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.push_back(data->enemy4_x_position);
    msg.data.push_back(data->enemy4_y_position);
    msg.data.push_back(data->enemy5_x_position);
    msg.data.push_back(data->enemy5_y_position);
    msg.data.push_back(data->enemy7_x_position);
    msg.data.push_back(data->enemy7_y_position);
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float32MultiArray>>(pub);
    pub_completed->publish(msg);
}
