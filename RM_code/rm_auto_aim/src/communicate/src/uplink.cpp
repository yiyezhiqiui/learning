#include "communicate/uplink.hpp"

#include <cstdint>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/joint_state__struct.hpp>

#include "communicate/protocol.hpp"

Uplink::Uplink(
    std::string serial_name,
    drivers::serial_driver::SerialPortConfig device_config
):
    Node("communicate_publisher"),
    owned_ctx_ { new IoContext(2) },
    stm32_serial { new drivers::serial_driver::SerialDriver(*owned_ctx_) } {
    stm32_serial->init_port(serial_name, device_config);

    this->debug = this->declare_parameter("debug", false);
    if (this->debug) {
        this->enemy_team_color = this->declare_parameter("enemy_team_color", 1);
        this->mode = this->declare_parameter("mode", 1);
        this->rune_flag = this->declare_parameter("rune_flag", 1);
    } else {
        this->enemy_team_color = -1;
        this->mode = -1;
        this->rune_flag = -1;
    }

    RCLCPP_INFO(this->get_logger(), "create client.....");
    this->client = this->create_client<communicate::srv::ModeSwitch>("/communicate/autoaim");
    while (!client->wait_for_service(0.1s)) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    this->InitPublisher();

    try {
        stm32_serial->port()->open();
        if (stm32_serial->port()->is_open()) {
            std::cout << "!!!!!!!!打开串口成功!!!!!!!!" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "!!!!!!!!打开串口失败!!!!!!!!" << std::endl;
        std::cout << e.what() << std::endl;
        if (!this->debug) {
            if (ReopenPort() == false) {
                exit(-1);
                RCLCPP_ERROR(this->get_logger(), "Reopen port failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Reopen port success");
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
        RCLCPP_INFO(
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
    if (this->debug) {
        this->PublishGyroDefault(pub_[LEFTGYRO]);
        this->PublishGyroDefault(pub_[RIGHTGYRO]);
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
            this->PublishAutoaim(this->datarecv.buffer, this->pub_[this->datarecv.type], this->pub_[LEFTGYRO], this->pub_[RIGHTGYRO], this->pub_[GYRO]);
        } else {
            this->PublishFunc[this->datarecv.type](this->datarecv.buffer, this->pub_[this->datarecv.type]);
        }
    }
}

void Uplink::Init(
    std::string serial_name,
    drivers::serial_driver::SerialPortConfig device_config
) {
    stm32_serial->init_port(serial_name, device_config);

    this->debug = this->declare_parameter("debug", true);
    if (this->debug) {
        this->enemy_team_color = this->declare_parameter("enemy_team_color", 1);
        this->mode = this->declare_parameter("mode", 1);
        this->rune_flag = this->declare_parameter("rune_flag", 1);
    } else {
        this->enemy_team_color = -1;
        this->mode = -1;
        this->rune_flag = -1;
    }

    this->InitPublisher();

    try {
        stm32_serial->port()->open();
        if (stm32_serial->port()->is_open()) {
            std::cout << "!!!!!!!!打开串口成功!!!!!!!!" << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "!!!!!!!!打开串口失败!!!!!!!!" << std::endl;
        std::cout << e.what() << std::endl;
        if (!this->debug) {
            if (ReopenPort() == false) {
                exit(-1);
                RCLCPP_ERROR(this->get_logger(), "Reopen port failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "Reopen port success");
            }
        }
    }
}

void Uplink::InitPublisher() {
    std::string topic[12];

    topic[AUTOAIM] = "/communicate/autoaim";
    topic[ROBOTPOSITION] = "/communicate/position/robot";
    topic[EXROBOTPOSITION] = "/communicate/position/exrobot";
    topic[GAME] = "/communicate/game";
    topic[REDROBITHP] = "/communicate/hp/redrobot";
    topic[BLUEROBITHP] = "/communicate/hp/bluerobot";
    topic[BUILDINGHP] = "/communicate/hp/building";
    topic[LEFTGYRO] = "/communicate/gyro/left";
    topic[RIGHTGYRO] = "/communicate/gyro/right";
    topic[GYRO] = "joint_states";

    this->pub_[AUTOAIM] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[AUTOAIM], rclcpp::SensorDataQoS());
    this->pub_[ROBOTPOSITION] = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic[ROBOTPOSITION], rclcpp::SensorDataQoS());
    this->pub_[EXROBOTPOSITION] = this->create_publisher<std_msgs::msg::Float32MultiArray>(topic[EXROBOTPOSITION], rclcpp::SensorDataQoS());
    this->pub_[GAME] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[GAME], rclcpp::SensorDataQoS());
    this->pub_[REDROBITHP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[REDROBITHP], rclcpp::SensorDataQoS());
    this->pub_[BLUEROBITHP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[BLUEROBITHP], rclcpp::SensorDataQoS());
    this->pub_[BUILDINGHP] = this->create_publisher<std_msgs::msg::Int32MultiArray>(topic[BUILDINGHP], rclcpp::SensorDataQoS());
    this->pub_[LEFTGYRO] = this->create_publisher<sensor_msgs::msg::JointState>(topic[LEFTGYRO], rclcpp::SensorDataQoS());
    this->pub_[RIGHTGYRO] = this->create_publisher<sensor_msgs::msg::JointState>(topic[RIGHTGYRO], rclcpp::SensorDataQoS());
    this->pub_[GYRO] = this->create_publisher<sensor_msgs::msg::JointState>(topic[GYRO], rclcpp::SensorDataQoS());

    this->PublishFunc[ROBOTPOSITION] = &PublishRobotPosition;
    this->PublishFunc[EXROBOTPOSITION] = &PublishExRobotPosition;
    this->PublishFunc[GAME] = &PublishGameInfo;
    this->PublishFunc[REDROBITHP] = &PublishRedRobotHP;
    this->PublishFunc[BLUEROBITHP] = &PublishBlueRobotHP;
    this->PublishFunc[BUILDINGHP] = &PublishBuildingHP;

    if (this->debug) {
        this->PublishAutoaimDefault(pub_[AUTOAIM]);
    }
}
/**
 * @brief 发布自瞄假信息信息话题
 * @param pub 发布者
 */
void Uplink::PublishAutoaimDefault(
    rclcpp::PublisherBase::SharedPtr pub
) {
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(this->enemy_team_color);
    msg.data.push_back(this->mode);
    msg.data.push_back(this->rune_flag);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
    auto request = std::make_shared<communicate::srv::ModeSwitch::Request>();
    request->set__enemy_color(this->enemy_team_color);
    request->set__mode(this->mode);
    request->set__rune_state(this->rune_flag);
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call ModeSwitch service");
    }
    RCLCPP_INFO(this->get_logger(), "success %d", result.get()->success);
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
    msg.header.frame_id = this->enemy_team_color == 0 ? 'r' : 'b';
    msg.position.push_back(0);
    msg.position.push_back(0);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub);
    pub_completed->publish(msg);
}

/**
 * @brief 发布自瞄信息信息话题
 * @param pub 发布者
 * @param buffer 发送数据
 */
void Uplink::PublishAutoaim(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub1,
    rclcpp::PublisherBase::SharedPtr pub2,
    rclcpp::PublisherBase::SharedPtr pub3,
    rclcpp::PublisherBase::SharedPtr pub4
) {
    AutoaimFeedbackBuffer* data = (AutoaimFeedbackBuffer*)(buffer);
    sensor_msgs::msg::JointState msg_l;
    sensor_msgs::msg::JointState msg_r;
    sensor_msgs::msg::JointState msg_gyro;
    msg_r.header.stamp = msg_l.header.stamp = msg_gyro.header.stamp = this->now();
    msg_r.header.frame_id = msg_l.header.frame_id = data->enemy_team_color;
    msg_l.position.push_back(data->left_yaw);
    msg_l.position.push_back(data->left_pitch);
    msg_r.position.push_back(data->right_yaw);
    msg_r.position.push_back(data->right_pitch);
    msg_gyro.name.push_back("base_joint");
    msg_gyro.position.push_back(data->yaw);

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub1_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub1);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub2_completed = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub2);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub3_completed = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub3);
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub4_completed = std::dynamic_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub4);

    pub2_completed->publish(msg_l);
    pub3_completed->publish(msg_r);
    pub4_completed->publish(msg_gyro);

    int color = (data->enemy_team_color == 'r') ? 0 : 1;
    int mode = (data->mode == 'a') ? 0 : 1;
    int rune_flag = data->rune_flag - '0';

    //哨兵使用时请注释掉这段代码
    if (color == this->enemy_team_color && mode == this->mode && rune_flag == this->rune_flag) {
        return;
    }
    if (mode == 0 && this->mode == 0 && rune_flag != this->rune_flag) {
        return;
    }
    this->enemy_team_color = color;
    this->mode = mode;
    this->rune_flag = rune_flag;
    //

    auto request = std::make_shared<communicate::srv::ModeSwitch::Request>();
    request->set__enemy_color(this->enemy_team_color);
    request->set__mode(this->mode);
    request->set__rune_state(this->rune_flag);
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "success %d", result.get()->success);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call ModeSwitch service");
    }
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
void PublishGameInfo(
    uint8_t* buffer,
    rclcpp::PublisherBase::SharedPtr pub
) {
    GameFeedbackBuffer* data = (GameFeedbackBuffer*)buffer;
    std_msgs::msg::Int32MultiArray msg;
    msg.data.push_back(data->enemy_team_color);
    msg.data.push_back(data->game_time);
    msg.data.push_back(data->game_economy);
    msg.data.push_back(data->allowance_capacity);
    msg.data.push_back(data->left_purchase);
    msg.data.push_back(data->right_purchase);
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pub_completed = std::dynamic_pointer_cast<rclcpp::Publisher<std_msgs::msg::Int32MultiArray>>(pub);
    pub_completed->publish(msg);
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
