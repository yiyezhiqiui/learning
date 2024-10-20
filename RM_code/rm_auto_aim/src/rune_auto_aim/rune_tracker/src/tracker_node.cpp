#include "rune_tracker/tracker_node.hpp"
#define PNP_ITERATION false
namespace rune {
// RuneTrackerNode类的构造函数
RuneTrackerNode::RuneTrackerNode(const rclcpp::NodeOptions& option):
    Node("rune_tracker_node", option) {
    // 打印信息，表示节点已启动
    RCLCPP_INFO(this->get_logger(), "Starting RuneTrackerNode!");
    InitParams();
    tracker_ = std::make_unique<Tracker>(this, std_a_, std_yawdd_, filter_astring_threshold);
    debug_ = this->declare_parameter("debug", false);

    if (debug_) {
        CreateDebugPublisher(); //创建Debug发布器
        // 可视化标记发布器
        // 参见 http://wiki.ros.org/rviz/DisplayTypes/Marker
        armor_marker_.ns = "armors";
        armor_marker_.type = visualization_msgs::msg::Marker::CUBE;
        armor_marker_.scale.x = 0.03; //TODO:x,y,z值还需修改
        armor_marker_.scale.y = 0.23;
        armor_marker_.scale.z = 0.125;
        armor_marker_.color.a = 1.0;
        armor_marker_.color.r = 1.0;
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/rune_tracker/marker", 10);
    }
    auto&& camera_matrix = declare_parameter("rune_camera_matrix", std::vector<double> {});
    auto&& distortion_coefficients = declare_parameter("distortion_coefficients", std::vector<double> {});
    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, distortion_coefficients);

    // tf2 buffer & listener 相关
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(),
        this->get_node_timers_interface()
    );
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // 订阅器和过滤器
    runes_sub_.subscribe(this, "/detector/runes", rmw_qos_profile_sensor_data);
    target_frame_ = this->declare_parameter("target_frame", "odom");
    tf2_filter_ = std::make_shared<tf2_filter>(
        runes_sub_,                         // message_filters subscriber
        *tf2_buffer_,                       // tf2 buffer
        target_frame_,                      // frame this filter should attempt to transform to
        100,                                // size of the tf2 cache
        this->get_node_logging_interface(), // node logging interface
        this->get_node_clock_interface(),   // node clock interface
        std::chrono::duration<int>(1)       // timeout
    );
    // 注册回调函数
    tf2_filter_->registerCallback(&RuneTrackerNode::RunesCallback, this);

    target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
        "/tracker/target",
        rclcpp::SensorDataQoS()
    );
}

// runeCallback函数实现 接收rune_detector发布的rune消息
void RuneTrackerNode::RunesCallback(const auto_aim_interfaces::msg::Rune::SharedPtr rune_ptr) {
    if (rune_ptr->is_find == false) {
        RCLCPP_INFO(this->get_logger(), "No target found");
        runes_msg_.is_find = false;
        target_pub_->publish(runes_msg_);
        return;
    }
    runes_msg_.is_find = true;
    // 可以动态调参
    rune_ptr->speed = this->get_parameter("bullet_speed").as_double();
    rune_ptr->chasedelay = this->get_parameter("chasedelay").as_double();
    rune_ptr->phase_offset = this->get_parameter("phase_offset").as_double();
    tracker_->Predict(rune_ptr, runes_msg_, debug_msg_);

    cv::Mat rvec, tvec; //tvec为旋转后的相机坐标系下的坐标
    pnp_solver_->SolvePnP(tracker_->GetRotatedRune(), rvec, tvec, PNP_ITERATION);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = rune_ptr->header;
    ps.pose.position.x = tvec.at<double>(0);
    ps.pose.position.y = tvec.at<double>(1);
    ps.pose.position.z = tvec.at<double>(2);
    // 将符叶位置从 相机坐标系 转换到 odom（目标坐标系）
    runes_msg_.predict_target.position = tf2_buffer_->transform(ps, target_frame_).pose.position;
    ps.pose.position = rune_ptr->pose_c.position;
    runes_msg_.detect_target.position = tf2_buffer_->transform(ps, target_frame_).pose.position;

    if (tracker_->GetRuneState() == 2 && tracker_->IsCeresFull()) {
        // 如果符叶运动状态为大符，且cereslist数据量够了，则发布消息
        target_pub_->publish(runes_msg_);
    } else if (tracker_->GetRuneState() == 1) {
        // 如果符叶运动状态为小符，则发布消息
        target_pub_->publish(runes_msg_);
    }

    if (debug_) {
        PublishMarkers(runes_msg_);
        PublishDebugInfo();
    }
}

void RuneTrackerNode::PublishMarkers(const auto_aim_interfaces::msg::Target& target_msg) {
    visualization_msgs::msg::MarkerArray marker_array;
    armor_marker_.header = target_msg.header;
    armor_marker_.header.frame_id = target_frame_;
    armor_marker_.action = visualization_msgs::msg::Marker::ADD;
    armor_marker_.id = 0;
    armor_marker_.pose.position = target_msg.detect_target.position;
    marker_array.markers.emplace_back(armor_marker_);
    armor_marker_.id = 1;
    armor_marker_.pose.position = target_msg.predict_target.position;
    marker_array.markers.emplace_back(armor_marker_);
    marker_pub_->publish(marker_array);
}

void RuneTrackerNode::CreateDebugPublisher() {
    debug_pub_ = this->create_publisher<auto_aim_interfaces::msg::DebugRune>(
        "/rune_tracker/debug",
        rclcpp::SensorDataQoS()
    );
}
void RuneTrackerNode::PublishDebugInfo() {
    debug_msg_.header.stamp = this->now();
    debug_msg_.rotate_angle = tracker_->GetRotateAngle();
    debug_msg_.phase_offset = phase_offset;
    double* tmp = tracker_->GetFittingPara();
    debug_msg_.a_omega_phi_b[0] = tmp[0];
    debug_msg_.a_omega_phi_b[1] = tmp[1];
    debug_msg_.a_omega_phi_b[2] = tmp[2];
    debug_msg_.a_omega_phi_b[3] = tmp[3];
    debug_pub_->publish(debug_msg_);
}

void RuneTrackerNode::InitParams() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.1;
    param_desc.floating_point_range[0].from_value = 20;
    param_desc.floating_point_range[0].to_value = 30;
    bullet_speed = this->declare_parameter("bullet_speed", 25.0, param_desc);
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.01;
    param_desc.floating_point_range[0].from_value = 0;
    param_desc.floating_point_range[0].to_value = 1;
    chasedelay = this->declare_parameter("chasedelay", 0.0, param_desc); //设置chasedelay的默认值0.0
    param_desc.floating_point_range.resize(1);
    param_desc.floating_point_range[0].step = 0.001;
    param_desc.floating_point_range[0].from_value = -3.0;
    param_desc.floating_point_range[0].to_value = 3.0;
    phase_offset = this->declare_parameter("phase_offset", 0.0, param_desc); //设置phaseoffset的默认值0.0
    param_desc.floating_point_range.clear();
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;
    param_desc.integer_range[0].from_value = 0;
    param_desc.integer_range[0].to_value = 30;
    filter_astring_threshold = this->declare_parameter("filter_astring_threshold", 0, param_desc);
    std_a_ = this->declare_parameter("std_a", 5.0);
    std_yawdd_ = this->declare_parameter("std_yawdd", 5.0);
}

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// 用class_loader注册组件。
// 这充当一种入口点，允许在将其库加载到运行中的进程时发现组件。
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneTrackerNode)
