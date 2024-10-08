#include "rune_detector/detector_node.hpp"

using std::placeholders::_1;
#define PNP_ITERATION false
namespace rune {
RuneDetectorNode::RuneDetectorNode(const rclcpp::NodeOptions& options):
    rclcpp::Node("rune_detector", options) {
    RCLCPP_INFO(this->get_logger(), "Starting DetectorNode!");
    confidence_threshold_ = this->declare_parameter("confidence_threshold", 0.9); // 置信度阈值
    model_path = this->declare_parameter("model_path", "/model/yolox_fp16.onnx"); // 模型路径
    detector_ = InitDetector();                                                   // 初始化神符识别器
    pnp_solver_ = nullptr;                                                        // 初始化 pnp 求解器
    debug_ = this->declare_parameter("debug", false);                             // debug 模式

    show_pic = this->declare_parameter("show_pic", false);

    // 创建神符信息发布者
    runes_pub_ = this->create_publisher<auto_aim_interfaces::msg::Rune>("/detector/runes", rclcpp::SensorDataQoS());

    auto&& camera_matrix = declare_parameter("rune_camera_matrix", std::vector<double> {});
    auto&& distortion_coefficients = declare_parameter("distortion_coefficients", std::vector<double> {});
    pnp_solver_ = std::make_unique<PnPSolver>(camera_matrix, distortion_coefficients);

    if (debug_) {
        // Visualization Marker Publisher
        // See http://wiki.ros.org/rviz/DisplayTypes/Marker
        rune_marker_.ns = "armors";
        rune_marker_.action = visualization_msgs::msg::Marker::ADD;
        rune_marker_.type = visualization_msgs::msg::Marker::CUBE;
        rune_marker_.scale.x = 0.05;
        rune_marker_.scale.y = 0.23; //设置默认的 x,y,z
        rune_marker_.scale.z = 0.125;
        rune_marker_.color.a = 1.0;
        rune_marker_.color.g = 0.5;
        rune_marker_.color.b = 1.0;
        rune_marker_.id = 0;
        // rune_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/rune_detector/marker", 10);
        debug_img_pub_ = image_transport::create_publisher(this, "/rune_detector/debug_img");
    }

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_for_rune", rclcpp::SensorDataQoS().keep_last(2), std::bind(&RuneDetectorNode::ImageCallback, this, std::placeholders::_1));
}

bool RuneDetectorNode::DetectRunes(const sensor_msgs::msg::Image::SharedPtr& img_msg) {
    auto&& img = cv::Mat(img_msg->height, img_msg->width, CV_8UC3,
                         img_msg->data.data()); // 把图像信息转换为 cv::Mat 格式
    std::vector<rune::NeuralNetwork::RuneObject> objects;
    detector_->Detect(img, objects); // 把神符识别结果放到 objects里面
    runes_msg_.header = rune_marker_.header = img_msg->header;

    bool flag1 = false, flag2 = false;    // bool flag3 = false;
    float max_prob = 0;                   // 未激活符叶最大置信度,防止识别到多个未激活符叶,取prob最大的
    cv::Point2f symbol;                   // 符叶 R 标的位置
    cv::Point2f rune_armor;               // 符叶未激活装甲板中心
    std::vector<cv::Point2d> rune_points; // 未激活符叶的五个点
    for (const auto& object: objects) {
        // 遍历所有的神符识别结果，把 R 标和未激活的符叶的信息画出来
        auto prob = object.prob;
        if (prob < confidence_threshold_) {
            RCLCPP_WARN(this->get_logger(),
                        "prob is too low"); // 如果置信度小于阈值，则不进行处理
            continue;
        }

        static auto&& get_symbol = [](const cv::Point2f& lightbar_mid_point, const cv::Point2f& armor_center, const double& center_lightbar_ratio, const bool& flag) {
            // get_symbol 是通过符叶的坐标来计算中心 R 标的位置
            if (!flag) {
                // flag = 0 使用装甲板中心和内灯条算出标识符位置
                return ((lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center);

            } else {
                // flag = 1 使用装甲板中心和外灯条算出标识符位置
                return (-(lightbar_mid_point - armor_center) * center_lightbar_ratio + armor_center);
            }
        };

        if (object.cls == 0) {
            // R标 RuneClass::Blue or Red
            auto&& detect_center = (object.vertices[0] + object.vertices[1] + object.vertices[2] + object.vertices[3] + object.vertices[4]) / 5; // 用于计算 R 标位置
            flag1 = true;
            symbol = detect_center;

        } else if (object.cls == 1) {
            // 未激活扇叶 RuneClass::BlueUnActivated or RedUnActivated;
            max_prob = std::max(max_prob, prob);
            if (max_prob != prob) {
                continue;
            }
            rune_points.clear();

            rune_points.emplace_back(object.vertices[1]);
            rune_points.emplace_back(object.vertices[2]);
            rune_points.emplace_back(object.vertices[4]);
            rune_points.emplace_back(object.vertices[0]);
            auto&& tmp1 = (object.vertices[0] + object.vertices[1]) / 2;
            auto&& tmp2 = (object.vertices[2] + object.vertices[4]) / 2;
            auto&& armor = (tmp1 + tmp2) / 2; // 装甲板中心
            rune_armor = armor;
            if (!flag1) {
                // 如果 yolo 没有检测到 R 标
                // symbol = (get_symbol(tmp1, armor, 5.295454, 1) + get_symbol(tmp2,
                // armor, 3.5542, 0)) / 2;
                symbol = (get_symbol(tmp1, armor, 5.295454, true) + get_symbol(tmp2, armor, 4.0542, false)) / 2;
            }
            flag1 = flag2 = true;
        } else if (object.cls == 2) {
            // 已激活的符叶，可以用来扩展一张图中的得到的信息数量
            // RuneClass::BlueActivated or RedActivated;
            // flag3 = true;
            continue;
        }

        if (show_pic) {
            for (auto vertice: object.vertices) {
                // 画出五个关键点
                cv::circle(img, vertice, 4, Colors::Green, -1);
            }
        }
    }

    if (debug_ && show_pic) {
        cv::circle(img, rune_armor, 4, Colors::Green, -1);                            //画出装甲板中心
        cv::circle(img, symbol, 4, Colors::Green, -1);                                //画出 R 标中心
        cv::circle(img, cv::Point2f(img.cols / 2, img.rows / 2), 2, Colors::Blue, 3); // 图像中心点
        PublishImg(img, img_msg);                                                     // 发布图片
    }
    // cv::imshow("img", img);
    // cv::waitKey(1);

    if (flag1 && flag2) {
        // 有 R 标数据和符叶数据，则认为识别完成
        cv::Mat rvec, tvec;
        bool success = pnp_solver_->SolvePnP(rune_points, rvec, tvec, PNP_ITERATION); // 输出旋转向量和平移向量
        if (success) {
            runes_msg_.pose_c.position.x = tvec.at<double>(0);
            runes_msg_.pose_c.position.y = tvec.at<double>(1);
            runes_msg_.pose_c.position.z = tvec.at<double>(2); // 未激活符叶 相机坐标系下的位置
            runes_msg_.leaf_dir.x = (rune_armor - symbol).x;
            runes_msg_.leaf_dir.y = (rune_armor - symbol).y; // 符叶向量

            for (int i = 0; i < 4; i++) {
                runes_msg_.rune_points[i].x = rune_points[i].x;
                runes_msg_.rune_points[i].y = rune_points[i].y;
            }
            runes_msg_.symbol.x = symbol.x;        // R 标位置 图像左上角为原点
            runes_msg_.symbol.y = symbol.y;        // R 标位置 图像左上角为原点
            runes_msg_.header.frame_id = "camera"; // 设置坐标系

            // Fill the markers
            rune_marker_.header.frame_id = "camera";
            rune_marker_.pose = runes_msg_.pose_c;
            return true;
        }
        RCLCPP_WARN(this->get_logger(), "PnP failed!");
    }
    return false;
}

void RuneDetectorNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
    if (pnp_solver_ == nullptr) {
        RCLCPP_WARN(this->get_logger(), "pnp_solver_ is nullptr");
    } else {
        // 检测图片 如果检测到了符叶则发布符叶信息
        // 符模式 0：不可激活 1：小符 2:大符
        runes_msg_.motion = img_msg->header.frame_id == "1" ? 1 : 2;
        img_msg->header.frame_id = "camera";
        runes_msg_.is_find = DetectRunes(img_msg);
        if (debug_) {
            PublishMarkers(); // 发布标记
        }
        runes_pub_->publish(runes_msg_);
    }
}

void RuneDetectorNode::PublishMarkers() {
    using Marker = visualization_msgs::msg::Marker;
    rune_marker_.action = Marker::ADD;
    marker_pub_->publish(rune_marker_);
}

void RuneDetectorNode::PublishImg(cv::Mat& img, const sensor_msgs::msg::Image::SharedPtr& img_msg) {
    debug_img_pub_.publish(cv_bridge::CvImage(img_msg->header, "bgr8", img).toImageMsg());
}

std::shared_ptr<NeuralNetwork> RuneDetectorNode::InitDetector() {
    auto&& detector = std::make_shared<NeuralNetwork>();
    auto pkg_path = ament_index_cpp::get_package_share_directory("rune_detector");
    auto model_load_path = pkg_path + model_path;
    detector->Init(model_load_path);
    return detector;
}

} // namespace rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rune::RuneDetectorNode)