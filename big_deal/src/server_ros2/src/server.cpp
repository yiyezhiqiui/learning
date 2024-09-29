#include "../include/server.hpp"

ServerWay::ServerWay() : Node("server")
{
}

void ServerWay::handle_request(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                               std::shared_ptr<interfaces_ros2::srv::Interface::Response> response)
{
    if (request->request_type == "image")
    {
        send_img(request, response);
    }
    else if (request->request_type == "video")
    {
        send_video(request, response);
    }
    else if (request->request_type == "camera")
    {
        send_camera(request, response);
    }
    else if (request->request_type == "transform")
    {
        send_transform(request, response);
    }
    else
    {
        std::cout << "request wrong" << std::endl;
    }
}

void ServerWay::send_img(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                         std::shared_ptr<interfaces_ros2::srv::Interface::Response> response)
{
    std::string image_path = "./src/server_ros2/data/img/" + std::to_string(request->request_name) + ".jpg";
    cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);

    if (image.empty())
    {
        std::cout << "Failed to load image for request_name: " << request->request_name << std::endl;
        // RCLCPP_INFO(this->get_logger(), "Failed to load image for request_name: %d", request->request_name);
        return;
    }
    else
    {
        std::cout << "Loaded image for request_name: " << request->request_name << " with size:" << image.cols << "x" << image.rows << std::endl;
        // RCLCPP_INFO(this->get_logger(), "Loaded image for request_name: %d with size: %d x %d", request->request_name, image.cols, image.rows);
    }

    cv_bridge::CvImage cv_image;
    cv_image.image = image;
    cv_image.encoding = "bgr8";

    response->image_data = *cv_image.toImageMsg();

    std::cout << "Successfully sent image for request_name std: " << request->request_name << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Successfully sent image for request_name: %d", request->request_name);
}
void ServerWay::send_video(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                           std::shared_ptr<interfaces_ros2::srv::Interface::Response> response)
{
    int video_id = request->request_name;

    // 如果没有打开过该视频，或者视频已经结束，则重新打开视频
    if (video_captures_.find(video_id) == video_captures_.end() || !video_captures_[video_id].isOpened())
    {
        std::string video_path = "./src/server_ros2/data/video/" + std::to_string(request->request_name) + ".mp4";
        video_captures_[video_id] = cv::VideoCapture(video_path);

        if (!video_captures_[video_id].isOpened())
        {
            std::cout << "Failed to open video for request_name: " << request->request_name << std::endl;
            //RCLCPP_INFO(this->get_logger(), "Failed to open video for request_name: %d", request->request_name);
            return;
        }
    }

    // 读取下一帧
    cv::Mat frame;
    video_captures_[video_id] >> frame;

    cnt_++;
    // std::cout<<cnt_<<std::endl;
    if (frame.empty()) // 视频结束
    {
        std::cout << "Finished sending video for request_name: " << request->request_name << std::endl;
        // RCLCPP_INFO(this->get_logger(), "Finished sending video for request_name: %d", request->request_name);

        // 关闭视频并移除捕获器
        video_captures_[video_id].release();
        video_captures_.erase(video_id);
        return;
    }

    cv_bridge::CvImage cv_image;
    cv_image.image = frame;
    cv_image.encoding = "bgr8";

    // 将帧转换为 ROS2 消息并发送
    response->image_data = *cv_image.toImageMsg();
}

void ServerWay::send_camera(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                            std::shared_ptr<interfaces_ros2::srv::Interface::Response> response)
{
    std::string matrix_path = "./src/server_ros2/data/camera/matrix.txt";
    std::string dist_path = "./src/server_ros2/data/camera/distcoeffs.txt";

    // 读取相机矩阵文件
    std::ifstream matrix_file(matrix_path);
    if (matrix_file.is_open())
    {
        std::vector<double> camera_matrix;
        std::string line;
        while (std::getline(matrix_file, line))
        {
            std::stringstream ss(line);
            double value;
            while (ss >> value)
            {
                camera_matrix.push_back(value);
            }
        }
        matrix_file.close();

        // 将相机矩阵写入到 response 中
        response->camerafile = camera_matrix;
    }
    else
    {
        std::cerr << "Failed to open camera matrix file." << std::endl;
    }

    // 读取畸变系数文件
    std::ifstream dist_file(dist_path);
    if (dist_file.is_open())
    {
        std::vector<double> dist_coeffs;
        std::string line;
        while (std::getline(dist_file, line))
        {
            std::stringstream ss(line);
            double value;
            while (ss >> value)
            {
                dist_coeffs.push_back(value);
            }
        }
        dist_file.close();

        // 将畸变系数写入到 response 中
        response->distcoeffs = dist_coeffs;
    }
    else
    {
        std::cerr << "Failed to open distortion coefficients file." << std::endl;
    }

    std::cout << "Successfully sent camera for request " << request->request_name << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Successfully sent camera for request_name: %d", request->request_name);
}

void ServerWay::send_transform(const std::shared_ptr<interfaces_ros2::srv::Interface::Request> request,
                               std::shared_ptr<interfaces_ros2::srv::Interface::Response> response)
{
    std::string odom_gimbal_path = "./src/server_ros2/data/transform/odom_gim.txt";
    std::string gimbal_camera_path = "./src/server_ros2/data/transform/gim_cam";
    std::string odom_camera_path = "./src/server_ros2/data/transform/odom_cam.txt";

    std::vector<double> trans;
    if (request->transform_request[0] == "Odom" && request->transform_request[1] == "Gimbal")
    {
        response->transform_reply[0] = "Odom";
        response->transform_reply[1] = "Gimbal";
        std::ifstream trans_file(odom_gimbal_path);
        if (trans_file.is_open())
        {
            std::vector<double> trans;
            std::string line;
            while (std::getline(trans_file, line))
            {
                std::stringstream ss(line);
                double value;
                while (ss >> value)
                {
                    trans.push_back(value);
                }
            }
            trans_file.close();

            // 将畸变系数写入到 response 中
            response->transform = trans;
        }
        else
        {
            std::cerr << "Failed to open distortion coefficients file." << std::endl;
        }
    }
    else if (request->transform_request[0] == "Odom" && request->transform_request[1] == "Camera")
    {
        response->transform_reply[0] = "Odom";
        response->transform_reply[1] = "Camera";
        std::ifstream trans_file(odom_camera_path);
        if (trans_file.is_open())
        {
            std::vector<double> trans;
            std::string line;
            while (std::getline(trans_file, line))
            {
                std::stringstream ss(line);
                double value;
                while (ss >> value)
                {
                    trans.push_back(value);
                }
            }
            trans_file.close();

            // 将畸变系数写入到 response 中
            response->transform = trans;
        }
        else
        {
            std::cerr << "Failed to open distortion coefficients file." << std::endl;
        }
    }
    else if (request->transform_request[0] == "Gimbal" && request->transform_request[1] == "Camera")
    {
        response->transform_reply[0] = "Gimbal";
        response->transform_reply[1] = "Camera";
        std::ifstream trans_file(gimbal_camera_path);
        if (trans_file.is_open())
        {
            std::vector<double> trans;
            std::string line;
            while (std::getline(trans_file, line))
            {
                std::stringstream ss(line);
                double value;
                while (ss >> value)
                {
                    trans.push_back(value);
                }
            }
            trans_file.close();

            // 将畸变系数写入到 response 中
            response->transform = trans;
        }
        else
        {
            std::cerr << "Failed to open distortion coefficients file." << std::endl;
        }
    }
    std::cout << "Successfully sent transform for request " << request->request_name << std::endl;
    // RCLCPP_INFO(this->get_logger(), "Successfully sent camera for request_name: %d", request->request_name);
}