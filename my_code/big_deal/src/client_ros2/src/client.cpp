#include "../include/client.hpp"
#include "../include/picture_process.hpp"

ClientWay::ClientWay(std::shared_ptr<rclcpp::Client<interfaces_ros2::srv::Interface>> client) : Node("client_way")
{
    new_requested_ = false;
    client_ = client;
}

void ClientWay::send_request(std::string type, int nameID,std::string from,std::string to)
{
    // 创建一个请求对象
    auto request = std::make_shared<interfaces_ros2::srv::Interface::Request>();

    // 设置image_id参数
    request->request_name = nameID;
    request->request_type = type;
    request->transform_request[0]=from;
    request->transform_request[1]=to;

    if (type == "image")
    {
        new_requested_ = true;
        // 发送请求，并在收到响应时调用 handle_img 方法处理
        auto result = client_->async_send_request(request,
                                                  std::bind(&ClientWay::handle_img, this, std::placeholders::_1));
    }
    else if (type == "video")
    {
        if (!new_requested_) // 只有在没有新的请求时才发出
        {
            current_video_id_ = nameID;
            new_requested_ = true;
            auto result = client_->async_send_request(request,
                                                      std::bind(&ClientWay::handle_video, this, std::placeholders::_1));
        }
    }
    else if (type == "transform")
    {
        auto result = client_->async_send_request(request,
                                                  std::bind(&ClientWay::handle_transform, this, std::placeholders::_1));
    }
    else if (type == "camera")
    {
        auto result = client_->async_send_request(request,
                                                  std::bind(&ClientWay::handle_camera, this, std::placeholders::_1));
    }
    else
    {
        std::cout << "request wrong" << std::endl;
    }
}

void ClientWay::handle_img(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future)//处理图片
{
    // 获取响应结果
    auto response = future.get();

    // 将收到的图像数据从 ROS 消息转换为 OpenCV 图像格式
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(response->image_data, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "cv_bridge exception: " << e.what() << std::endl;
        // RCLCPP_INFO(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // 显示图像
    cv::namedWindow("Received Image", cv::WINDOW_NORMAL);
    cv::resizeWindow("Received Image", 400, 300);
    // 重置标志位，以便新图像能够正常显示

    new_requested_ = false;
    // 显示图像，直到有新图像请求
    while (rclcpp::ok() && !new_requested_)
    {
        cv::imshow("Received Image", cv_ptr->image);
        if (cv::waitKey(10) >= 0)
        {
            break; // 如果用户按下键盘按键则退出显示
        }
    }
}

void ClientWay::handle_video(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future)//处理视频帧
{
    auto response = future.get();

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(response->image_data, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        std::cout << "cv_bridge exception: " << e.what() << std::endl;
        return;
    }
    cv::namedWindow("Received Video", cv::WINDOW_NORMAL);
    cv::resizeWindow("Received Video", 400, 300);
    cv::imshow("Received Video", cv_ptr->image);

    Process process(gimtranslation_, gimrotation_, camtranslation_, camrotation_);
    std::cout<<gimtranslation_[0]<<" "<<gimrotation_[0]<<" "<<camtranslation_[0]<<" "<<camrotation_[0]<<std::endl;
    picture_process(cv_ptr->image, process);

    if (cv::waitKey(30) >= 0) // 30ms延迟，以模拟视频播放的帧率
    {
        return;
    }

    new_requested_ = false; // 重置标志位，以便继续请求下一帧

    if (!new_requested_)
    {
        send_request("video", current_video_id_," "," "); // 使用相同的 nameID 请求下一帧
    }
}

void ClientWay::handle_transform(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future)//平移和唯一向量
{
    auto response = future.get();
    std::vector<double> trans = response->transform;
    if(response->transform_reply[0] == "Odom" && response->transform_reply[1] == "Gimbal")
    {
        // 检查 trans 的大小是否满足需要
        if (trans.size() >= 7)
        {
            // 先存储旋转向量 gimrotation_ (4 个元素)
            for (int i = 0; i < 4; ++i)
            {
                gimrotation_[i] = trans[i];
            }

            // 再存储平移向量 gimtranslation_ (3 个元素)
            for (int i = 0; i < 3; ++i)
            {
                gimtranslation_[i] = trans[i + 4];
            }

            std::cout << "Odom to Gimbal rotation and translation data saved." << std::endl;
        }
        else
        {
            std::cerr << "Insufficient data in transform vector." << std::endl;
        }
    }
    if(response->transform_reply[0] == "Gimbal" && response->transform_reply[1] == "Camera")
    {
        // 检查 trans 的大小是否满足需要
        if (trans.size() >= 7)
        {
            // 先存储旋转向量 gimrotation_ (4 个元素)
            for (int i = 0; i < 4; ++i)
            {
                camrotation_[i] = trans[i];
            }

            // 再存储平移向量 gimtranslation_ (3 个元素)
            for (int i = 0; i < 3; ++i)
            {
                camtranslation_[i] = trans[i + 4];
            }

            std::cout << "Gimbal to Camera rotation and translation data saved." << std::endl;
        }
        else
        {
            std::cerr << "Insufficient data in transform vector." << std::endl;
        }
    }
}

void ClientWay::handle_camera(rclcpp::Client<interfaces_ros2::srv::Interface>::SharedFuture future)//相机矩阵和畸变系数
{
    auto response = future.get();

    // 获取相机矩阵和畸变系数
    std::vector<double> camera_matrix = response->camerafile;
    std::vector<double> dist_coeffs = response->distcoeffs;

    // 将相机矩阵写入文件
    std::ofstream matrix_file("./src/client_ros2/data/camera/matrix.txt");
    if (matrix_file.is_open())
    {
        int cnt_line=0;//换行
        for (const auto &value : camera_matrix)
        {
            matrix_file << value << " ";
            cnt_line++;
            if(cnt_line%3==0)
            {
                matrix_file<<"\n";
            }
        }
        matrix_file.close();
        std::cout << "Camera matrix saved successfully." << std::endl;
    }
    else
    {
        std::cerr << "Failed to open matrix file for writing." << std::endl;
    }

    // 将畸变系数写入文件
    std::ofstream dist_file("./src/client_ros2/data/camera/distcoeffs.txt");
    if (dist_file.is_open())
    {
        for (const auto &value : dist_coeffs)
        {
            dist_file << value << " ";
        }
        dist_file.close();
        std::cout << "Distortion coefficients saved successfully." << std::endl;
    }
    else
    {
        std::cerr << "Failed to open distortion coefficients file for writing." << std::endl;
    }
}

void ClientWay::get_input()//获取输入
{
    while (rclcpp::ok())
    {
        std::string input;
        std::cout << "Enter command (image x/video x/camera/close/transform from to x to y ): ";
        std::getline(std::cin, input);

        std::istringstream iss(input);
        std::string command;
        std::string from_name, to_name, other;
        int name_id;
        iss >> command;
        if (command == "image" || command == "video")
        {
            iss >> name_id;
            if (command == "image")
            {
                std::cout << "image ID is :" << name_id << std::endl;
                send_request(command, name_id, from_name, to_name);
            }
            else if (command == "video")
            {
                std::cout << " video ID is :" << name_id << std::endl;
                send_request(command, name_id, from_name, to_name);
            }
            else
            {
                std::cout << "Invalid command. " << std::endl;
            }
        }
        else if (command == "transform")
        {
            iss >> other >> from_name >> other >> to_name;
            send_request(command, 0, from_name, to_name);
            // std::cout<<from_name<<"  "<<to_name<<std::endl;
        }
        else if (command == "camera")
        {
            send_request(command, 0, from_name, to_name);
        }
        else if (command == "close")
        {
            rclcpp::shutdown();
        }
        else
        {
            std::cout << "Invalid input. Please enter in the format 'image/video x','transform from to X to Y'." << std::endl;
        }
    }
}