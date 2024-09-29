cmake_minimum_required(VERSION 3.16)
project(btcpp_ros2_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成接口，特别是 .action 文件
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Sleep.action"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
