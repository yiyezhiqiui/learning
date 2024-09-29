cmake_minimum_required(VERSION 3.8)
project(goal_client)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE "Debug")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  DIRECTORY src
)

ament_auto_package()
