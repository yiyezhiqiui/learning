cmake_minimum_required(VERSION 3.8)
project(status_management)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE "Debug")

find_package(ament_cmake_auto REQUIRED)
find_package(std_msgs REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  DIRECTORY src
)

install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}
)
ament_auto_package()