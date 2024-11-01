cmake_minimum_required(VERSION 3.8)
project(behaviortree)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)

find_package(behaviortree_cpp REQUIRED)
ament_auto_find_build_dependencies()

# 查找 src 目录中的所有 cpp 文件
file(GLOB_RECURSE SRC_FILES src/*.cpp)



# install(
# DESTINATION share/${PROJECT_NAME}
# )
ament_auto_package()
