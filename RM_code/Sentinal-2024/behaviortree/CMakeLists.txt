cmake_minimum_required(VERSION 3.8)
project(behaviortree)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)

find_package(behaviortree_cpp REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}_node /usr/local/lib/libbehaviortree_cpp.so)

# install(
# DESTINATION share/${PROJECT_NAME}
# )
ament_auto_package()
