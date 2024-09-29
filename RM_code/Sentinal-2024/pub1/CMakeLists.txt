cmake_minimum_required(VERSION 3.8)
project(pub1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)
set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  DIRECTORY src
)

# install(
# DESTINATION share/${PROJECT_NAME}
# )
ament_auto_package()
