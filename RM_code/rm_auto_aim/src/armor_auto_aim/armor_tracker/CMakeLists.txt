cmake_minimum_required(VERSION 3.10)
project(armor_tracker)

# Use C++20
SET(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
# enforcing cleaner code.
add_definitions(-Wall -Werror)

# Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

######################
# Find dependencies ##
######################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

##########
# Build ##
##########
ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN armor::ArmorTrackerNode
    EXECUTABLE ${PROJECT_NAME}_node
)

############
# Install ##
############
ament_auto_package(
    INSTALL_TO_SHARE
)
