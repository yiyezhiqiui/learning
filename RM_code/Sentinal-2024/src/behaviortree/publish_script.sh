#!/bin/zsh

# Source ROS 2 environment and workspace setup
source /opt/ros/humble/setup.bash
source install/setup.sh

# Publishing game info
ros2 topic pub -r 10 /communicate/gameinfo std_msgs/msg/Int32MultiArray "{
    data: [100, 150, 500, 500, 5000, 100, 1000]
}" &

# Publishing artificial location
ros2 topic pub -r 10 /communicate/artificial/location std_msgs/msg/Int32MultiArray "{
    data: [10, 10]
}" &

# Publishing position info
ros2 topic pub -r 1 /communicate/position std_msgs/msg/Int32MultiArray "{
    data: [5, 10]
}" &

# Wait for all processes to finish
wait
