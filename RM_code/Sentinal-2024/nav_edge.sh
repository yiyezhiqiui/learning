#!/bin/bash
source ./install/setup.bash

sudo chmod 777 /dev/ttyACM0

ros2 launch communicate launch.py &
pid10=$!
echo -e "communicate pid: $pid10\n\n"

ros2 run goal_client goal_client_node & 
pid9=$!
echo -e "goal_client pid: $pid9\n\n"

ros2 launch rm_bringup bringup.launch.py &
pid8=$!
echo -e "rm_bringup pid: $pid8\n\n"

ros2 launch livox_ros_driver2 msg_MID360_launch.py &
pid7=$!
echo -e "livox_ros_driver2 pid: $pid7\n\n"

ros2 launch linefit_ground_segmentation_ros segmentation.launch.py &
pid6=$!
echo -e "linefit_ground_segmentation_ros pid: $pid6\n\n"

ros2 launch fast_lio mapping.launch.py &
pid5=$!
echo -e "fast_lio pid: $pid5\n\n"

ros2 launch imu_complementary_filter complementary_filter.launch.py &
pid4=$!
echo -e "imu_complementary_filter pid: $pid4\n\n"

ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py &
pid3=$!
echo -e "pointcloud_to_laserscan pid: $pid3\n\n"

ros2 launch icp_localization_ros2 bringup.launch.py &
pid2=$!
echo -e "icp_localization_ros2 pid: $pid2\n\n"

ros2 launch rm_navigation bringup_launch.py &
pid1=$!
echo -e "rm_navigation pid: $pid1\n\n" 

# 当捕获到 SIGINT 信号时，结束这两个进程
trap "kill $pid10 $pid9 $pid8 $pid7 $pid6 $pid5 $pid4 $pid3 $pid2 $pid1" SIGINT

# 等待所有后台进程结束
wait