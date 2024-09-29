# Sentinal-2024
2024哨兵视觉仓库

## 1.框架

### （1）决策部分
- **behaviortree**

### （2）导航部分
- [**fast_lio**](https://github.com/Ericsii/FAST_LIO) (提供里程计)
- [**icp_localization_ros2**](https://github.com/leggedrobotics/icp_localization) (提供map到odom的坐标变换)
- [**imu_complementary_filter**](https://github.com/CCNYRoboticsLab/imu_tools) (IMU滤波)
- [**linefit_ground_segmentation_ros2**](https://github.com/lorenwel/linefit_ground_segmentation) (点云分割)
- [**livox_ros_driver2**](https://github.com/Livox-SDK/livox_ros_driver2) (MID360驱动)
- [**pointcloud_to_laserscan**](https://github.com/ros-perception/pointcloud_to_laserscan) (将PointCloud2转换为LaserScan)
- **rm_bringup** (启动robot_state_publisher)
- **rm_navigation** (Nav2的launch和参数)
- **rm_robot_description**(机器人的urdf)

### （3）通讯部分
- **communicate**(上下位机通讯模块)
- **goal_client**（订阅目的地坐标，发布目的地的动作）
- **pub1**（测试模块，发布一个坐标，测试导航系统能否成功运行）
- **status_management**（记录哨兵此时的状态，避免重复发布相同的命令）

### （4）启动文件
- **mapping.sh**(建图)
- **nav.sh**(导航)
- **nav_edge.sh**(导航)