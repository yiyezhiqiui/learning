import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
sys.path.append(os.path.join(get_package_share_directory('rm_bringup'), 'launch'))


def generate_launch_description():

    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node,SetParameter
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription
    
    robot_description = Command(['xacro ', os.path.join(
        get_package_share_directory('rm_robot_description'), 'urdf', 'rm_robot.urdf.xacro')])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                    'publish_frequency': 1000.0}]
    )

    return LaunchDescription([robot_state_publisher])
