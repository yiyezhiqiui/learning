import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
# from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('communicate'),
        'config', 'config.yaml'
    )

    load_nodes=GroupAction(
        actions=[
            Node(
                package='communicate',
                executable='communicate_node',
                output='screen',
                parameters=[config]
            ),
        ]
    )
    ld = LaunchDescription()
    ld.add_action(load_nodes)
    return ld