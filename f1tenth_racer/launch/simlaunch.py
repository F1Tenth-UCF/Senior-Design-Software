import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def format_share_path(rel_path, package='f1tenth_racer'):
    return os.path.join(get_package_share_directory(package), *rel_path.split('/'))

def generate_launch_description():

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('f1tenth_racer'), 'urdf', 'model.urdf'])
    ])

    # publish robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                "robot_description": robot_description_content
            }
        ]
    )

    # fuse the IMU and Odom data, and publish the odom -> base_link transform
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[format_share_path('config/ekf.yaml')]
    )

    return LaunchDescription([
        node_robot_state_publisher,
        robot_localization_node,
    ])
