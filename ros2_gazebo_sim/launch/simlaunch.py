import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def format_share_path(rel_path, package='ros2_gazebo_sim'):
    return os.path.join(get_package_share_directory(package), *rel_path.split('/'))

def generate_launch_description():
    # # Create the wheel odometry node
    # wheel_odom_node = Node(
    #     package='ros2_gazebo_sim',
    #     executable='wheel_odometry_node',
    #     name='wheel_odometry_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}]
    # )

    # fuse the IMU and Odom data, and publish the odom -> base_link transform
    # Add a delay to ensure wheel odometry is running
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[format_share_path('config/ekf.yaml'), {'use_sim_time': False}]
    )

    # Delay the start of robot_localization to ensure wheel odometry is publishing
    delayed_robot_localization = TimerAction(
        period=2.0,  # 2 second delay
        actions=[robot_localization_node]
    )

    return LaunchDescription([
        # wheel_odom_node,
        robot_localization_node
        # delayed_robot_localization
    ])