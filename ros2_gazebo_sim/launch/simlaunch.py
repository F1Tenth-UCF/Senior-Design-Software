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
from launch_ros.actions import PushROSNamespace
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_yaml.launch_description_sources import YAMLLaunchDescriptionSource
import xacro

def format_share_path(rel_path, package='ros2_gazebo_sim'):
    return os.path.join(get_package_share_directory(package), *rel_path.split('/'))

def generate_launch_description():
    
    # arguments
    arg_car_name = DeclareLaunchArgument(
        'car_name',
        default_value=TextSubstitution(text="car_1")
    )
    arg_x_pos = DeclareLaunchArgument(
        'x_pos',
        default_value=TextSubstitution(text="0")
    )
    arg_y_pos = DeclareLaunchArgument(
        'y_pos',
        default_value=TextSubstitution(text="0")
    )
    arg_z_pos = DeclareLaunchArgument(
        'z_pos',
        default_value=TextSubstitution(text="0.05")
    )
    arg_paint = DeclareLaunchArgument(
        'paint',
        default_value=TextSubstitution(text="Blue")
    )
    arg_track = DeclareLaunchArgument(
        'track',
        default_value=TextSubstitution(text=format_share_path('world/race_track.world'))
    )

    # launch gazebo
    os.environ['GZ_SIM_RESOURCE_PATH'] = get_package_share_directory('ros2_gazebo_sim')
    gazebo_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(format_share_path('launch/gz_sim.launch.py', package="ros_gz_sim")),
        launch_arguments=[
            ("gz_args", LaunchConfiguration('track'))
        ]
    )

    # get robot description and controller description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('ros2_gazebo_sim'),
                 'urdf', 'macros.xacro']
            ),
        ]
    )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('ros2_gazebo_sim'),
            'config',
            'control.yaml',
        ]
    )

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

    # spawn car
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', 
                   '-name', LaunchConfiguration('car_name'), 
                   '-allow_renaming', 'true'
                   '-x', LaunchConfiguration('x_pos'),
                   '-y', LaunchConfiguration('y_pos'),
                   '-z', LaunchConfiguration('z_pos'),],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    ackermann_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ackermann_controller',
                   '--param-file',
                   robot_controllers,
                   ],
    )

    # fuse the IMU and Odom data, and publish the odom -> base_link transform
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[format_share_path('config/ekf.yaml'), {'use_sim_time': True}]
    )

    # bridge gazebo topics to ROS
    bridge_params = format_share_path('config/topic_config.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen'
    )

    return LaunchDescription([
        arg_car_name,
        arg_x_pos,
        arg_y_pos,
        arg_z_pos,
        arg_paint,
        arg_track,
        bridge,
        gazebo_include,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ackermann_steering_controller_spawner],
            )
        ),
        node_robot_state_publisher,
        gz_spawn_entity,
        robot_localization_node,
    ])