import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro
from launch.events import matches_action
from launch.event_handlers import OnExecutionComplete
import time

def format_share_path(rel_path, package='f1tenth_racer'):
    return os.path.join(get_package_share_directory(package), *rel_path.split('/'))

def generate_launch_description():

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('f1tenth_racer'), 'urdf', 'model.urdf'])
    ])

    # start the lidar
    urg2_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'urg_node2', 'urg_node2.launch.py'],
    )

    # start the mavros bridge
    open_port_cmd = ExecuteProcess(
        cmd=['sudo', 'chmod', '777', '/dev/ttyTHS0'],
    )
    mavros_node = ExecuteProcess(
        cmd=['ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '--param', 'fcu_url:=/dev/ttyTHS0:921600'],
    )
    
    set_stream_rate_srv = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/mavros/set_stream_rate', 'mavros_msgs/srv/StreamRate', '{stream_id: 0, message_rate: 20, on_off: 1}'],
    )
    set_imu_frame_id_param = ExecuteProcess(
        cmd=['ros2', 'param', 'set', '/mavros/imu', 'frame_id', 'imu'],
    )
    
    # publish robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                "robot_description": robot_description_content,
                # "use_tf_static": False
            }
        ]
    )

    # fuse the IMU and Odom data, and publish the odom -> base_link transform
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[format_share_path('config/ekf.yaml')]
    )

    # slam tool box
    slam_toolbox_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', 'use_sim_time:=False'],
    )

    # nav2
    nav2_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'params_file:=' + format_share_path('config/nav2_params.yaml')],
    )

    # Helper function to create a topic checker node
    def create_topic_checker(topic_name, topic_type):
        return ExecuteProcess(
            cmd=['ros2', 'run', 'f1tenth_racer', 'topic_checker_node', '--ros-args', '--param', 'topic_name:=' + topic_name, '--param', 'topic_type:=' + topic_type],
            output='screen'
        )

    # Create checker nodes
    scan_checker = create_topic_checker('/scan', 'LaserScan')
    odom_checker = create_topic_checker('/mavros/local_position/odom', 'Odometry')
    odom_transform_checker = create_topic_checker('/odometry/filtered', 'Odometry')
    map_checker = create_topic_checker('/map', 'OccupancyGrid')

    return LaunchDescription([
        urg2_node, # start lidar
        open_port_cmd, # open fcu port
        node_robot_state_publisher, # publish the base_link -> imu/laser transform
        RegisterEventHandler(
            OnProcessExit(
                target_action=open_port_cmd, # wait for fcu port to be open
                on_exit=[mavros_node, odom_checker] # start mavros and start listening for odom
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=odom_checker, # wait for odom to be available
                on_exit=[set_stream_rate_srv, set_imu_frame_id_param, robot_localization_node, odom_transform_checker] # set stream rate and imu frame id, start ekf and start listening for fused odometry
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=odom_transform_checker, # wait for fused odometry to be available
                on_exit=[scan_checker] # start scan checker
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=scan_checker, # wait for scan to be available. At this point, we have the full odom->sensors transform and the scan
                on_exit=[slam_toolbox_node, map_checker] # start slam toolbox and start listening for map
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=map_checker, # wait for map to be available
                on_exit=[nav2_node] # start nav2
            )
        )
    ])
