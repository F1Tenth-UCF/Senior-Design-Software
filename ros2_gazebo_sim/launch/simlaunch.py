import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
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

    # publish robot state
    doc = xacro.parse(open(format_share_path('urdf/macros.xacro')))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # spawn car
    # car_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(format_share_path('launch/ros_gz_spawn_model.launch.py', package="ros_gz_sim")),
    #     launch_arguments=[
    #         ("world", "default"),
    #         ("file", format_share_path('urdf/model.urdf')),
    #         ("entity_name", LaunchConfiguration('car_name')),
    #         ("x", LaunchConfiguration('x_pos')),
    #         ("y", LaunchConfiguration('y_pos')),
    #         ("z", LaunchConfiguration('z_pos')),
    #         ("bridge_name", "ros_gz_bridge"),
    #         ("config_file", format_share_path('config/topic_config.yaml')),
    #         ("paint", LaunchConfiguration('paint'))
    #     ]
    # )

    return LaunchDescription([
        arg_car_name,
        arg_x_pos,
        arg_y_pos,
        arg_z_pos,
        arg_paint,
        arg_track,
        gazebo_include,
        node_robot_state_publisher,
        # car_include
    ])