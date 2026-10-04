import os

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_path('robin_description')
    bringup_share = get_package_share_path('robin_bringup')

    urdf_path = os.path.join(description_share, 'urdf', 'my_robot.urdf.xacro')
    gazebo_config_path = os.path.join(
        bringup_share, 'config', 'gazebo_bridge.yaml'
    )
    gazebo_launch_path = os.path.join(
        get_package_share_path('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    )
    rviz_config_path = os.path.join(
        description_share, 'rviz', 'urdf_config.rviz'
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
    )

    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description'],
    )

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gazebo_config_path}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_node,
        spawn_robot_node,
        bridge_node,
        rviz_node,
    ])