import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, TimerAction


def generate_launch_description():
    
    package_description = "fastbot_description"
    robot_name = "fastbot"
    xacro_file = "fastbot.urdf.xacro"

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'robot.rviz')

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), "models/urdf/", xacro_file
    )
    # Load XACRO file with ARGUMENTS
    robot_desc = xacro.process_file(
        robot_desc_path, mappings={"robot_name": robot_name}
    )
    xml = robot_desc.toxml()

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": xml}],
        output="screen",
    )
    
    # Joint State Publisher Node with a 2-second delay
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # RVIZ Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )
    delayed_rviz = TimerAction(
        actions=[rviz_node],
        period=2.0
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        delayed_rviz,
    ])
