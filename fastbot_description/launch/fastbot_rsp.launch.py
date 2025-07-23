import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    package_description = "fastbot_description"
    robot_name = "fastbot"
    xacro_file = "fastbot.urdf.xacro"

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
        parameters=[{"use_sim_time": False, "robot_description": xml}],
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
    ])
