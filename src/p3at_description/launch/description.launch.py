import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory("p3at_description"), "urdf", "p3at.urdf.xacro")
    description = ParameterValue(Command(["xacro ", xacro_file, " lidar_noise:=", LaunchConfiguration("lidar_noise")]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("lidar_noise", default_value="0.01"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": description, "use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ])
