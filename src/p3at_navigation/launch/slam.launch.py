import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory("p3at_navigation")
    slam_dir = get_package_share_directory("slam_toolbox")
    nav2_dir = get_package_share_directory("nav2_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("lidar_noise", default_value="0.01"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_dir, "launch", "simulation.launch.py")),
            launch_arguments={"lidar_noise": LaunchConfiguration("lidar_noise")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam_dir, "launch", "online_async_launch.py")),
            launch_arguments={"slam_params_file": os.path.join(nav_dir, "config", "slam_toolbox.yaml"), "use_sim_time": "true"}.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(nav2_dir, "rviz", "nav2_default_view.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ])
