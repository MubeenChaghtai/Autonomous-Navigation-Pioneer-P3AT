import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory("p3at_navigation")
    description_dir = get_package_share_directory("p3at_description")
    gz_dir = get_package_share_directory("ros_gz_sim")
    world = LaunchConfiguration("world")
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=os.path.join(nav_dir, "worlds", "office.sdf")),
        DeclareLaunchArgument("x", default_value="2.5"),
        DeclareLaunchArgument("y", default_value="3.0"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("lidar_noise", default_value="0.01"),
        DeclareLaunchArgument("headless", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gz_dir, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": ["-r ", world], "on_exit_shutdown": "true"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(description_dir, "launch", "description.launch.py")),
            launch_arguments={"use_sim_time": "true", "lidar_noise": LaunchConfiguration("lidar_noise")}.items(),
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            output="screen",
            arguments=["-topic", "robot_description", "-name", "p3at", "-x", LaunchConfiguration("x"), "-y", LaunchConfiguration("y"), "-z", "0.05", "-Y", LaunchConfiguration("yaw")],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            output="screen",
            parameters=[{"config_file": os.path.join(nav_dir, "config", "bridge.yaml"), "use_sim_time": True}],
        ),
    ])
