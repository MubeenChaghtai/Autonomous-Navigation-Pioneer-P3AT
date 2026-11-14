import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_dir = get_package_share_directory("p3at_navigation")
    nav2_dir = get_package_share_directory("nav2_bringup")
    params = RewrittenYaml(
        source_file=LaunchConfiguration("params_file"),
        param_rewrites={
            "use_sim_time": "True",
            "robot_radius": "0.38",
            "base_frame_id": "base_footprint",
            "robot_base_frame": "base_footprint",
            "laser_max_range": "10.0",
            "max_particles": "3000",
            "alpha1": "0.3",
            "alpha2": "0.2",
            "alpha3": "0.3",
            "alpha4": "0.2",
            "recovery_alpha_slow": "0.001",
            "recovery_alpha_fast": "0.1",
            "inflation_radius": "0.8",
        },
        convert_types=True,
    )
    return LaunchDescription([
        DeclareLaunchArgument("map", default_value=os.path.join(nav_dir, "maps", "office.yaml")),
        DeclareLaunchArgument("params_file", default_value=os.path.join(nav2_dir, "params", "nav2_params.yaml")),
        DeclareLaunchArgument("lidar_noise", default_value="0.01"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("run_goals", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_dir, "launch", "simulation.launch.py")),
            launch_arguments={"lidar_noise": LaunchConfiguration("lidar_noise")}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_dir, "launch", "bringup_launch.py")),
            launch_arguments={"map": LaunchConfiguration("map"), "params_file": params, "use_sim_time": "true", "autostart": "true", "slam": "False"}.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(nav2_dir, "rviz", "nav2_default_view.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
        TimerAction(
            period=15.0,
            actions=[Node(
                package="p3at_navigation",
                executable="goal_runner",
                output="screen",
                parameters=[{"goals_file": os.path.join(nav_dir, "config", "goals.yaml"), "use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("run_goals")),
            )],
        ),
    ])
