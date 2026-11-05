"""假决策测试 launch。

用法：
  ros2 launch mas2027_decision decision_test.launch.py

默认行为（读取 config/test_points.yaml）：
  原地等 5 s -> a1 停 5 s -> a2 停 5 s -> a3 停 5 s -> 回到 a1 循环。
  把 debug_hp_ratio 调到 0.3 以下会返回 home，调回 0.6 以上继续巡逻：
    ros2 param set /mas2027_decision_test debug_hp_ratio 0.2
    ros2 param set /mas2027_decision_test debug_hp_ratio 0.7

导航节点不在这里启动，保持和正式决策相同的分工：
先启动 nav（nav_executor / 定位 / TF），再启动本 launch。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("mas2027_decision")

    params_file = LaunchConfiguration("params_file")
    points_file = LaunchConfiguration("points_file")
    enabled = LaunchConfiguration("enabled")
    goal_frame = LaunchConfiguration("goal_frame")
    start_delay_s = LaunchConfiguration("start_delay_s")
    dwell_s = LaunchConfiguration("dwell_s")
    arrive_tolerance_m = LaunchConfiguration("arrive_tolerance_m")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share, "config", "test_decision.yaml"),
                description="Test decision parameter file",
            ),
            DeclareLaunchArgument(
                "points_file",
                default_value=os.path.join(share, "config", "test_points.yaml"),
                description="Test point table (home / a1 / a2 / a3)",
            ),
            DeclareLaunchArgument(
                "enabled",
                default_value="true",
                description="Publish navigation goals (test launch defaults to on)",
            ),
            DeclareLaunchArgument(
                "goal_frame",
                default_value="map",
                description="Goal frame: map (prior localization) or odom (pure online)",
            ),
            DeclareLaunchArgument(
                "start_delay_s",
                default_value="5.0",
                description="Seconds to stay still after start",
            ),
            DeclareLaunchArgument(
                "dwell_s",
                default_value="5.0",
                description="Seconds to dwell at each patrol point",
            ),
            DeclareLaunchArgument(
                "arrive_tolerance_m",
                default_value="0.4",
                description="Distance to goal that counts as arrived",
            ),
            Node(
                package="mas2027_decision",
                executable="mas2027_decision_test_node",
                name="mas2027_decision_test",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "points_file": ParameterValue(points_file, value_type=str),
                        "enabled": ParameterValue(enabled, value_type=bool),
                        "goal_frame": ParameterValue(goal_frame, value_type=str),
                        "start_delay_s": ParameterValue(
                            start_delay_s, value_type=float
                        ),
                        "dwell_s": ParameterValue(dwell_s, value_type=float),
                        "arrive_tolerance_m": ParameterValue(
                            arrive_tolerance_m, value_type=float
                        ),
                    },
                ],
            ),
        ]
    )
