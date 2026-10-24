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
    mirror_points = LaunchConfiguration("mirror_points")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share, "config", "decision.yaml"),
                description="Decision parameter file",
            ),
            DeclareLaunchArgument(
                "points_file",
                default_value=os.path.join(share, "config", "points.yaml"),
                description="Taught point table",
            ),
            DeclareLaunchArgument(
                "enabled",
                default_value="false",
                description="Publish navigation goals (safety switch)",
            ),
            DeclareLaunchArgument(
                "goal_frame",
                default_value="map",
                description="Goal frame: map (prior localization) or odom (pure online)",
            ),
            DeclareLaunchArgument(
                "mirror_points",
                default_value="false",
                description="Mirror point table across the field centre line",
            ),
            Node(
                package="mas2027_decision",
                executable="mas2027_decision_node",
                name="mas2027_decision",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "points_file": ParameterValue(points_file, value_type=str),
                        "enabled": ParameterValue(enabled, value_type=bool),
                        "goal_frame": ParameterValue(goal_frame, value_type=str),
                        "mirror_points": ParameterValue(mirror_points, value_type=bool),
                    },
                ],
            ),
        ]
    )
