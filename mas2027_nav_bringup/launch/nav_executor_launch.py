"""Standalone map_server + MINCO + MPC navigation bringup."""

import glob
import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_navigation_map_files(bringup_dir):
    """Load and validate the single map-selection manifest."""
    config_path = os.path.join(bringup_dir, "config", "navigation_map.yaml")
    with open(config_path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)

    if not isinstance(document, dict) or not isinstance(document.get("map_files"), dict):
        raise RuntimeError(f"{config_path}: missing map_files mapping")

    def resolve(key):
        value = document["map_files"].get(key)
        if not isinstance(value, str) or not value.strip():
            raise RuntimeError(f"{config_path}: map_files.{key} must be a non-empty path")
        path = value if os.path.isabs(value) else os.path.join(bringup_dir, value)
        path = os.path.realpath(path)
        if not os.path.isfile(path):
            raise RuntimeError(f"{config_path}: map_files.{key} does not exist: {path}")
        return path

    return {
        "localization_pcd": resolve("localization_pcd"),
        "occupancy_yaml": resolve("occupancy_yaml"),
        "terrain_msgpack": resolve("terrain_msgpack"),
    }


def _load_map_source(executor_dir):
    """Use the executor YAML as the single startup mode selector."""
    config_path = os.path.join(executor_dir, "config", "node_params.yaml")
    with open(config_path, encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    try:
        map_source = document["/**"]["ros__parameters"]["node"]["map_source"]
    except (KeyError, TypeError) as exc:
        raise RuntimeError(f"{config_path}: missing node.map_source") from exc
    if map_source not in ("prior", "online"):
        raise RuntimeError(f"{config_path}: node.map_source must be prior or online")
    return map_source


def _launch_setup(context):
    bringup_dir = get_package_share_directory("mas2027_nav_bringup")
    executor_dir = get_package_share_directory("mas2027_nav_executor")
    map_source = _load_map_source(executor_dir)
    online = map_source == "online"
    launch_dir = os.path.join(bringup_dir, "launch")
    system_first_library_path = os.pathsep.join(
        ["/lib/x86_64-linux-gnu", os.environ.get("LD_LIBRARY_PATH", "")]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("use_rviz")
    use_nav_serial = LaunchConfiguration("use_nav_serial")
    nav_serial_device = LaunchConfiguration("nav_serial_device")
    use_odom_localizer = LaunchConfiguration("use_odom_localizer")
    use_foxglove = LaunchConfiguration("use_foxglove")
    foxglove_address = LaunchConfiguration("foxglove_address")
    foxglove_port = LaunchConfiguration("foxglove_port")
    output_topic = LaunchConfiguration("output_topic")
    normal_mode = LaunchConfiguration("normal_mode")
    normal_mode_parameter = ParameterValue(normal_mode, value_type=int)

    lio_params = os.path.join(bringup_dir, "config", "small_point_lio_params.yaml")
    map_files = {} if online else _load_navigation_map_files(bringup_dir)
    executor_params = sorted(glob.glob(os.path.join(executor_dir, "config", "*.yaml")))
    localizer_params = os.path.join(
        get_package_share_directory("odom_localizer"), "config", "params.yaml"
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, "robot_state_publisher_launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    mid360_driver = Node(
        package="mid360_driver",
        executable="mid360_driver_node",
        name="mid360_driver",
        output="screen",
        parameters=[lio_params, {"use_sim_time": use_sim_time}],
    )
    small_point_lio = Node(
        package="small_point_lio",
        executable="small_point_lio_node",
        name="small_point_lio",
        output="screen",
        parameters=[lio_params, {"use_sim_time": use_sim_time, "publish_odom_tf": False}],
    )
    odom_localizer = Node(
        package="odom_localizer",
        executable="odom_localizer_node",
        name="odom_localizer",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_odom_localizer),
        additional_env={"LD_LIBRARY_PATH": system_first_library_path},
        parameters=[
            localizer_params,
            {
                "map.prior_pcd_file": map_files.get("localization_pcd", ""),
                "use_sim_time": use_sim_time,
                "tf.publish_direct": False,
            },
        ],
    )
    tf_maintainer = Node(
        package="tf_maintainer",
        executable="tf_maintainer_node",
        name="tf_maintainer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "publish_odom_to_base": True,
                "publish_map_to_odom": not online,
            }
        ],
    )
    online_params = {
        "planner.exploration.unknown_as_occupied": True,
        "planner.rog_map.projection.unknown_as_occupied": True,
        "planner.rog_map.projection.prior_map.enable": False,
        "planner.allow_unknown": False,
    } if online else {}
    # Speed and acceleration limits come from the same YAML files in both modes.
    nav_executor = Node(
        package="mas2027_nav_executor",
        executable="mas2027_nav_executor_node",
        output="screen",
        emulate_tty=True,
        additional_env={
            "LD_LIBRARY_PATH": system_first_library_path,
            "OMP_NUM_THREADS": "2",
            "OMP_WAIT_POLICY": "PASSIVE",
        },
        parameters=executor_params
        + [{"use_sim_time": use_sim_time, "node.topics.cmd_vel_pub": output_topic,
            "region_control.normal_mode": normal_mode_parameter,
            "node.map_source": map_source}, online_params],
    )
    terrain_map_server = Node(
        package="map_server",
        executable="map_server_node",
        name="terrain_map_server",
        output="screen",
        additional_env={"LD_LIBRARY_PATH": system_first_library_path},
        parameters=[{
            "use_sim_time": use_sim_time,
            "terrain_map_path": map_files.get("terrain_msgpack", ""),
            "map_yaml_path": map_files.get("occupancy_yaml", ""),
            "frame_id": "map",
        }],
        remappings=[
            ("cost_map", "/cost_map"),
            ("terrain_label_map", "/terrain_label_map"),
        ],
    )
    nav_serial_bridge = Node(
        package="nav_serial_bridge",
        executable="nav_serial_bridge_node",
        name="nav_serial_bridge",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_nav_serial),
        parameters=[{
            "device": nav_serial_device,
        }],
    )
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rviz_launch.py")),
        condition=IfCondition(use_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": os.path.join(bringup_dir, "rviz", "nav_executor_view.rviz"),
            "fixed_frame": "odom" if online else "map",
        }.items(),
    )
    # 默认随 nav 栈起 bridge：8766 避开 mapping Web UI 占用的 8765。
    # SSH 隧道场景绑 127.0.0.1；直连改 foxglove_address:=0.0.0.0。
    # 不需要可视化时用 use_foxglove:=False；已有独立 bridge 时也关掉，避免端口冲突。
    foxglove_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("foxglove_bridge"),
                "launch",
                "foxglove_bridge_launch.xml",
            )
        ),
        condition=IfCondition(use_foxglove),
        launch_arguments={
            "address": foxglove_address,
            "port": foxglove_port,
            "use_sim_time": use_sim_time,
        }.items(),
    )

    actions = [robot_state_publisher, mid360_driver, small_point_lio, tf_maintainer,
        nav_executor, nav_serial_bridge, rviz, foxglove_bridge]
    if not online:
        actions[3:3] = [odom_localizer, terrain_map_server]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        # RViz 默认关闭，需要图形窗口时显式传 use_rviz:=True。
        DeclareLaunchArgument("use_rviz", default_value="false"),
        # CDC1 发送 vx/vy、小 yaw ref 和 follow_gimbal 跟随模式请求；底盘角速度由下位机生成。
        # 只观察导航时用 use_nav_serial:=False 关掉硬件输出。
        DeclareLaunchArgument("use_nav_serial", default_value="True"),
        DeclareLaunchArgument(
            "nav_serial_device",
            default_value="",
        ),
        DeclareLaunchArgument("use_odom_localizer", default_value="True"),
        DeclareLaunchArgument("use_foxglove", default_value="True"),
        DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("foxglove_port", default_value="8766"),
        DeclareLaunchArgument("output_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("normal_mode", default_value="4"),
        OpaqueFunction(function=_launch_setup),
    ])
