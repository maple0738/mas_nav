"""Standalone map_server + MINCO + MPC navigation bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mas2027_nav_bringup.configuration import parameter_files, read_manifest, resolve_map_files


def _launch_setup(context):
    bringup_dir = get_package_share_directory("mas2027_nav_bringup")
    manifest = read_manifest(
        LaunchConfiguration("navigation_config").perform(context),
        LaunchConfiguration("map_source").perform(context).strip(),
    )
    map_source = manifest["map_source"]
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
    control_overrides = {}
    output_topic = LaunchConfiguration("output_topic").perform(context).strip()
    if output_topic:
        control_overrides["node.topics.cmd_vel_pub"] = output_topic
    normal_mode = LaunchConfiguration("normal_mode").perform(context).strip()
    if normal_mode:
        control_overrides["region_control.normal_mode"] = int(normal_mode)

    lio_params = os.path.join(bringup_dir, "config", "small_point_lio_params.yaml")
    map_files = {} if online else resolve_map_files(manifest)
    config_dir = LaunchConfiguration("config_dir").perform(context)
    executor_params = parameter_files(config_dir)
    os.makedirs(manifest["log_dir"], exist_ok=True)
    startup_params = {
        "use_sim_time": use_sim_time,
        "node.map_source": map_source,
        "planner.rog_map.performance.summary_csv_path": os.path.join(manifest["log_dir"], "rog_map_perf_summary.csv"),
        "planner.rog_map.performance.detailed_csv_path": os.path.join(manifest["log_dir"], "rog_map_perf_detailed.csv"),
        **control_overrides,
    }
    if not online:
        startup_params["planner.rog_map.projection.prior_map.yaml_path"] = map_files["occupancy_yaml"]
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
    odom_localizer = None
    if not online:
        odom_localizer = Node(
            package="odom_localizer",
            executable="odom_localizer_node",
            name="odom_localizer",
            output="screen",
            emulate_tty=True,
            condition=IfCondition(use_odom_localizer),
            additional_env={"LD_LIBRARY_PATH": system_first_library_path},
            parameters=[
                os.path.join(get_package_share_directory("odom_localizer"), "config", "params.yaml"),
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
        + [startup_params, online_params],
    )
    terrain_map_server = None
    if not online:
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
    foxglove_bridge = None
    if use_foxglove.perform(context).lower() in ("true", "1"):
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

    actions = [LogInfo(msg=f"Navigation: mode={map_source}, manifest={manifest['path']}, "
        f"config_dir={os.path.realpath(os.path.expanduser(config_dir))}, maps={map_files}, logs={manifest['log_dir']}"),
        robot_state_publisher, mid360_driver, small_point_lio, tf_maintainer,
        nav_executor, nav_serial_bridge, rviz, *([foxglove_bridge] if foxglove_bridge else [])]
    if not online:
        actions[4:4] = [odom_localizer, terrain_map_server]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("navigation_config", default_value=os.path.join(
            get_package_share_directory("mas2027_nav_bringup"), "config", "navigation_map.yaml")),
        DeclareLaunchArgument("config_dir", default_value=os.path.join(
            get_package_share_directory("mas2027_nav_executor"), "config")),
        DeclareLaunchArgument("map_source", default_value="", description="online/prior; empty uses navigation_config"),
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
        DeclareLaunchArgument("output_topic", default_value="", description="Empty uses node_params.yaml"),
        DeclareLaunchArgument("normal_mode", default_value="", description="Empty uses region_control.yaml"),
        OpaqueFunction(function=_launch_setup),
    ])
