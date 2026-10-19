#!/usr/bin/env bash
# 一键测两台 MID360 的安装参数。
#
# 分话题起驱动（两台雷达各自一条话题）+ robot_state_publisher + 两个 small_point_lio
# （一个吃 136 一个吃 193），然后跑 measure_lidar_mount.py --all：先静止测倾角和
# gravity，再让云台自旋测两台的水平安装偏移。测完把起的进程全收掉。
#
# 用法（车停在水平地面。看到「开始匀速转云台」再动手：底盘刹住，只转云台，匀速 1-2 圈）：
#     bash mas2027_nav_bringup/scripts/measure_lidar_mount.sh
#
# 测安装要独占雷达 UDP，导航栈必须先停。只按记下来的 PID 收，绝不 pkill -f。

set -u

HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/.." && pwd)
WS=$(cd "$SRC/../.." && pwd)
MEASURE_PY=$HERE/measure_lidar_mount.py
[ -f "$MEASURE_PY" ] || MEASURE_PY=$SRC/scripts/measure_lidar_mount.py
PARAMS=$SRC/config/small_point_lio_params.yaml
LEFT_PARAMS=/tmp/measure_lidar_mount_left.yaml
LOG_DIR=${LOG_DIR:-/tmp/measure_lidar_mount}
PIDS=()

cleanup() {
    local pid
    # Each helper is started with setsid, so $pid is also the process-group id.
    # Kill the whole group: `ros2 run`/`ros2 launch` can leave the actual node as
    # an orphan if only the wrapper pid is killed.
    for pid in "${PIDS[@]:-}"; do
        [ -n "$pid" ] && kill -TERM -"$pid" 2>/dev/null
    done
    sleep 2
    for pid in "${PIDS[@]:-}"; do
        [ -n "$pid" ] && kill -KILL -"$pid" 2>/dev/null
    done
}
trap cleanup EXIT INT TERM

start() {
    local name=$1
    shift
    setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
    PIDS+=("$!")
    echo "  起 $name (pid $!)  日志 $LOG_DIR/$name.log"
}

# 不强制 QoS：ros2 topic echo 会按发布者匹配。强制 best_effort 反而订不上
# 驱动默认的 reliable IMU/点云。
wait_topic() {
    timeout "${2:-25}" ros2 topic echo --once "$1" >/dev/null 2>&1
}

die() {
    echo
    echo "失败：$*"
    echo "日志在 $LOG_DIR/"
    exit 1
}

command -v setsid >/dev/null 2>&1 || die "缺少 setsid（util-linux），无法可靠清理测量子进程"
mkdir -p "$LOG_DIR"
# ROS 的 setup.bash 会读未定义变量，source 期间关掉 set -u
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
if [ ! -f "$WS/install/setup.bash" ]; then
    echo "失败：找不到 $WS/install/setup.bash。先在仓库根目录构建并 source install/setup.bash"
    exit 1
fi
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

ros2 pkg prefix mid360_driver >/dev/null 2>&1 \
    || die "没 source 到 mid360_driver。在仓库根目录先：
       source /opt/ros/jazzy/setup.bash
       colcon build --packages-select mid360_driver small_point_lio mas2027_nav_bringup mas2027_robot_description --symlink-install
       source install/setup.bash"
ros2 pkg prefix small_point_lio >/dev/null 2>&1 \
    || die "没 source 到 small_point_lio，先把 LIO 编进 install"

if ros2 node list 2>/dev/null | grep -Eq '(mid360_driver|small_point_lio)'; then
    die "已经有驱动或 LIO 在跑。测安装要独占雷达 UDP 56301/56401，先停导航再跑本脚本"
fi

[ -f "$MEASURE_PY" ] || die "找不到 $MEASURE_PY"
[ -f "$PARAMS" ] || die "找不到 $PARAMS"

echo "== 0. 雷达能不能通 =="
for ip in 192.168.1.136 192.168.1.193; do
    ping -c1 -W1 "$ip" >/dev/null 2>&1 || die "$ip ping 不通，雷达没上电或者网线/交换机断了"
    echo "  $ip 通"
done

echo
echo "== 1. 起驱动（分话题，两台各一条） =="
start driver ros2 run mid360_driver mid360_driver_node --ros-args \
    -r __node:=mid360_driver --params-file "$PARAMS" \
    -p enable_lidar_merge:=false -p is_topic_name_with_lidar_ip:=true

for ip_topic in 136 193; do
    echo "  等 /mid360_driver/imu_192_168_1_$ip_topic ……"
    wait_topic "/mid360_driver/imu_192_168_1_$ip_topic" 30 \
        || die "192.168.1.$ip_topic 没在推流。ping 通但没数据 = 雷达没在发点云/IMU，
       去 Livox Viewer 确认两台的 Host IP 都是 192.168.1.50、点云和 IMU 推送都开着"
done
for ip_topic in 136 193; do
    wait_topic "/mid360_driver/lidar_192_168_1_$ip_topic" 20 \
        || die "192.168.1.$ip_topic 有 IMU 但没点云"
    echo "  192.168.1.$ip_topic 点云 + IMU 都在"
done

echo
echo "== 2. 起 TF 和两个 LIO =="
start rsp ros2 launch mas2027_nav_bringup robot_state_publisher_launch.py
sleep 3

LEFT_BLIND_CENTER=$(python3 "$MEASURE_PY" --print-blind left) \
    || die "无法从 URDF 计算左侧雷达 blind_center"
echo "  左侧 LIO blind_center=$LEFT_BLIND_CENTER"

start lio_right ros2 run small_point_lio small_point_lio_node --ros-args \
    -r __node:=small_point_lio --params-file "$PARAMS" \
    -p lidar_topic:=/mid360_driver/lidar_192_168_1_136 \
    -p imu_topic:=/mid360_driver/imu_192_168_1_136

# 换个节点名，免得两个实例同名；blind_center 从 lidar_back_joint 的当前 URDF 计算。
# 不要 remap /tf：左 LIO 还得从 robot_state_publisher 的 /tf lookup lidar_back_link。
# odom_frame 改成 odom_left，避免和右 LIO 抢 odom→base_link（测参只看 /Odometry* 位姿）。
sed 's/^small_point_lio:/small_point_lio_left:/' "$PARAMS" >"$LEFT_PARAMS"
start lio_left ros2 run small_point_lio small_point_lio_node --ros-args \
    -r __node:=small_point_lio_left --params-file "$LEFT_PARAMS" \
    -p lidar_topic:=/mid360_driver/lidar_192_168_1_193 \
    -p imu_topic:=/mid360_driver/imu_192_168_1_193 \
    -p lidar_frame:=lidar_back_link \
    -p odom_frame:=odom_left \
    -p "blind_center:=$LEFT_BLIND_CENTER" \
    -r /Odometry:=/Odometry_left \
    -r /cloud_registered:=/cloud_registered_left \
    -r /cloud_registered_full:=/cloud_registered_full_left

echo "  LIO 开头要静止估重力方向，车别动 ……"
wait_topic /Odometry 60 || die "右雷达的 LIO 没发 /Odometry，看 $LOG_DIR/lio_right.log"
echo "  /Odometry 有了"
wait_topic /Odometry_left 60 || die "左雷达的 LIO 没发 /Odometry_left，看 $LOG_DIR/lio_left.log"
echo "  /Odometry_left 有了"

echo
echo "== 3. 开测 =="
echo "   静止段：车别动。之后脚本会提示转云台，那时底盘刹住、只转云台，匀速 1-2 圈。"
echo
python3 "$MEASURE_PY" --all --timeout 120 --duration 60 \
    2>&1 | tee "$LOG_DIR/measure.log"
status=${PIPESTATUS[0]}

echo
echo "结果也存在 $LOG_DIR/measure.log"
exit "$status"
