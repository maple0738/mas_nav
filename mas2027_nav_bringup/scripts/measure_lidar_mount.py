#!/usr/bin/env python3
# Copyright 2026 mas2027
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
量两台 MID360 在云台上的安装参数，把要改的那几行算好打印出来。

base_link 就是云台。两台雷达随云台自旋，各自离云台 yaw 轴有个水平半径。两种测量
互补，都要做：

倾角 / gravity（静止）
    静止时加速度计读的是"向上"的比力，取反就是重力向量，也就是 small_point_lio 的
    gravity。两个雷达关节都只有 Rx(r)，重力在雷达系里只有 [0, -n·sin r, -n·cos r]，
    所以 r = atan2(a_y, a_z)。右雷达 r>0，左雷达 r<0（手性对称）。

旋转半径（自旋）
    底盘刹住、云台匀速转一圈以上。雷达绕 yaw 轴画圆，但 /Odometry 已经被 LIO 用
    URDF 的 lidar_joint 折算到 base_link 了，所以拟合出来的不是绝对半径，而是那个
    平移的**残差**：URDF 填对了圆就收成一个点。按 p = a + Rz(yaw)·v 解最小二乘
    （对 a 和 v 都线性，不用先拟合圆再去 yaw），v 就是残差，加回 URDF 现值即真值。

用法（车停在水平地面、底盘静止）：

    # 一键：起分话题驱动 + 两个 LIO，先静止再自旋，测完收进程
    bash src/mas2027_nav_bringup/scripts/measure_lidar_mount.sh

    # 1. 只算倾角和 gravity。要两个 IMU 话题就得分话题起驱动：
    #    enable_lidar_merge: false + is_topic_name_with_lidar_ip: true
    python3 src/mas2027_nav_bringup/scripts/measure_lidar_mount.py

    # 2. 右雷达 136 的旋转半径：照常融合起 LIO，底盘别动，云台转一圈
    python3 src/mas2027_nav_bringup/scripts/measure_lidar_mount.py --spin

    # 3. 左雷达 193 的旋转半径：LIO 得改订 193 的分话题、lidar_frame 换成
    #    lidar_back_link，再转一圈
    python3 src/mas2027_nav_bringup/scripts/measure_lidar_mount.py --spin --lidar left

    # 不连车，只看某个倾角对应的参数
    python3 src/mas2027_nav_bringup/scripts/measure_lidar_mount.py --tilt 48.07

每次跑都以 URDF 现值为底、叠上这一轮实测，打印完整的一套（两个关节 + 融合矩阵 +
blind_center）。所以顺序是：测完静止改 rpy，测完自旋改 xyz，改完再跑一次核对。

量不出来的：
- 云台/车体自身的俯仰会整个叠加到 r 上，重力分不开这两者。地面不平就先量车体俯仰，
  用 --chassis-pitch 减掉。
- 绕重力的 yaw：重力测不到，自旋也测不到（它和云台转角混在一起），只能靠装配保证。
- 自旋测不到 z：解出来的 v 只有水平两个分量，z 仍然靠尺量或 CAD。
- 想事后再算：ros2 bag record /Odometry，回放时照常跑本脚本，不用另装 rosbags。
"""

import argparse
import math
import os
import re
import time

URDF_REL_PATH = "mas2027_robot_description/urdf/mas2027_sentry.urdf"
# base_link（云台原点）离地高度，见 URDF base_link_to_base_footprint
BASE_HEIGHT = 0.10
# Mid360 垂直视场，厂家标称值，仓库里没有东西能验证它
FOV_MIN_DEG, FOV_MAX_DEG = -7.0, 52.0
# 现在配置里 gravity 的模长（单位 g，IMU 的标度误差就体现在这 0.4% 上）
CURRENT_NORM = 0.995636
# side -> (IP, URDF 关节名, 人看的标签)
LIDARS = {
    "right": ("192.168.1.136", "lidar_joint", "右 136（融合参考系 lidar_link）"),
    "left": ("192.168.1.193", "lidar_back_joint", "左 193（点云乘 T 拼进右雷达系）"),
}
SIDES = ("right", "left")


def rpy_to_matrix(roll, pitch, yaw):
    """Rz(yaw)·Ry(pitch)·Rx(roll)，URDF 和驱动 make_transform 用的就是这个顺序。"""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


def matrix_to_rpy(m):
    """上面那个的逆。pitch 落在 ±90° 内，本车不会碰到万向锁。"""
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    return math.atan2(m[2][1], m[2][2]), pitch, math.atan2(m[1][0], m[0][0])


def transposed_times_vector(m, v):
    """Rᵀ·v。"""
    return tuple(sum(m[k][i] * v[k] for k in range(3)) for i in range(3))


def transposed_times_matrix(a, b):
    """Aᵀ·B。"""
    return tuple(tuple(sum(a[k][i] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def blind_center(joint):
    """base_link 原点在该雷达系中的坐标 = -Rᵀ·t。LIO 的 blind_center 就是这个。"""
    xyz, rpy = joint
    return tuple(-value for value in transposed_times_vector(rpy_to_matrix(*rpy), xyz))


def relative_extrinsic(front, back):
    """T_front⁻¹·T_back，即驱动的 merge_extrinsic_back_to_front，[x,y,z,roll,pitch,yaw]。"""
    rotation_front = rpy_to_matrix(*front[1])
    delta = tuple(back[0][i] - front[0][i] for i in range(3))
    translation = transposed_times_vector(rotation_front, delta)
    rotation = transposed_times_matrix(rotation_front, rpy_to_matrix(*back[1]))
    return translation + matrix_to_rpy(rotation)


def lidar_elevation(rotation, azimuth_deg, rho, dz):
    """从雷达原点看：水平距离 rho、方位角 azimuth_deg、高度差 dz 的点，在雷达系里的仰角（度）。"""
    offset = (rho * math.cos(math.radians(azimuth_deg)), rho * math.sin(math.radians(azimuth_deg)), dz)
    norm = math.sqrt(sum(value * value for value in offset))
    if norm < 1e-9:
        return 0.0
    return math.degrees(math.asin(max(-1.0, min(1.0, transposed_times_vector(rotation, offset)[2] / norm))))


def first_visible(rotation, azimuth_deg, dz, limit=10.0):
    """沿某方位角，高度差 dz 的水平面上第一个落进视场的水平距离；整条线都看不到返回 None。"""
    rho = 0.01
    while rho <= limit:
        if FOV_MIN_DEG <= lidar_elevation(rotation, azimuth_deg, rho, dz) <= FOV_MAX_DEG:
            return rho
        rho += 0.005
    return None


def find_urdf():
    """源码树，以及从 install/lib 回退到仓库根。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = (
        os.path.join(here, "..", "..", URDF_REL_PATH),
        os.path.join("/home/mas/mas_nav_2027_native", URDF_REL_PATH),
        os.path.join(here, "..", "..", "..", URDF_REL_PATH),
    )
    for path in candidates:
        path = os.path.normpath(path)
        if os.path.isfile(path):
            return path
    return os.path.normpath(candidates[0])


def read_urdf_joint(name):
    """读 URDF 某个 fixed 关节的 origin，返回 ((x,y,z), (r,p,y))；读不到返回 None。"""
    path = find_urdf()
    try:
        with open(path, encoding="utf-8") as handle:
            text = handle.read()
    except OSError:
        return None
    match = re.search(
        r'<joint\s+name="%s".*?<origin\s+xyz="([^"]+)"\s+rpy="([^"]+)"' % re.escape(name), text, re.S)
    if not match:
        return None
    xyz = tuple(float(value) for value in match.group(1).split())
    rpy = tuple(float(value) for value in match.group(2).split())
    return (xyz, rpy) if len(xyz) == 3 and len(rpy) == 3 else None


def solve_linear(matrix, rhs):
    """高斯消元解小方程组，奇异返回 None。"""
    size = len(rhs)
    rows = [list(row) + [rhs[index]] for index, row in enumerate(matrix)]
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(rows[row][column]))
        if abs(rows[pivot][column]) < 1e-12:
            return None
        rows[column], rows[pivot] = rows[pivot], rows[column]
        for row in range(size):
            if row != column:
                factor = rows[row][column] / rows[column][column]
                for k in range(column, size + 1):
                    rows[row][k] -= factor * rows[column][k]
    return [rows[index][size] / rows[index][index] for index in range(size)]


def fit_spin(samples):
    """解 p_i = a + Rz(yaw_i)·v。返回 (轴心 a, 残差矢量 v, 单帧 RMSE)；解不出返回 None。

    对 (a_x, a_y, v_x, v_y) 是线性的，所以不用先拟合圆再去 yaw：残差趋近 0 时圆退化成
    一团噪声、圆心无意义，这个式子照样能解出 v。
    """
    design = []
    target = []
    for x, y, yaw in samples:
        c, s = math.cos(yaw), math.sin(yaw)
        design.append((1.0, 0.0, c, -s))
        target.append(x)
        design.append((0.0, 1.0, s, c))
        target.append(y)
    normal = [[sum(row[i] * row[j] for row in design) for j in range(4)] for i in range(4)]
    projected = [sum(design[k][i] * target[k] for k in range(len(design))) for i in range(4)]
    solution = solve_linear(normal, projected)
    if solution is None:
        return None
    ax, ay, vx, vy = solution
    squared = 0.0
    for x, y, yaw in samples:
        c, s = math.cos(yaw), math.sin(yaw)
        squared += (ax + c * vx - s * vy - x) ** 2 + (ay + s * vx + c * vy - y) ** 2
    return (ax, ay), (vx, vy), math.sqrt(squared / len(samples))


def total_rotation(yaws):
    """把 yaw 序列解缠后的累计转角（弧度，带符号）。"""
    total = 0.0
    previous = yaws[0]
    for yaw in yaws[1:]:
        delta = yaw - previous
        while delta > math.pi:
            delta -= 2.0 * math.pi
        while delta < -math.pi:
            delta += 2.0 * math.pi
        total += delta
        previous = yaw
    return total


def collect(imu_topics, odom_topics, args):
    """一个节点两段采集：先静止收 IMU，再提示转云台收里程计。两段都可以为空。"""
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    accel = {key: [] for key in imu_topics}
    odom = {key: [] for key in odom_topics}
    rclpy.init()
    node = rclpy.create_node("measure_lidar_mount")
    for key, topic in imu_topics.items():
        # best_effort 订阅能同时匹配 reliable 和 best_effort 的发布者
        node.create_subscription(
            Imu, topic,
            lambda msg, side=key: accel[side].append((msg.linear_acceleration.x,
                                                      msg.linear_acceleration.y,
                                                      msg.linear_acceleration.z)),
            qos_profile_sensor_data)
    for key, topic in odom_topics.items():
        node.create_subscription(
            Odometry, topic,
            lambda msg, side=key: odom[side].append(odom_sample(msg)),
            qos_profile_sensor_data)

    if imu_topics:
        node.get_logger().info("静止段：等 %s 各 %d 帧，车别动……"
                               % ("、".join(imu_topics.values()), args.samples))
        deadline = time.time() + args.timeout
        # 先等一会儿再判断"够了没"，免得起得慢的那台被当成没数据
        grace = time.time() + 2.0
        while rclpy.ok() and time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            live = [side for side, frames in accel.items() if frames]
            if live and time.time() > grace and all(len(accel[side]) >= args.samples for side in live):
                break
    if odom_topics:
        collect_spin(node, odom, odom_topics, args)
    node.destroy_node()
    rclpy.shutdown()
    return accel, odom


def collect_spin(node, odom, odom_topics, args):
    """自旋段：清掉静止段攒下的里程计，等云台转，边转边报累计角度。"""
    import rclpy

    for samples in odom.values():
        samples.clear()
    node.get_logger().info("自旋段：底盘刹住，现在开始匀速转云台，采 %.0f s（%s）……"
                           % (args.duration, "、".join(odom_topics.values())))
    # 计时从"真的转起来了"开始算，不是从收到第一帧算，免得还没上手就把时间耗掉
    wait_deadline = time.time() + max(args.timeout, 60.0)
    started = None
    reported = 0.0
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        live = [side for side, samples in odom.items() if len(samples) > 1]
        if not live:
            if time.time() > wait_deadline:
                break
            continue
        rotation = max(abs(math.degrees(total_rotation([sample[3] for sample in odom[side]])))
                       for side in live)
        if started is None:
            if rotation > 5.0:
                node.get_logger().info("云台转起来了，开始计时 %.0f s" % args.duration)
                started = time.time()
            elif time.time() > wait_deadline:
                node.get_logger().warn("一直没检测到云台转动，不等了")
                break
            else:
                continue
        if rotation - reported >= 90.0:
            reported = rotation
            node.get_logger().info("已转 %.0f°" % rotation)
        if time.time() - started > args.duration or rotation > 720.0:
            break


def odom_sample(msg):
    """Odometry -> (x, y, z, yaw)。"""
    q = msg.pose.pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, yaw)


def tidy(values):
    """把 -0.0 和打印精度以下的噪声压成 0.0，免得 -0.0000 被贴进配置里。"""
    return tuple(0.0 if abs(value) < 5e-7 else value for value in values)


def report(joints, urdf, gravity_by_side=None):
    """打印两个关节的安装姿态、要改的每一行、以及这套几何的地面覆盖。"""
    gravity_by_side = gravity_by_side or {}

    print("\n安装姿态（半径 = 雷达到云台 yaw 轴的水平距离）")
    for side in SIDES:
        xyz, rpy = joints[side]
        mark = "" if joints[side] == urdf[side] else "   ← 本轮实测，URDF 还是旧值"
        print("  %-26s roll %+7.2f°  xyz %+.4f %+.4f %+.4f  半径 %.4f%s"
              % (LIDARS[side][2], math.degrees(rpy[0]), xyz[0], xyz[1], xyz[2],
                 math.hypot(xyz[0], xyz[1]), mark))
    print("  两雷达间距 %.4f m" % math.dist(joints["right"][0], joints["left"][0]))

    print("\n要改的地方")
    print("  %s" % URDF_REL_PATH)
    for side in SIDES:
        xyz, rpy = joints[side]
        print('    %-17s <origin xyz="%.4f %.4f %.4f" rpy="%.4f %.4f %.4f"/>'
              % ((LIDARS[side][1],) + tidy(xyz) + tidy(rpy)))
    extrinsic = relative_extrinsic(joints["right"], joints["left"])
    print("  mas2027_nav_bringup/config/small_point_lio_params.yaml")
    print("  （2026-09-18 起这是唯一真源：mid360_driver/config/params.yaml 已删除）")
    print("    merge_extrinsic_back_to_front: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]" % tidy(extrinsic))
    print("    # = T_lidar_link⁻¹·T_lidar_back，由上面两个关节算出，不是手推的手性对称")
    print("    # 相对旋转 roll %+.2f°（两侧倾角之差）  pitch %+.2f°  yaw %+.2f°"
          % tidy(tuple(math.degrees(value) for value in extrinsic[3:])))
    bx, by, bz = blind_center(joints["right"])
    print("    blind_center: [ %.4f, %.4f, %.4f ]" % tidy((bx, by, bz)))
    print("    # |blind_center| = %.4f，与倾角无关，恒等于右雷达 |t| = %.4f"
          % (math.hypot(math.hypot(bx, by), bz), math.dist(joints["right"][0], (0.0, 0.0, 0.0))))
    if "right" in gravity_by_side:
        print("    gravity: [ %.6f, %.6f, %.6f ]" % gravity_by_side["right"])
    else:
        print("    gravity: 本轮没测右雷达 IMU，保持原值")
    if "left" in gravity_by_side:
        print("    # 左雷达 gravity [%.6f, %.6f, %.6f] 只用来核对倾角，LIO 不吃它"
              % gravity_by_side["left"])

    print("\n地面覆盖（视场 %+.0f°…%+.0f°）" % (FOV_MIN_DEG, FOV_MAX_DEG))
    for side in SIDES:
        xyz, rpy = joints[side]
        rotation = rpy_to_matrix(*rpy)
        height = BASE_HEIGHT + xyz[2]
        cells = []
        for label, azimuth in (("前", 0.0), ("左", 90.0), ("右", -90.0), ("后", 180.0)):
            ground = first_visible(rotation, azimuth, -height)
            cells.append("%s %s" % (label, "%.2f m 起" % ground if ground else "看不到"))
        print("  %-26s 离地 %.2f m   %s" % (LIDARS[side][2], height, "  ".join(cells)))
    print("  倾角是 Rx（绕车体 X），主要剪侧向视场；两台一左一右合起来才是接近 360°")


def stats(accel):
    """三轴均值和单帧标准差。"""
    count = len(accel)
    mean = tuple(sum(sample[i] for sample in accel) / count for i in range(3))
    std = tuple(math.sqrt(sum((sample[i] - mean[i]) ** 2 for sample in accel) / count) for i in range(3))
    return mean, std


def analyze_static(collected, urdf, args):
    """静止段：每台雷达的倾角和 gravity。返回 ({side: 关节}, {side: gravity})。"""
    per_side = {side: collected[side] for side in SIDES if len(collected[side]) >= 20}
    if not per_side and len(collected.get("merged", [])) >= 20:
        per_side["right"] = collected["merged"]
        print("\n只有 %s 有数据：这是融合模式，驱动只转发右雷达 IMU。" % args.topic)
        print("要一起测左雷达就分话题重起驱动：enable_lidar_merge: false + "
              "is_topic_name_with_lidar_ip: true")
    if not per_side:
        print("\n三条 IMU 话题都没数据，驱动没起或者话题名不对。")
        return None, None

    joints = {}
    gravity_by_side = {}
    for side in SIDES:
        if side not in per_side:
            print("\n%s：没收到 IMU，倾角沿用 URDF 的 %+.2f°"
                  % (LIDARS[side][2], math.degrees(urdf[side][1][0])))
            continue
        mean, std = stats(per_side[side])
        norm = math.sqrt(sum(value * value for value in mean))
        # 静止时加速度计读向上的比力，取反就是重力向量
        gravity_by_side[side] = tuple(-value for value in mean)
        tilt_deg = math.degrees(math.atan2(mean[1], mean[2])) - args.chassis_pitch
        # a_x 只跟车体侧倾有关：a_x(雷达系) 恒等于 a_y(base_link)
        side_deg = math.degrees(math.asin(max(-1.0, min(1.0, mean[0] / norm))))

        print("\n%s —— %d 帧，单位 g" % (LIDARS[side][2], len(per_side[side])))
        print("  加速度均值   x=%+.6f  y=%+.6f  z=%+.6f" % mean)
        print("  单帧标准差   x=%.5f   y=%.5f   z=%.5f%s"
              % (std[0], std[1], std[2],
                 "" if max(std) <= 0.02 else "   ← 偏大，没停稳或有振动，重测"))
        print("  模长         %.6f g%s"
              % (norm, "" if abs(norm - CURRENT_NORM) <= 0.01
                 else "   ← 和配置的 %.4f 差超 1%%，IMU 标度不该变，先查有没有停稳" % CURRENT_NORM))
        print("  车体侧倾残差 %+.2f°%s" % (side_deg, "" if abs(side_deg) < 1.5 else "   ← 车歪了或装配带了 roll"))
        if args.chassis_pitch:
            print("  实测倾角已减掉车体俯仰 %.2f°" % args.chassis_pitch)
        print("  倾角 r       %+.2f°  (%+.4f rad)  URDF 现值 %+.2f°  差 %+.2f°"
              % (tilt_deg, math.radians(tilt_deg), math.degrees(urdf[side][1][0]),
                 tilt_deg - math.degrees(urdf[side][1][0])))
        joints[side] = (urdf[side][0], (math.radians(tilt_deg), 0.0, 0.0))

    return joints, gravity_by_side


def analyze_spin(collected, urdf, args):
    """自旋段：每台雷达的水平安装偏移。返回 {side: 关节}。"""
    joints = {}
    for side in SIDES:
        samples = collected.get(side, [])
        if len(samples) < 50:
            if side in collected:
                print("\n%s：只收到 %d 帧里程计，这一台的 LIO 没起或者没在发，xyz 沿用 URDF"
                      % (LIDARS[side][2], len(samples)))
            continue
        measured = analyze_spin_side(side, samples, urdf, args)
        if measured is not None:
            joints[side] = measured
    return joints


def analyze_spin_side(side, samples, urdf, args):
    """单台雷达的自旋拟合与打印。返回新关节，或 None。"""
    yaws = [sample[3] for sample in samples]
    rotation_deg = abs(math.degrees(total_rotation(yaws)))
    z_span = max(sample[2] for sample in samples) - min(sample[2] for sample in samples)
    fit = fit_spin([(sample[0], sample[1], sample[3]) for sample in samples])
    if fit is None:
        print("\n%s：解不出来，云台大概没真的转，yaw 一直没变" % LIDARS[side][2])
        return None
    center, residual, rmse = fit

    print("\n%s 自旋（%d 帧）" % (LIDARS[side][2], len(samples)))
    print("  累计转角     %.0f°%s" % (rotation_deg, "" if rotation_deg >= 270.0
                                    else "   ← 不足 270°，圆没画够，残差方向不可信，重测"))
    print("  yaw 轴心     (%+.4f, %+.4f) m（odom 里，只是拟合副产物）" % center)
    print("  拟合 RMSE    %.2f mm%s" % (rmse * 1000.0, "" if rmse < 0.01
                                       else "   ← 偏大：底盘动了、云台不匀速、或者 LIO 在漂"))
    print("  z 变化       %.1f mm%s" % (z_span * 1000.0, "" if z_span < 0.02
                                       else "   ← 平台在晃或 LIO z 在漂"))
    print("  残差矢量 v   x=%+.4f  y=%+.4f m   |v| = %.1f mm"
          % (residual[0], residual[1], math.hypot(*residual) * 1000.0))
    print("  # v 是 URDF 那个平移的误差（去 yaw 后的体系常量偏移）。URDF 填对了 v 就趋近 0。")

    assumed = urdf[side][0]
    corrected = (assumed[0] + residual[0], assumed[1] + residual[1], assumed[2])
    print("  URDF 现值    xyz %+.4f %+.4f %+.4f   半径 %.4f"
          % (assumed[0], assumed[1], assumed[2], math.hypot(assumed[0], assumed[1])))
    print("  加上残差     xyz %+.4f %+.4f %+.4f   半径 %.4f"
          % (corrected[0], corrected[1], corrected[2], math.hypot(corrected[0], corrected[1])))
    print("  z 自旋测不到，%.3f 沿用 URDF" % corrected[2])
    return (corrected, urdf[side][1])


def run_measure(args, urdf):
    """按 --spin / --all 决定跑哪几段，最后合成一份完整参数。"""
    want_static = not args.spin
    want_spin = args.spin or args.all
    imu_topics = {}
    if want_static:
        imu_topics = {side: "%s_%s" % (args.topic, LIDARS[side][0].replace(".", "_")) for side in SIDES}
        imu_topics["merged"] = args.topic
    odom_topics = {}
    if want_spin:
        # 只有一台 LIO 在跑时用 --lidar 指名它发的是哪一台，否则两台各订一条
        odom_topics = ({args.lidar: args.odom_topic} if args.lidar
                       else {"right": args.odom_topic, "left": args.odom_topic_left})

    accel, odom = collect(imu_topics, odom_topics, args)

    joints = dict(urdf)
    gravity_by_side = {}
    if want_static:
        measured, gravity_by_side = analyze_static(accel, urdf, args)
        if measured is None:
            return 1
        joints.update(measured)
    if want_spin:
        measured = analyze_spin(odom, urdf, args)
        if not measured and not want_static:
            print("\n两条里程计话题都没数据，LIO 没起或者话题名不对。")
            return 1
        # analyze_spin_side returns the corrected xyz but deliberately keeps the
        # URDF rpy. In --all mode do not let that overwrite the static measured
        # roll/pitch; only xyz belongs to the spin stage.
        for side, (xyz, _unused_rpy) in measured.items():
            joints[side] = (xyz, joints[side][1])

    report(joints, urdf, gravity_by_side)
    if not want_spin:
        print("\n自旋半径这轮没测，xyz 沿用 URDF。一次测全：--all（要两个 LIO 各吃一台雷达）")
    elif not want_static:
        print("\n倾角这轮没测，rpy 沿用 URDF。一次测全：--all")
    return 0


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/mid360_driver/imu", help="IMU 话题；分话题模式自动补 IP 后缀")
    parser.add_argument("--odom-topic", default="/Odometry", help="--spin 用的里程计话题")
    parser.add_argument("--samples", type=int, default=400, help="静止采样帧数，200 Hz 下 400 帧约 2 秒")
    parser.add_argument("--timeout", type=float, default=30.0, help="等不到数据就放弃的秒数")
    parser.add_argument("--odom-topic-left", default="/Odometry_left",
                        help="左雷达那个 LIO 的里程计话题（两个 LIO 同时跑时用）")
    parser.add_argument("--spin", action="store_true", help="只测旋转半径（底盘静止、云台转圈）")
    parser.add_argument("--all", action="store_true", help="一次跑完：先静止测倾角，再自旋测半径")
    parser.add_argument("--lidar", choices=SIDES,
                        help="只有一台 LIO 在跑时，指名 --odom-topic 是哪一台雷达的")
    parser.add_argument("--duration", type=float, default=40.0, help="--spin 采集时长（秒）")
    parser.add_argument("--tilt", type=float, help="跳过测量，直接算这个倾角（度）对应的参数")
    parser.add_argument("--print-blind", choices=SIDES,
                        help="只打印指定雷达当前 URDF 对应的 blind_center，供一键脚本启动左侧 LIO 使用")
    parser.add_argument("--chassis-pitch", type=float, default=0.0,
                        help="车体自身的俯仰（度），车头朝上为正，会从实测倾角里减掉")
    args = parser.parse_args()

    urdf = {side: read_urdf_joint(LIDARS[side][1]) for side in SIDES}
    missing = [LIDARS[side][1] for side in SIDES if urdf[side] is None]
    if missing:
        print("URDF 里没读到关节 %s，确认 %s" % ("、".join(missing), URDF_REL_PATH))
        return 1

    if args.print_blind is not None:
        bx, by, bz = blind_center(urdf[args.print_blind])
        print("[%.6f,%.6f,%.6f]" % (bx, by, bz))
        return 0

    if args.tilt is not None:
        tilt = math.radians(args.tilt)
        joints = {
            "right": (urdf["right"][0], (tilt, 0.0, 0.0)),
            "left": (urdf["left"][0], (-tilt, 0.0, 0.0)),
        }
        report(joints, urdf,
               {"right": (0.0, -CURRENT_NORM * math.sin(tilt), -CURRENT_NORM * math.cos(tilt))})
        print("\n（--tilt 模式：假设两侧严格手性对称，gravity 取理想值、模长沿用配置里的 %.4f g。"
              "实车仍要静止实测，把装配残差量进去。）" % CURRENT_NORM)
        return 0

    return run_measure(args, urdf)


if __name__ == "__main__":
    raise SystemExit(main())
