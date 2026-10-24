#!/usr/bin/env python3
"""示教点表：把车摆到目标位置，按回车记录当前位姿。

用法：
    ros2 run mas2027_decision teach_points.py --file config/points.yaml
    # 或在源码目录直接运行
    python3 tools/teach_points.py --file mas2027_decision/config/points.yaml

每输入一个点名（如 home），脚本会读取 TF 中 parent -> child 的位姿并写回 YAML。
已有文件会被读入并保留未修改的键。
"""
import argparse
import math
import os
import sys
import threading

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TeachNode(Node):
    def __init__(self, parent_frame: str, child_frame: str):
        super().__init__("teach_points")
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def lookup(self):
        tf = self.buffer.lookup_transform(
            self.parent_frame, self.child_frame, rclpy.time.Time())
        t = tf.transform.translation
        yaw = yaw_from_quaternion(tf.transform.rotation)
        return [round(t.x, 3), round(t.y, 3), round(yaw, 3)]


def load_points(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise SystemExit(f"{path} 不是 YAML 映射")
    return data


def save_points(path: str, points: dict) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(points, f, sort_keys=False,
                       allow_unicode=True, default_flow_style=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", default="points.yaml", help="点表输出路径")
    parser.add_argument("--parent", default="map", help="参考系（map 或 odom）")
    parser.add_argument("--child", default="base_link")
    args = parser.parse_args()

    points = load_points(args.file)

    rclpy.init()
    node = TeachNode(args.parent, args.child)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    print(f"参考系 {args.parent} -> {args.child}，输出 {args.file}")
    print("输入点名后回车记录位姿；输入 l 列出已有点；输入 q 退出。")
    try:
        while True:
            name = input("点名> ").strip()
            if not name:
                continue
            if name == "q":
                break
            if name == "l":
                for k, v in points.items():
                    print(f"  {k}: {v}")
                continue
            try:
                pose = node.lookup()
            except Exception as exc:  # noqa: BLE001 - 示教工具，直接显示原因
                print(f"  取 TF 失败: {exc}")
                continue
            points[name] = pose
            save_points(args.file, points)
            print(f"  {name}: x={pose[0]} y={pose[1]} yaw={pose[2]}  (已保存)")
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
