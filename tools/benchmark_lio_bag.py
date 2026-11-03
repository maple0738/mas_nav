#!/usr/bin/env python3
"""Replay a Livox CustomMsg + SI-IMU SQLite bag into always-adaptive Small Point-LIO.

No Livox SDK/messages are required: decode the checked-in CustomMsg CDR layout,
apply the same strict tag mask to both configurations and publish common PointCloud2.
Recording files are opened read-only. Only isolated frontend processes are started.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import time

import numpy as np
import psutil
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import yaml

ROOT = Path(__file__).resolve().parents[1]
POINT = np.dtype({'names': ['offset', 'x', 'y', 'z', 'tag'],
                  'formats': ['<u4', '<f4', '<f4', '<f4', 'u1'],
                  'offsets': [0, 4, 8, 12, 17], 'itemsize': 20})
OUTPUT = np.dtype({'names': ['x', 'y', 'z', 'timestamp'],
                   'formats': ['<f4', '<f4', '<f4', '<f8'],
                   'offsets': [0, 4, 8, 16], 'itemsize': 24})


def decode_livox(data):
    if data[:4] != b'\0\1\0\0':
        raise ValueError('expected little-endian CDR1 encapsulation')
    sec, nsec, length = struct.unpack_from('<iII', data, 4)
    if length < 1 or 16 + length > len(data) or data[15 + length] != 0:
        raise ValueError('invalid Header frame_id')
    # CDR alignment is relative to the payload after its 4-byte encapsulation.
    cursor = 4 + ((12 + length + 7) // 8) * 8
    timebase, declared = struct.unpack_from('<QI', data, cursor)
    count = struct.unpack_from('<I', data, cursor + 16)[0]
    if declared != count:
        raise ValueError('CustomMsg point_num differs from sequence length')
    start = cursor + 20
    remaining = len(data) - start
    if count > 1000000 or remaining not in (count * 20, count * 20 - 1):
        raise ValueError('invalid CustomPoint array length')
    if remaining == count * 20 - 1:
        data += b'\0'
    points = np.frombuffer(data, dtype=POINT, offset=start, count=count)
    valid = (points['tag'] & 0x3f) == 0
    result = np.empty(int(valid.sum()), dtype=OUTPUT)
    for field in ('x', 'y', 'z'):
        result[field] = points[field][valid]
    result['timestamp'] = (timebase + points['offset'][valid].astype(np.uint64)).astype(np.float64) * 1e-9
    cloud = PointCloud2()
    cloud.header.frame_id = 'lidar_link'
    cloud.header.stamp.sec, cloud.header.stamp.nanosec = sec, nsec
    cloud.height, cloud.width = 1, len(result)
    cloud.point_step, cloud.row_step = 24, 24 * len(result)
    cloud.fields = [PointField(name=name, offset=offset, datatype=datatype, count=1)
                    for name, offset, datatype in [('x', 0, 7), ('y', 4, 7), ('z', 8, 7), ('timestamp', 16, 8)]]
    cloud.data = result.tobytes()
    cloud.is_dense = False
    return cloud, count, len(result), timebase * 1e-9, (int(points['offset'].max()) if count else 0) * 1e-9


def connection(path):
    return sqlite3.connect('file:' + str(path.resolve()) + '?mode=ro', uri=True)


def small_parameters():
    params = yaml.safe_load((ROOT / 'mas2027_nav_bringup/config/small_point_lio_params.yaml').read_text())['small_point_lio']['ros__parameters']
    params.update(lidar_topic='/bench/lidar', imu_topic='/bench/imu', lidar_type='custom_mid360_driver',
                  gravity=[0., 0., -9.81], acc_norm=9.81, blind_center=[0., 0., 0.],
                  min_distance=.38, max_distance=100., save_pcd=False, publish_odom_tf=False,
                  publish_processing_stats=True, reliable_input=True, imu_queue_depth=4000, lidar_queue_depth=200)
    # This bag's raw acceleration is SI, while the production threshold is in g.
    params['satu_acc'] *= 9.81
    return params


def run(args, mode, index, database, topics, first_record, last_record):
    out = args.output / f'{index}-{mode}'
    out.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(args.domain + index), ROS_LOG_DIR=str(out / 'ros'),
               OMP_NUM_THREADS='2', OMP_WAIT_POLICY='PASSIVE')
    params = small_parameters()
    small_path = out / 'small.yaml'
    small_path.write_text(yaml.safe_dump({'small_point_lio': {'ros__parameters': params}}))
    commands = [[str(args.executable or ROOT / 'install/small_point_lio/lib/small_point_lio/small_point_lio_node'),
                 '--ros-args', '--params-file', str(small_path)]]
    stats_topic = '/small_point_lio/processing_stats'
    if args.library_path:
        env['LD_LIBRARY_PATH'] = str(args.library_path.resolve()) + ':' + env.get('LD_LIBRARY_PATH', '')
    logs = [(out / f'node-{i}.log').open('w') for i in range(len(commands))]
    processes = [subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True) for command, log in zip(commands, logs)]
    context = Context()
    rclpy.init(context=context, domain_id=args.domain + index)
    node = Node('bag_benchmark_driver', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    tf = StaticTransformBroadcaster(node)
    transform = TransformStamped()
    transform.header.frame_id, transform.child_frame_id = 'base_link', 'lidar_link'
    transform.transform.rotation.w = 1.
    tf.sendTransform(transform)
    lidar = node.create_publisher(PointCloud2, '/bench/lidar', QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))
    imu = node.create_publisher(Imu, '/bench/imu', QoSProfile(depth=4000, reliability=ReliabilityPolicy.RELIABLE))
    stats = {'published_scans': 0, 'published_imus': 0, 'raw_points': 0, 'published_points': 0,
             'odometry': 0, 'clouds': 0, 'cloud_points': 0, 'matches': 0,
             'finite': True, 'monotonic_odometry': True, 'session_failed': False, 'processing': {}, 'max_speed_mps': 0.,
             'max_position_m': 0., 'first_output_elapsed_s': None}
    latencies = []
    trajectory = []
    last_output_stamp = None
    started = None

    def on_odom(message):
        nonlocal last_output_stamp
        stats['odometry'] += 1
        p, v, q = message.pose.pose.position, message.twist.twist.linear, message.pose.pose.orientation
        stats['finite'] &= all(math.isfinite(x) for x in [p.x, p.y, p.z, v.x, v.y, v.z, q.x, q.y, q.z, q.w])
        stats['max_speed_mps'] = max(stats['max_speed_mps'], math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z))
        stats['max_position_m'] = max(stats['max_position_m'], math.sqrt(p.x*p.x + p.y*p.y + p.z*p.z))
        output_stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        stats['monotonic_odometry'] &= last_output_stamp is None or output_stamp > last_output_stamp
        last_output_stamp = output_stamp
        trajectory.append([last_output_stamp, p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z])
        if started is not None:
            if stats['first_output_elapsed_s'] is None:
                stats['first_output_elapsed_s'] = time.monotonic() - started
            elapsed_sensor = last_output_stamp - first_record * 1e-9
            if elapsed_sensor >= args.warmup:
                latencies.append(1000 * (time.monotonic() - started - elapsed_sensor / args.rate))

    def on_cloud(message):
        stats['clouds'] += 1
        stats['cloud_points'] += message.width * message.height

    node.create_subscription(Odometry, '/Odometry', on_odom, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/cloud_registered_full', on_cloud, qos_profile_sensor_data)
    node.create_subscription(String, stats_topic, lambda message: stats.update(processing=json.loads(message.data)), 10)
    samples = []

    def resources():
        process_stats = [psutil.Process(p.pid) for p in processes]
        return sum(sum(p.cpu_times()[:2]) for p in process_stats), sum(p.memory_info().rss for p in process_stats)

    try:
        deadline = time.monotonic() + 10
        while (not lidar.get_subscription_count() or not imu.get_subscription_count()) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.02)
        if not lidar.get_subscription_count() or not imu.get_subscription_count():
            raise RuntimeError('frontend input subscriptions not ready')
        # Graph discovery can precede usable best-effort data paths by a few packets.
        settle = time.monotonic() + 1.
        while time.monotonic() < settle:
            executor.spin_once(timeout_sec=.02)
        db = connection(database)
        limit = min(last_record, first_record + int(args.duration * 1e9)) if args.duration else last_record
        messages = db.execute('select topic_id,timestamp,data from messages where timestamp<=? order by timestamp,id', (limit,))
        started = time.monotonic()
        cpu_begin, _ = resources()
        cpu_steady = wall_steady = odom_steady = cloud_steady = None
        peak_rss = 0
        next_sample, next_progress = 0., 0.
        for topic, recorded, data in messages:
            if topic not in topics:
                continue
            bag_elapsed = (recorded - first_record) * 1e-9
            target = started + bag_elapsed / args.rate
            while time.monotonic() < target:
                executor.spin_once(timeout_sec=min(.002, max(0, target-time.monotonic())))
            if topics[topic] == 'imu':
                message = deserialize_message(data, Imu)
                imu.publish(message)
                stats['published_imus'] += 1
            else:
                message, raw, valid, _, _ = decode_livox(data)
                lidar.publish(message)
                stats['published_scans'] += 1
                stats['raw_points'] += raw
                stats['published_points'] += valid
            executor.spin_once(timeout_sec=0)
            if bag_elapsed >= next_sample:
                used, rss = resources()
                peak_rss = max(peak_rss, rss)
                samples.append({'bag_seconds': bag_elapsed, 'wall_seconds': time.monotonic()-started,
                                'cpu_seconds': used-cpu_begin, 'rss_mib': rss/2**20,
                                'odometry': stats['odometry'], 'clouds': stats['clouds']})
                next_sample = bag_elapsed + 1
                if cpu_steady is None and bag_elapsed >= args.warmup:
                    cpu_steady, wall_steady = used, time.monotonic()
                    odom_steady, cloud_steady = stats['odometry'], stats['clouds']
            if bag_elapsed >= next_progress:
                print(json.dumps({'mode': mode, 'run': index, 'bag_seconds': round(bag_elapsed, 1),
                                  'outputs': stats['odometry'], 'accepted_points': stats['processing'].get('accepted_points', 0),
                                  'processing': stats['processing'], 'session_failed': stats['session_failed']}), flush=True)
                next_progress = bag_elapsed + 30
            if any(p.poll() is not None for p in processes) or stats['session_failed']:
                raise RuntimeError('frontend exited or session failed')
        cpu_end, rss = resources()
        wall_end = time.monotonic()
        # Let queued outputs and the 1 Hz delivery counters settle; exclude drain from CPU rate.
        drain = time.monotonic() + 3
        while time.monotonic() < drain:
            executor.spin_once(timeout_sec=.01)
        delivered = stats['processing']
        delivery_equal = all(delivered.get(key) == stats[published] for key, published in
                             [('received_scans', 'published_scans'), ('received_imus', 'published_imus'),
                              ('received_points', 'published_points')])
        matched = delivered.get('accepted_points', 0)
        stats['matches'] = matched
        lag = limit * 1e-9 - last_output_stamp if last_output_stamp else None
        healthy = bool(delivery_equal and stats['finite'] and stats['monotonic_odometry'] and not stats['session_failed'] and
                       matched > 100 and stats['clouds'] > 10 and lag is not None and lag < .5)
        elapsed = wall_end - wall_steady if wall_steady else None
        used = cpu_end - cpu_steady if cpu_steady is not None else None
        result = {'mode': mode, 'index': index, 'rate': args.rate, 'bag_seconds': (limit-first_record)*1e-9,
                  'warmup_bag_seconds': args.warmup, 'steady_wall_seconds': elapsed,
                  'steady_cpu_seconds': used, 'total_cpu_seconds': cpu_end-cpu_begin,
                  'cpu_percent_one_core': 100*used/elapsed if elapsed else None,
                  'cpu_seconds_per_bag_second': (cpu_end-cpu_begin)/((limit-first_record)*1e-9),
                  'peak_combined_rss_mib': max(peak_rss, rss)/2**20,
                  'steady_odom_hz_wall': (stats['odometry']-odom_steady)/elapsed if elapsed else None,
                  'steady_cloud_hz_wall': (stats['clouds']-cloud_steady)/elapsed if elapsed else None,
                  'output_lag_bag_seconds': lag, 'delivery_equal': delivery_equal, 'healthy': healthy,
                  'odom_replay_age_p95_ms': float(np.percentile(latencies, 95)) if latencies else None, **stats}
        (out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        (out/'trajectory.json').write_text(json.dumps(trajectory)+'\n')
        (out/'samples.json').write_text(json.dumps(samples, indent=2)+'\n')
        print(json.dumps(result), flush=True)
        if not healthy:
            raise RuntimeError('benchmark health gate failed; result is not a valid CPU comparison')
    except Exception as error:
        (out/'failure.json').write_text(json.dumps({'error': str(error), **stats}, indent=2)+'\n')
        raise
    finally:
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        for log in logs:
            log.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, help='independent reference executable for binary A/B')
    parser.add_argument('--library-path', type=Path, help='library directory matching the reference executable')
    parser.add_argument('--label', default='adaptive', help='result label, not an algorithm switch')
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rate', type=float, default=2.)
    parser.add_argument('--duration', type=float, default=0., help='recorded seconds, zero for full bag')
    parser.add_argument('--warmup', type=float, default=20.)
    parser.add_argument('--domain', type=int, default=210)
    args = parser.parse_args()
    if not math.isfinite(args.rate) or args.rate <= 0 or args.warmup < 0 or args.duration < 0:
        parser.error('invalid replay rate or durations')
    if not 0 <= args.domain <= 232:
        parser.error('ROS domain must be between 0 and 232')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=True)
    databases = sorted(args.bag.glob('*.db3'))
    if len(databases) != 1:
        parser.error('this benchmark requires exactly one SQLite db3')
    db = connection(databases[0])
    topics = {}
    description = list(db.execute('select id,name,type from topics'))
    for tid, name, kind in description:
        if name == '/livox/lidar' and kind == 'livox_ros_driver2/msg/CustomMsg': topics[tid] = 'lidar'
        if name == '/livox/imu' and kind == 'sensor_msgs/msg/Imu': topics[tid] = 'imu'
    if set(topics.values()) != {'imu', 'lidar'}:
        parser.error('required Livox lidar and IMU topics not found')
    first, last = db.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    (args.output/'input.json').write_text(json.dumps({'database': str(databases[0].resolve()),
        'size_bytes': databases[0].stat().st_size, 'topics': description, 'first_record_ns': first,
        'last_record_ns': last, 'duration_seconds': (last-first)*1e-9, 'rate': args.rate,
        'tag_mask': '(tag & 0x3f) == 0', 'imu_scale': 1., 'adaptation': 'spatial and temporal are built in; no switches', 'mount_assumption':
        'identity base_link<-lidar_link for benchmark; factory lidar<-IMU translation retained'}, indent=2)+'\n')
    run(args, args.label, 0, databases[0], topics, first, last)


if __name__ == '__main__':
    main()
