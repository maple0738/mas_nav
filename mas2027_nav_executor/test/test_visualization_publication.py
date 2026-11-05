"""Exercise the actual executor with isolated ROS topics and no hardware nodes."""

import os
from pathlib import Path
import random
import signal
import subprocess
import time
import uuid

import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from interfaces.msg import MpcPositionCommand, PositionCommand
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker


def retained_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ExecutorHarness:
    def __init__(self, temporary):
        binary = Path(os.environ['NAV_EXECUTOR_BINARY'])
        config = Path(os.environ['NAV_EXECUTOR_CONFIG_DIR'])
        self.prefix = '/visualization_test_' + uuid.uuid4().hex
        domain = random.randint(150, 200)
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=domain)
        self.node = rclpy.create_node('visualization_reader', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.log_path = temporary / 'executor.log'
        self.log = self.log_path.open('w')
        command = [str(binary), '--ros-args']
        for name in ('node_params', 'planner_params', 'perception', 'terrain_recognition',
                     'region_control', 'mpc_params'):
            command += ['--params-file', str(config / (name + '.yaml'))]
        topics = {
            'goal_sub': 'goal', 'odom_sub': 'odom', 'trajectory_sub': 'trajectory',
            'spin_cmd_sub': 'spin', 'cmd_vel_pub': 'velocity', 'chassis_cmd_pub': 'chassis',
            'region_status_pub': 'region_status', 'minco_path_pub': 'path',
            'minco_trajectory_pub': 'trajectory_marker', 'global_plan_pub': 'global_path',
            'global_plan_marker_pub': 'global_marker', 'planning_constraints_pub': 'constraints',
            'planning_constraints_marker_pub': 'constraints_marker',
            'terrain_cost_sub': 'cost', 'terrain_label_sub': 'labels',
            'terrain_uphill_sub': 'directions', 'online_terrain_label_sub': 'online_labels',
        }
        overrides = {
            'node.map_source': 'prior',
            'planner.rog_map.ros_callback.enable': 'false',
            'planner.rog_map.visualization.enable': 'false',
            'planner.rog_map.performance.enable': 'false',
            'planner.rog_map.performance.summary_csv_enable': 'false',
            'planner.rog_map.performance.detailed_csv_enable': 'false',
            'planner.performance.detailed_csv_enable': 'false',
        }
        overrides.update({'node.topics.' + key: self.topic(value)
                          for key, value in topics.items()})
        for key, value in overrides.items():
            command += ['-p', key + ':=' + value]
        environment = dict(os.environ, ROS_DOMAIN_ID=str(domain), OMP_NUM_THREADS='2')
        self.process = subprocess.Popen(command, env=environment,
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.publishers = {
            'cost': self.node.create_publisher(OccupancyGrid, self.topic('cost'), retained_qos()),
            'labels': self.node.create_publisher(Image, self.topic('labels'), retained_qos()),
            'directions': self.node.create_publisher(Image, self.topic('directions'), retained_qos()),
            'trajectory': self.node.create_publisher(MpcPositionCommand,
                                                    self.topic('trajectory'), 1),
        }

    def topic(self, suffix):
        return self.prefix + '/' + suffix

    def spin_until(self, condition, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.log_path.read_text()
            if condition():
                return
            self.executor.spin_once(timeout_sec=0.02)
        assert condition(), self.log_path.read_text()

    def settle(self, seconds=0.5):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.log_path.read_text()
            self.executor.spin_once(timeout_sec=0.02)

    def publish_maps(self, labels_last=False):
        cost = OccupancyGrid()
        cost.header.frame_id = 'odom'
        cost.info.width = cost.info.height = 20
        cost.info.resolution = 0.1
        cost.info.origin.orientation.w = 1.0
        cost.data = [0] * 400
        cost.data[10] = 100
        labels = Image()
        labels.header.frame_id = 'odom'
        labels.width = labels.height = 20
        labels.encoding = 'mono8'
        labels.step = 20
        labels.data = bytes(400)
        directions = Image()
        directions.header.frame_id = 'odom'
        directions.width = directions.height = 20
        directions.encoding = '32FC2'
        directions.step = 20 * 2 * 4
        directions.data = bytes(20 * directions.step)
        messages = {'cost': cost, 'labels': labels, 'directions': directions}
        order = ('cost', 'directions', 'labels') if labels_last else ('cost', 'labels', 'directions')
        for name in order:
            self.spin_until(lambda: self.publishers[name].get_subscription_count() > 0)
            self.publishers[name].publish(messages[name])
            self.settle(0.15)

    def publish_trajectory(self, trajectory_id):
        message = MpcPositionCommand()
        message.header.frame_id = 'odom'
        message.header.stamp = self.node.get_clock().now().to_msg()
        for i in range(3):
            point = PositionCommand()
            point.position.x = 0.2 + i * 0.2
            point.position.y = 0.5 + 0.1 * trajectory_id
            point.vel_norm = 1.0
            point.trajectory_id = trajectory_id
            message.cmds.append(point)
        self.spin_until(lambda: self.publishers['trajectory'].get_subscription_count() > 0)
        self.publishers['trajectory'].publish(message)
        self.settle()

    def close(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()
        self.log.close()


@pytest.fixture
def executor(tmp_path):
    harness = ExecutorHarness(tmp_path)
    try:
        yield harness
    finally:
        harness.close()


def test_constraints_publish_when_labels_arrive_last(executor):
    grids = []
    grid_sub = executor.node.create_subscription(
        OccupancyGrid, executor.topic('constraints'), grids.append, 1)
    executor.publish_maps(labels_last=True)
    executor.spin_until(lambda: bool(grids))
    assert grids[-1].data[10] == 100

    # No Marker subscriber existed during the map callbacks. A late volatile
    # reader must receive a newly constructed current Marker from the timer.
    markers = []
    marker_sub = executor.node.create_subscription(
        Marker, executor.topic('constraints_marker'), markers.append, 1)
    executor.spin_until(lambda: bool(markers))
    assert markers[-1].ns == 'planning_constraints'
    assert len(markers[-1].points) == 1
    executor.settle()
    assert len(markers) == 1

    # A later durable grid reader also receives the retained static raster.
    late_grids = []
    late_sub = executor.node.create_subscription(
        OccupancyGrid, executor.topic('constraints'), late_grids.append, retained_qos())
    executor.spin_until(lambda: bool(late_grids))
    assert late_grids[-1].data == grids[-1].data
    assert grid_sub and marker_sub and late_sub


def test_trajectory_path_and_marker_are_independent_and_late_subscriptions_catch_up(executor):
    executor.publish_maps()
    executor.publish_trajectory(1)
    paths, markers = [], []
    path_sub = executor.node.create_subscription(RosPath, executor.topic('path'), paths.append, 1)
    executor.spin_until(lambda: bool(paths))
    assert len(paths[-1].poses) == 3
    assert paths[-1].header.frame_id == 'odom'
    assert paths[-1].poses[0].pose.position.y == pytest.approx(0.6)
    executor.settle()
    assert len(paths) == 1

    # Adding the other output later must not depend on a new trajectory.
    marker_sub = executor.node.create_subscription(
        Marker, executor.topic('trajectory_marker'), markers.append, 1)
    executor.spin_until(lambda: bool(markers))
    assert len(markers[-1].points) == len(markers[-1].colors) == 3
    assert markers[-1].type == Marker.LINE_STRIP
    executor.settle()
    assert len(paths) == len(markers) == 1

    executor.publish_trajectory(2)
    executor.spin_until(lambda: len(paths) == len(markers) == 2)
    assert paths[-1].header.stamp != paths[0].header.stamp
    assert markers[-1].header.stamp == paths[-1].header.stamp
    assert all(pose.pose.position.y == pytest.approx(0.7) for pose in paths[-1].poses)
    assert all(point.y == pytest.approx(0.7) for point in markers[-1].points)
    executor.settle()
    assert len(paths) == len(markers) == 2

    executor.node.destroy_subscription(path_sub)
    executor.node.destroy_subscription(marker_sub)
    late_paths, late_markers = [], []
    late_path = executor.node.create_subscription(
        RosPath, executor.topic('path'), late_paths.append, retained_qos())
    late_marker = executor.node.create_subscription(
        Marker, executor.topic('trajectory_marker'), late_markers.append, retained_qos())
    executor.spin_until(lambda: bool(late_paths) and bool(late_markers))
    assert late_paths[-1].header.stamp == paths[-1].header.stamp
    assert late_markers[-1].header.stamp == markers[-1].header.stamp
    executor.settle()
    assert len(late_paths) == len(late_markers) == 1
    assert late_path and late_marker
