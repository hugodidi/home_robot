#!/usr/bin/env python3
"""Custom Behavior Tree patrol node.

This implementation owns the patrol behavior tree instead of delegating the
patrol logic to Nav2's BT Navigator. Nav2 is still used as the navigation action
backend, but selection, fallback, recovery and event publication are implemented
as local BT nodes.
"""

from enum import Enum
import math
import os
import random
import signal
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.action import ActionClient
from std_msgs.msg import String


stop_patrol = False


class Status(Enum):
    SUCCESS = 'SUCCESS'
    FAILURE = 'FAILURE'
    RUNNING = 'RUNNING'


class Behavior:
    def __init__(self, name: str):
        self.name = name

    def tick(self, blackboard: dict) -> Status:
        raise NotImplementedError

    def reset(self) -> None:
        pass


class Sequence(Behavior):
    def __init__(self, name: str, children: list[Behavior]):
        super().__init__(name)
        self.children = children
        self.index = 0

    def tick(self, blackboard: dict) -> Status:
        while self.index < len(self.children):
            status = self.children[self.index].tick(blackboard)
            if status == Status.RUNNING:
                return Status.RUNNING
            if status == Status.FAILURE:
                self.reset()
                return Status.FAILURE
            self.children[self.index].reset()
            self.index += 1
        self.reset()
        return Status.SUCCESS

    def reset(self) -> None:
        self.index = 0
        for child in self.children:
            child.reset()


class Fallback(Behavior):
    def __init__(self, name: str, children: list[Behavior]):
        super().__init__(name)
        self.children = children
        self.index = 0

    def tick(self, blackboard: dict) -> Status:
        while self.index < len(self.children):
            status = self.children[self.index].tick(blackboard)
            if status == Status.RUNNING:
                return Status.RUNNING
            if status == Status.SUCCESS:
                self.reset()
                return Status.SUCCESS
            self.children[self.index].reset()
            self.index += 1
        self.reset()
        return Status.FAILURE

    def reset(self) -> None:
        self.index = 0
        for child in self.children:
            child.reset()


class PatrolLoop(Behavior):
    def __init__(self, child: Behavior):
        super().__init__('PatrolLoop')
        self.child = child

    def tick(self, blackboard: dict) -> Status:
        if blackboard.get('patrol_done', False):
            return Status.SUCCESS
        status = self.child.tick(blackboard)
        if status == Status.SUCCESS:
            self.child.reset()
            return Status.RUNNING
        if status == Status.FAILURE:
            if blackboard.get('patrol_done', False):
                return Status.SUCCESS
            return Status.FAILURE
        return Status.RUNNING

    def reset(self) -> None:
        self.child.reset()


class SelectWaypoint(Behavior):
    def __init__(self, navigator: BasicNavigator):
        super().__init__('SelectWaypoint')
        self.navigator = navigator

    def tick(self, blackboard: dict) -> Status:
        queue = blackboard['wp_queue']
        if not queue:
            blackboard['patrol_done'] = True
            self.navigator.get_logger().info('BT patrol complete: no waypoints left')
            return Status.FAILURE

        name, wp = queue.pop(0)
        blackboard['current_wp_name'] = name
        blackboard['current_wp'] = wp
        blackboard['navigation_failed'] = False
        total = blackboard['wp_total']
        current = total - len(queue)
        self.navigator.get_logger().info(f'[{current}/{total}] BT selected waypoint: {name}')
        return Status.SUCCESS


class NavigateToWaypoint(Behavior):
    def __init__(self, navigator: BasicNavigator, label: str,
                 stuck_timeout: float = 15.0, stuck_dist_threshold: float = 0.1):
        super().__init__(label)
        self.navigator = navigator
        self.started = False
        self.stuck_timeout = stuck_timeout
        self.stuck_dist_threshold = stuck_dist_threshold
        self.last_dist = None
        self.last_dist_time = None

    def tick(self, blackboard: dict) -> Status:
        name = blackboard['current_wp_name']
        if not self.started:
            pose = make_pose(self.navigator, blackboard['current_wp'])
            self.navigator.get_logger().info(f'{self.name}: sending Nav2 goal for {name}')
            self.navigator.goToPose(pose)
            self.started = True
            self.last_dist = None
            self.last_dist_time = None
            return Status.RUNNING

        feedback = self.navigator.getFeedback()
        now = time.time()
        if feedback and feedback.distance_remaining is not None:
            dist = feedback.distance_remaining
            if self.last_dist is not None:
                if abs(dist - self.last_dist) <= self.stuck_dist_threshold:
                    if now - self.last_dist_time >= self.stuck_timeout:
                        self.navigator.get_logger().warn(
                            f'{self.name}: STUCK detected ({self.stuck_timeout}s '
                            f'without progress). Aborting navigation.')
                        self.navigator.cancelTask()
                        blackboard['navigation_failed'] = True
                        blackboard['is_stuck'] = True
                        self.reset()
                        return Status.FAILURE
                else:
                    self.last_dist = dist
                    self.last_dist_time = now
            else:
                self.last_dist = dist
                self.last_dist_time = now

            if dist > 0:
                self.navigator.get_logger().info(f'{name}: {dist:.2f} m remaining')

        if not self.navigator.isTaskComplete():
            return Status.RUNNING

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.navigator.get_logger().info(f'{self.name}: reached {name}')
            return Status.SUCCESS

        blackboard['navigation_failed'] = True
        self.navigator.get_logger().warn(f'{self.name}: failed to reach {name}')
        return Status.FAILURE

    def reset(self) -> None:
        self.started = False
        self.last_dist = None
        self.last_dist_time = None


class BackUpRecovery(Behavior):
    def __init__(self, navigator: BasicNavigator, cmd_vel_pub):
        super().__init__('BackUpRecovery')
        self.navigator = navigator
        self.cmd_vel_pub = cmd_vel_pub
        self.start_time = None

    def tick(self, blackboard: dict) -> Status:
        del blackboard
        now = time.time()
        if self.start_time is None:
            self.navigator.get_logger().warn('BT recovery: moving backwards')
            self.start_time = now

        twist = Twist()
        if now - self.start_time < 1.0:
            twist.linear.x = -0.5
            self.cmd_vel_pub.publish(twist)
            return Status.RUNNING

        self.cmd_vel_pub.publish(Twist())
        return Status.SUCCESS

    def reset(self) -> None:
        self.start_time = None


class PublishArrival(Behavior):
    def __init__(self, navigator: BasicNavigator, event_pub):
        super().__init__('PublishArrival')
        self.navigator = navigator
        self.event_pub = event_pub

    def tick(self, blackboard: dict) -> Status:
        name = blackboard['current_wp_name']
        msg = String()
        msg.data = f'llegada:{name}'
        self.event_pub.publish(msg)
        self.navigator.get_logger().info(f'BT event published: {msg.data}')
        return Status.SUCCESS


class HandleFailure(Behavior):
    def __init__(self, navigator: BasicNavigator):
        super().__init__('HandleFailure')
        self.navigator = navigator

    def tick(self, blackboard: dict) -> Status:
        name = blackboard.get('current_wp_name', 'unknown')
        if blackboard.get('skip_errors', False):
            self.navigator.get_logger().warn(f'BT skipping failed waypoint: {name}')
            return Status.SUCCESS
        self.navigator.get_logger().error(f'BT stopping on failed waypoint: {name}')
        return Status.FAILURE


def signal_handler(signum, frame):
    del signum, frame
    global stop_patrol
    stop_patrol = True


def load_waypoints() -> dict:
    try:
        pkg_share = get_package_share_directory('home_robot')
        wp_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    except Exception:
        project_root = os.environ.get('PROJECT_ROOT', '/home/ubuntu/home_robot')
        wp_path = os.path.join(project_root, 'ros2_ws/src/home_robot/config/waypoints.yaml')

    if not os.path.exists(wp_path):
        raise FileNotFoundError(f'waypoints.yaml not found at {wp_path}')

    with open(wp_path, 'r') as stream:
        return yaml.safe_load(stream).get('waypoints', {})


def make_pose(navigator: BasicNavigator, wp: dict) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(wp['x'])
    pose.pose.position.y = float(wp['y'])
    theta = float(wp['theta'])
    pose.pose.orientation.z = math.sin(theta / 2.0)
    pose.pose.orientation.w = math.cos(theta / 2.0)
    return pose


def wait_for_nav2_action(navigator: BasicNavigator) -> bool:
    action_client = ActionClient(navigator, NavigateToPose, 'navigate_to_pose')
    start = time.time()
    while not action_client.wait_for_server(timeout_sec=1.0):
        if stop_patrol:
            return False
        if time.time() - start > 60.0:
            navigator.get_logger().error('Timeout waiting for /navigate_to_pose')
            return False
        navigator.get_logger().info('Waiting for /navigate_to_pose action server...')
    return True


def build_tree(navigator: BasicNavigator, event_pub, cmd_vel_pub) -> Behavior:
    return PatrolLoop(
        Sequence('PatrolOneWaypoint', [
            SelectWaypoint(navigator),
            Fallback('NavigateOrRecover', [
                Sequence('PrimaryNavigation', [
                    NavigateToWaypoint(navigator, 'NavigatePrimary'),
                    PublishArrival(navigator, event_pub),
                ]),
                Sequence('RecoveryNavigation', [
                    BackUpRecovery(navigator, cmd_vel_pub),
                    NavigateToWaypoint(navigator, 'NavigateRetry'),
                    PublishArrival(navigator, event_pub),
                ]),
                HandleFailure(navigator),
            ]),
        ])
    )


def main() -> None:
    global stop_patrol

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    use_random = '--random' in sys.argv
    skip_errors = '--skip-errors' in sys.argv

    rclpy.init()
    navigator = BasicNavigator()
    event_pub = navigator.create_publisher(String, 'patrol_events', 10)
    cmd_vel_pub = navigator.create_publisher(Twist, '/cmd_vel', 10)

    def stop_callback(msg):
        del msg
        global stop_patrol
        navigator.get_logger().warn('Stop request received on /stop_patrol')
        stop_patrol = True

    navigator.create_subscription(String, '/stop_patrol', stop_callback, 10)

    try:
        waypoints = load_waypoints()
    except FileNotFoundError as exc:
        navigator.get_logger().error(str(exc))
        rclpy.shutdown()
        return

    wp_items = list(waypoints.items())
    if use_random:
        random.shuffle(wp_items)

    blackboard = {
        'wp_queue': wp_items,
        'wp_total': len(wp_items),
        'skip_errors': skip_errors,
        'patrol_done': False,
    }

    navigator.get_logger().info('Custom BT patrol starting')
    navigator.get_logger().info(f'Mode: random={use_random}, skip_errors={skip_errors}')

    if not wait_for_nav2_action(navigator):
        rclpy.shutdown()
        return

    tree = build_tree(navigator, event_pub, cmd_vel_pub)

    try:
        while rclpy.ok() and not stop_patrol:
            rclpy.spin_once(navigator, timeout_sec=0.1)
            status = tree.tick(blackboard)
            navigator.get_logger().info(f'BT root status: {status.value}')
            if status in (Status.SUCCESS, Status.FAILURE):
                break
            time.sleep(0.2)
    finally:
        if rclpy.ok():
            navigator.cancelTask()
            cmd_vel_pub.publish(Twist())
            navigator.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
