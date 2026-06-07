#!/usr/bin/env python3
"""PDDL patrol controller with execution monitoring and replanning.

Generates a PDDL problem from waypoints, calls the POPF planner binary
directly, parses the plan, and executes move actions against Nav2.
If the robot gets stuck, it performs a recovery manoeuvre (backs up
for ~1 s) and replans from the new state.
"""

import math
import os
import random
import re
import signal
import subprocess
import sys
import time
from typing import List, Tuple

import json
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from home_robot_pddl.problem_generator import (
    load_waypoints, sanitize, write_problem, resolve_start
)


stop_patrol = False


def signal_handler(signum, frame):
    del signum, frame
    global stop_patrol
    stop_patrol = True


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


def publish_markers(pub, navigator: BasicNavigator, waypoints: dict, visited: set, current: str = ''):
    markers = MarkerArray()
    stamp = navigator.get_clock().now().to_msg()
    for idx, (name, wp) in enumerate(waypoints.items()):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = 'pddl_patrol_waypoints'
        marker.id = idx
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(wp['x'])
        marker.pose.position.y = float(wp['y'])
        marker.pose.position.z = 0.15
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        if name == current:
            marker.color.b = 1.0
        elif name in visited:
            marker.color.g = 1.0
        else:
            marker.color.r = 0.7
            marker.color.g = 0.7
            marker.color.b = 0.7
        markers.markers.append(marker)
    pub.publish(markers)


def run_popf(domain_path: str, problem_path: str) -> str:
    popf_bin = '/opt/ros/jazzy/lib/popf/popf'
    if not os.path.exists(popf_bin):
        raise FileNotFoundError(f'POPF binary not found at {popf_bin}')

    result = subprocess.run(
        [popf_bin, domain_path, problem_path],
        capture_output=True, text=True, timeout=60
    )
    return result.stdout + result.stderr


def parse_plan(popf_output: str) -> List[Tuple[str, ...]]:
    plan = []
    for line in popf_output.splitlines():
        m = re.match(r'^\s*[\d.]+:\s*\(([^)]+)\)', line)
        if m:
            tokens = m.group(1).split()
            if tokens:
                plan.append(tuple(tokens))
    return plan


def get_current_waypoint(navigator, tf_buffer, listener, waypoints: dict, timeout_sec: float = 3.0) -> str:
    """Return nearest symbolic waypoint from TF map->base_link."""
    deadline = navigator.get_clock().now() + rclpy.duration.Duration(seconds=timeout_sec)
    while navigator.get_clock().now() < deadline:
        try:
            transform = tf_buffer.lookup_transform('map', 'base_link', Time())
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return nearest_waypoint(waypoints, x, y)
        except Exception:
            rclpy.spin_once(navigator, timeout_sec=0.1)
    names = [sanitize(n) for n in waypoints.keys()]
    return 'inicio' if 'inicio' in names else names[0]


def nearest_waypoint(waypoints: dict, x: float, y: float) -> str:
    best_name = None
    best_dist = None
    for name, wp in waypoints.items():
        dist = math.hypot(float(wp['x']) - x, float(wp['y']) - y)
        if best_dist is None or dist < best_dist:
            best_name = sanitize(name)
            best_dist = dist
    return best_name


def recover_from_stuck(navigator, cmd_vel_pub):
    """Back up for ~1 second to unstick the robot."""
    navigator.get_logger().warn('Executing stuck recovery (back up 1s)')
    twist = Twist()
    twist.linear.x = -0.5
    cmd_vel_pub.publish(twist)
    time.sleep(1.0)
    twist.linear.x = 0.0
    cmd_vel_pub.publish(twist)
    navigator.get_logger().info('Recovery complete')


def build_plan_ui(completed_actions: List[str], current_plan: List[Tuple[str, ...]],
                   active_idx: int = -1):
    """Build list of dicts for the web UI.

    Completed actions (already executed) stay at the top in grey.
    Current plan from POPF follows below.  active_idx indexes into
    current_plan (0 = first action of the new plan).
    """
    out = []
    for action in completed_actions:
        out.append({'action': action, 'status': 'done'})
    for idx, action in enumerate(current_plan):
        label = ' '.join(action)
        if idx == active_idx:
            status = 'active'
        else:
            status = 'pending'
        out.append({'action': label, 'status': status})
    return out


def publish_ui_state(navigator, ui_pub, completed_actions, current_plan,
                     replanning: bool = False, active_idx: int = -1,
                     current_wp: str = '', visited=None):
    plan_ui = build_plan_ui(completed_actions, current_plan, active_idx)
    payload = {
        'plan': plan_ui,
        'replanning': replanning,
        'current_wp': current_wp,
        'visited': sorted(visited) if visited else [],
    }
    msg = String()
    msg.data = json.dumps(payload)
    ui_pub.publish(msg)


def execute_move(
    navigator,
    wp: dict,
    cmd_vel_pub,
    event_pub,
    marker_pub,
    waypoints: dict,
    visited: set,
    target_name: str,
    action_timeout: float = 120.0,
    stuck_timeout: float = 15.0,
    stuck_dist_threshold: float = 0.05,
) -> bool:
    """Send Nav2 goal, monitor for stuck/timeout, return True if succeeded."""
    navigator.goToPose(make_pose(navigator, wp))
    publish_markers(marker_pub, navigator, waypoints, visited, target_name)

    start_time = time.time()
    last_pos = None
    last_move_time = start_time
    action_client = ActionClient(navigator, NavigateToPose, 'navigate_to_pose')

    try:
        while not stop_patrol:
            rclpy.spin_once(navigator, timeout_sec=0.1)

            try:
                if navigator.isTaskComplete():
                    break
            except rclpy.executors.ExternalShutdownException:
                navigator.get_logger().info('ExternalShutdownException during isTaskComplete')
                return False

            # Global action timeout
            if time.time() - start_time > action_timeout:
                navigator.get_logger().error(
                    f'Action timeout ({action_timeout}s) for {target_name}')
                try:
                    navigator.cancelTask()
                except Exception:
                    pass
                return False

    except KeyboardInterrupt:
        pass

    if stop_patrol:
        try:
            navigator.cancelTask()
        except Exception:
            pass
        return False

    try:
        result = navigator.getResult()
    except Exception:
        return False
    if result == TaskResult.SUCCEEDED:
        return True
    return False


def main() -> None:
    global stop_patrol

    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)

    use_random = '--random' in sys.argv
    skip_errors = '--skip-errors' in sys.argv
    links_mode = 'sequential' if '--sequential-links' in sys.argv else 'all_to_all'

    rclpy.init()
    navigator = BasicNavigator()
    event_pub = navigator.create_publisher(String, 'patrol_events', 10)
    cmd_vel_pub = navigator.create_publisher(Twist, '/cmd_vel', 10)
    marker_pub = navigator.create_publisher(MarkerArray, 'pddl_patrol/markers', 10)
    ui_pub = navigator.create_publisher(String, '/pddl_patrol/ui_state', 10)

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, navigator)

    try:
        waypoints = load_waypoints()
    except Exception as exc:
        navigator.get_logger().error(str(exc))
        rclpy.shutdown()
        return

    pkg_share = None
    try:
        import ament_index_python
        pkg_share = ament_index_python.get_package_share_directory('home_robot_pddl')
    except Exception:
        pass
    domain_path = os.path.join(
        os.path.dirname(__file__), '..', 'pddl', 'patrol_domain.pddl'
    )
    if pkg_share:
        domain_path = os.path.join(pkg_share, 'pddl', 'patrol_domain.pddl')

    if not os.path.exists(domain_path):
        navigator.get_logger().error(f'Domain file not found: {domain_path}')
        rclpy.shutdown()
        return

    navigator.get_logger().info(f'Domain: {domain_path}')

    if not wait_for_nav2_action(navigator):
        rclpy.shutdown()
        return

    waypoint_by_symbol = {sanitize(name): (name, wp) for name, wp in waypoints.items()}
    visited: set = set()
    publish_markers(marker_pub, navigator, waypoints, visited)

    # Give TF a moment to populate
    time.sleep(0.5)

    replan_count = 0
    max_replans = 20
    completed_actions: List[str] = []  # persistent across replans for the UI

    try:
        while replan_count < max_replans and not stop_patrol:
            # 1. Determine current symbolic location
            current_wp = get_current_waypoint(navigator, tf_buffer, tf_listener, waypoints)
            navigator.get_logger().info(
                f'Replanning {replan_count}: current symbolic wp = {current_wp}, '
                f'visited = {sorted(visited)}')

            publish_ui_state(
                navigator, ui_pub,
                completed_actions=completed_actions,
                current_plan=[],
                replanning=True,
                current_wp=current_wp,
                visited=visited,
            )

            # 2. Generate problem with visited waypoints recorded
            problem_path = write_problem(
                '/tmp/home_robot_patrol_problem.pddl',
                links_mode,
                start=current_wp,
                visited_names=sorted(visited),
                blocked=False,
            )

            # 3. Call POPF
            try:
                popf_output = run_popf(domain_path, problem_path)
            except Exception as exc:
                navigator.get_logger().error(f'POPF failed: {exc}')
                break

            plan = parse_plan(popf_output)
            if not plan:
                navigator.get_logger().info(
                    'POPF returned no plan (all goals already satisfied)')
                break

            navigator.get_logger().info('POPF plan:')
            for idx, action in enumerate(plan):
                navigator.get_logger().info(f'  {idx}: ({" ".join(action)})')

            # Show merged plan: completed greyed out + new plan below
            publish_ui_state(
                navigator, ui_pub,
                completed_actions=completed_actions,
                current_plan=plan,
                replanning=False,
                active_idx=-1,
                current_wp=current_wp,
                visited=visited,
            )

            # 4. Execute the first move action of the plan
            first_action = plan[0]
            if len(first_action) < 4 or first_action[0] != 'move':
                navigator.get_logger().warn(f'Skipping unknown first action: {first_action}')
                if first_action[0] == 'recover':
                    recover_from_stuck(navigator, cmd_vel_pub)
                replan_count += 1
                continue

            _, robot, src, dst = first_action
            if dst not in waypoint_by_symbol:
                navigator.get_logger().error(f'Unknown waypoint in plan: {dst}')
                break

            real_name, wp = waypoint_by_symbol[dst]
            navigator.get_logger().info(
                f'Execute: move {robot} {src} {dst} -> {real_name}')

            # Mark first action as active in the merged view
            publish_ui_state(
                navigator, ui_pub,
                completed_actions=completed_actions,
                current_plan=plan,
                replanning=False,
                active_idx=0,
                current_wp=dst,
                visited=visited,
            )

            # 5. Execute with Nav2
            success = execute_move(
                navigator, wp, cmd_vel_pub, event_pub, marker_pub,
                waypoints, visited, real_name,
            )

            if success:
                visited.add(real_name)
                publish_markers(marker_pub, navigator, waypoints, visited)
                msg = String()
                msg.data = f'llegada:{real_name}'
                event_pub.publish(msg)
                navigator.get_logger().info(f'Reached {dst}')
                # Append to persistent history so it stays greyed out
                completed_actions.append(f'move {robot} {src} {dst}')
            else:
                navigator.get_logger().warn(
                    f'Failed to reach {dst}. Attempting recovery and replan...')
                if not skip_errors:
                    recover_from_stuck(navigator, cmd_vel_pub)
                else:
                    navigator.get_logger().warn('skip-errors: skipping without recovery')

            replan_count += 1

        navigator.get_logger().info('PDDL patrol finished')
    finally:
        if rclpy.ok():
            navigator.cancelTask()
            cmd_vel_pub.publish(Twist())
            navigator.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
