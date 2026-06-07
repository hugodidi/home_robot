#!/usr/bin/env python3
"""Generate a PDDL patrol problem from home_robot waypoints."""

import argparse
import math
import os
from typing import Dict, List, Tuple

import yaml
from ament_index_python.packages import get_package_share_directory

try:
    import rclpy
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener
except Exception:
    rclpy = None
    Time = None
    Buffer = None
    TransformListener = None


def _project_root() -> str:
    return os.environ.get('PROJECT_ROOT', '/home/ubuntu/home_robot')


def load_waypoints() -> Dict[str, dict]:
    try:
        pkg_share = get_package_share_directory('home_robot')
        wp_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    except Exception:
        wp_path = os.path.join(
            _project_root(), 'ros2_ws/src/home_robot/config/waypoints.yaml')

    if not os.path.exists(wp_path):
        raise FileNotFoundError(f'waypoints.yaml not found at {wp_path}')

    with open(wp_path, 'r') as stream:
        return yaml.safe_load(stream).get('waypoints', {})


def load_robot_speed() -> float:
    """Read max linear velocity from Nav2 params (m/s)."""
    candidates = [
        os.path.join(_project_root(), 'ros2_ws/src/home_robot/config/nav2_params_slam.yaml'),
        os.path.join(_project_root(), 'ros2_ws/src/home_robot/config/nav2_params.yaml'),
    ]
    for path in candidates:
        if not os.path.exists(path):
            continue
        try:
            with open(path, 'r') as stream:
                cfg = yaml.safe_load(stream)
            max_vel = cfg.get('velocity_smoother', {}).get('ros__parameters', {}).get('max_velocity')
            if isinstance(max_vel, list) and len(max_vel) >= 1:
                speed = float(max_vel[0])
                if speed > 0:
                    return speed
        except Exception:
            continue
    return 0.5


def sanitize(name: str) -> str:
    return name.lower().replace('-', '_').replace(' ', '_')


def sequential_links(names: List[str]) -> List[Tuple[str, str]]:
    links = []
    for idx in range(len(names) - 1):
        links.append((names[idx], names[idx + 1]))
        links.append((names[idx + 1], names[idx]))
    return links


def all_to_all_links(names: List[str]) -> List[Tuple[str, str]]:
    return [(a, b) for a in names for b in names if a != b]


def nearest_waypoint(waypoints: Dict[str, dict], x: float, y: float) -> str:
    best_name = None
    best_dist = None
    for name, wp in waypoints.items():
        dist = math.hypot(float(wp['x']) - x, float(wp['y']) - y)
        if best_dist is None or dist < best_dist:
            best_name = sanitize(name)
            best_dist = dist
    return best_name


def resolve_start(waypoints: Dict[str, dict], start: str = 'auto', tf_timeout: float = 5.0) -> str:
    names = [sanitize(name) for name in waypoints.keys()]
    if start != 'auto':
        clean = sanitize(start)
        if clean not in names:
            raise ValueError(f'Unknown start waypoint: {start}')
        return clean

    if rclpy is None:
        return 'inicio' if 'inicio' in names else names[0]

    did_init = False
    if not rclpy.ok():
        rclpy.init(args=None)
        did_init = True
    node = rclpy.create_node('patrol_problem_start_resolver')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    deadline = node.get_clock().now() + Duration(seconds=tf_timeout)

    try:
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                transform = tf_buffer.lookup_transform('map', 'base_link', Time())
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                start_wp = nearest_waypoint(waypoints, x, y)
                node.get_logger().info(
                    f'Auto PDDL start: robot at ({x:.2f}, {y:.2f}) -> {start_wp}')
                return start_wp
            except Exception:
                pass
        node.get_logger().warn('Could not resolve map->base_link; falling back to inicio')
    finally:
        node.destroy_node()
        if did_init:
            rclpy.shutdown()

    return 'inicio' if 'inicio' in names else names[0]


def _travel_time(waypoints: Dict[str, dict], src: str, dst: str, speed: float) -> float:
    s = waypoints.get(src, {})
    d = waypoints.get(dst, {})
    dist = math.hypot(float(s.get('x', 0)) - float(d.get('x', 0)),
                      float(s.get('y', 0)) - float(d.get('y', 0)))
    return dist / speed if speed > 0 else 5.0


def generate_problem(
    links_mode: str = 'all_to_all',
    start: str = 'auto',
    tf_timeout: float = 5.0,
    robot_speed: float = None,
    visited_names=None,
    blocked: bool = False,
) -> str:
    if robot_speed is None or robot_speed <= 0:
        robot_speed = load_robot_speed()
    waypoints = load_waypoints()
    names = [sanitize(name) for name in waypoints.keys()]
    start = resolve_start(waypoints, start, tf_timeout)
    links = all_to_all_links(names) if links_mode == 'all_to_all' else sequential_links(names)

    objects = ' '.join(names)
    init_lines = [f'    (robot_at robot1 {start})']
    already_visited = set(visited_names) if visited_names else set()
    already_visited.add(start)
    for v in sorted(already_visited):
        if v in names:
            init_lines.append(f'    (visited {v})')
    init_lines.extend(f'    (connected {src} {dst})' for src, dst in links)
    for src, dst in links:
        t = _travel_time(waypoints, src, dst, robot_speed)
        init_lines.append(f'    (= (travel_time {src} {dst}) {t:.2f})')
    if blocked:
        init_lines.append('    (blocked)')
    goal_lines = [f'      (visited {name})' for name in names]
    goal_lines.append(f'      (robot_at robot1 inicio)')

    return f"""(define (problem home_robot_patrol_problem)
  (:domain home_robot_patrol)

  (:objects
    robot1 - robot
    {objects} - location
  )

  (:init
{os.linesep.join(init_lines)}
  )

  (:goal
    (and
{os.linesep.join(goal_lines)}
    )
  )
)
"""


def generate_terminal_commands(
    links_mode: str = 'all_to_all', start: str = 'auto', tf_timeout: float = 5.0
) -> str:
    waypoints = load_waypoints()
    names = [sanitize(name) for name in waypoints.keys()]
    start = resolve_start(waypoints, start, tf_timeout)
    links = all_to_all_links(names) if links_mode == 'all_to_all' else sequential_links(names)

    lines = ['set instance robot1 robot']
    lines.extend(f'set instance {name} location' for name in names)
    lines.append(f'set predicate (robot_at robot1 {start})')
    lines.append(f'set predicate (visited {start})')
    lines.extend(f'set predicate (connected {src} {dst})' for src, dst in links)
    # plansys2_terminal is picky with compound expression spacing. The compact
    # form matches the official examples: set goal (and(predicate)(predicate))
    goal = '(and' + ''.join(f'(visited {name})' for name in names) + ')'
    lines.append(f'set goal {goal}')
    lines.append('run')
    return os.linesep.join(lines) + os.linesep


def write_problem(
    output: str,
    links_mode: str = 'all_to_all',
    start: str = 'auto',
    tf_timeout: float = 5.0,
    robot_speed: float = None,
    visited_names=None,
    blocked: bool = False,
) -> str:
    problem = generate_problem(links_mode, start, tf_timeout, robot_speed, visited_names, blocked)
    with open(output, 'w') as stream:
        stream.write(problem)
    return output


def write_terminal_commands(
    output: str, links_mode: str = 'all_to_all', start: str = 'auto', tf_timeout: float = 5.0
) -> str:
    commands = generate_terminal_commands(links_mode, start, tf_timeout)
    with open(output, 'w') as stream:
        stream.write(commands)
    return output


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--output', default='/tmp/home_robot_patrol_problem.pddl')
    parser.add_argument(
        '--links', default='all_to_all', choices=['all_to_all', 'sequential'])
    parser.add_argument(
        '--start', default='auto',
        help='Initial symbolic waypoint. Use auto to choose nearest waypoint from TF map->base_link')
    parser.add_argument(
        '--tf-timeout', type=float, default=5.0,
        help='Seconds to wait for TF when --start auto')
    parser.add_argument(
        '--robot-speed', type=float, default=None,
        help='Robot cruising speed in m/s for travel_time estimation. '
             'If omitted, reads max_velocity from home_robot/config/nav2_params*.yaml')
    parser.add_argument(
        '--terminal-commands', default='',
        help='Optional output path for PlanSys2 terminal commands')
    args = parser.parse_args()
    path = write_problem(args.output, args.links, args.start, args.tf_timeout, args.robot_speed)
    print(f'Generated PDDL problem: {path}')
    if args.terminal_commands:
        commands_path = write_terminal_commands(
            args.terminal_commands, args.links, args.start, args.tf_timeout)
        print(f'Generated PlanSys2 terminal commands: {commands_path}')


if __name__ == '__main__':
    main()
