#!/usr/bin/env python3
"""
FSM Patrol Node — YASMIN-based Finite State Machine for Home Robot.

This module implements a modular waypoint patrol system using YASMIN
state machines. It wraps Nav2's NavigateToPose action inside YASMIN
states, providing a visual FSM representation through the YASMIN Viewer.

The FSM has the following structure:

    ┌──────────────┐
    │ SELECT_WP    │──(finished)──► [patrol_done]
    └──────┬───────┘
           │ navigate
    ┌──────▼───────┐
    │ NAVIGATE     │──(abort/cancel)──► HANDLE_ERROR
    └──────┬───────┘
           │ succeeded
    ┌──────▼───────┐
    │ WP_REACHED   │──(next)──► SELECT_WP
    └──────────────┘

    ┌──────────────┐
    │ HANDLE_ERROR │──(recover)──► SELECT_WP
    │              │──(abort)────► [patrol_done]
    └──────────────┘

Usage:
    # After launching the navigation stack (main_nav.launch.py):
    ros2 run home_robot_fsm fsm_patrol
    ros2 run home_robot_fsm fsm_patrol -- --random
    ros2 run home_robot_fsm fsm_patrol -- --random --skip-errors

    # Or via launch file (starts viewer automatically):
    ros2 launch home_robot_fsm fsm_patrol.launch.py
    ros2 launch home_robot_fsm fsm_patrol.launch.py random:=true

Note:
    This package does NOT modify any existing home_robot code.
    It reads the waypoints from the home_robot package's installed
    config and uses Nav2's action interface independently.
"""

import os
import sys
import math
import yaml
import time
import random as random_module

import rclpy
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_ros import set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, CANCEL
from yasmin_viewer import YasminViewerPub

from ament_index_python.packages import get_package_share_directory


# ──────────────────────────────────────────────────────────────────────
#  Helper: Load waypoints from home_robot's installed config
# ──────────────────────────────────────────────────────────────────────

def load_waypoints() -> dict:
    """Load waypoints from the home_robot package's waypoints.yaml.

    Tries the installed share directory first, falls back to
    PROJECT_ROOT environment variable.

    Returns:
        dict: Waypoint dictionary ``{name: {x, y, theta}}``.

    Raises:
        FileNotFoundError: If the waypoints file cannot be located.
    """
    # Primary: installed package path
    try:
        pkg_share = get_package_share_directory('home_robot')
        wp_path = os.path.join(pkg_share, 'config', 'waypoints.yaml')
    except Exception:
        # Fallback: PROJECT_ROOT env var (Docker default)
        project_root = os.environ.get(
            'PROJECT_ROOT', '/home/ubuntu/home_robot')
        wp_path = os.path.join(
            project_root, 'ros2_ws/src/home_robot/config/waypoints.yaml')

    if not os.path.exists(wp_path):
        raise FileNotFoundError(
            f'Waypoints file not found at {wp_path}')

    with open(wp_path, 'r') as f:
        data = yaml.safe_load(f)
    return data.get('waypoints', {})


# ──────────────────────────────────────────────────────────────────────
#  State 1: SELECT_WAYPOINT — picks the next waypoint
# ──────────────────────────────────────────────────────────────────────

class SelectWaypointState(State):
    """Selects the next waypoint from the queue and writes it to the
    blackboard.

    Outcomes:
        ``navigate``  — a waypoint was selected, proceed to navigation.
        ``finished``  — all waypoints have been visited.
    """

    def __init__(self) -> None:
        super().__init__(['navigate', 'finished'])

    def execute(self, blackboard: Blackboard) -> str:
        wp_names = blackboard['wp_names']
        wp_dict = blackboard['wp_dict']
        wp_index = int(blackboard['wp_index'])

        if wp_index >= len(wp_names):
            yasmin.YASMIN_LOG_INFO(
                '🏁 All waypoints visited! Patrol complete.')
            return 'finished'

        # Advance by index instead of mutating a list in-place. This makes the
        # current patrol position explicit in the blackboard across transitions.
        wp_name = wp_names[wp_index]
        blackboard['wp_index'] = wp_index + 1
        wp_data = wp_dict[wp_name]

        # Store in blackboard for the NavigateState
        blackboard['current_wp_name'] = wp_name
        blackboard['current_wp_x'] = float(wp_data['x'])
        blackboard['current_wp_y'] = float(wp_data['y'])
        blackboard['current_wp_theta'] = float(wp_data['theta'])

        total = blackboard['wp_total']
        idx = wp_index + 1
        yasmin.YASMIN_LOG_INFO(
            f'[{idx}/{total}] 🎯 Selected waypoint: {wp_name} '
            f'(x={wp_data["x"]}, y={wp_data["y"]})')

        time.sleep(1.0)
        return 'navigate'


# ──────────────────────────────────────────────────────────────────────
#  State 2: NAVIGATE — ActionState wrapping NavigateToPose
# ──────────────────────────────────────────────────────────────────────

class NavigateState(ActionState):
    """Sends a NavigateToPose goal to Nav2 and monitors progress.

    Inherits from ``yasmin_ros.ActionState`` to handle the full
    action lifecycle (goal / feedback / result) automatically.

    Outcomes (inherited + custom):
        ``succeeded`` — navigation completed successfully.
        ``aborted``   — Nav2 aborted the goal.
        ``canceled``  — navigation was canceled externally.
    """

    def __init__(self) -> None:
        super().__init__(
            NavigateToPose,                 # action type
            '/navigate_to_pose',            # action name
            self._create_goal,              # goal callback
            set(),                          # extra outcomes (SUCCEED/ABORT/CANCEL auto)
            self._handle_result,            # result callback
            self._handle_feedback,          # feedback callback
        )
        self.last_dist = None
        self.last_time = None

    def _create_goal(
        self, blackboard: Blackboard
    ) -> NavigateToPose.Goal:
        """Build a NavigateToPose goal from blackboard data."""
        
        # Reset stuck checking variables for this new goal
        self.last_dist = None
        self.last_time = None
        blackboard['is_stuck'] = False
        
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'

        goal.pose.pose.position.x = blackboard['current_wp_x']
        goal.pose.pose.position.y = blackboard['current_wp_y']

        theta = blackboard['current_wp_theta']
        goal.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.pose.orientation.w = math.cos(theta / 2.0)

        yasmin.YASMIN_LOG_INFO(
            f'📤 Sending goal to Nav2: {blackboard["current_wp_name"]}')
        return goal

    def _handle_result(
        self,
        blackboard: Blackboard,
        result: NavigateToPose.Result,
    ) -> str:
        """Process the action result."""
        yasmin.YASMIN_LOG_INFO(
            f'✅ Nav2 reports success for: '
            f'{blackboard["current_wp_name"]}')
        return SUCCEED

    def _handle_feedback(
        self,
        blackboard: Blackboard,
        feedback: NavigateToPose.Feedback,
    ) -> None:
        """Log navigation feedback (remaining distance) and check for stuck."""
        dist = feedback.distance_remaining
        current_time = time.time()
        
        # Stuck detection logic
        if self.last_dist is not None:
            if abs(dist - self.last_dist) <= 0.1:
                # Has been within tolerance for how long?
                if (current_time - self.last_time) >= 15.0:
                    yasmin.YASMIN_LOG_WARN("🤖 Robot is physically stuck! Canceling navigation to recover.")
                    blackboard['is_stuck'] = True
                    self.cancel_state()
            else:
                # Moved out of tolerance, reset
                self.last_dist = dist
                self.last_time = current_time
        else:
            self.last_dist = dist
            self.last_time = current_time
            
        if dist > 0:
            yasmin.YASMIN_LOG_INFO(
                f'  📍 {blackboard["current_wp_name"]}: '
                f'{dist:.2f} m remaining')


# ──────────────────────────────────────────────────────────────────────
#  State 3: WP_REACHED — announces arrival and loops back
# ──────────────────────────────────────────────────────────────────────

class WaypointReachedState(State):
    """Announces that the robot has reached a waypoint.

    Publishes an event on ``/patrol_events`` so that the existing
    ``voice_controller`` node (if running) can announce the arrival.

    Outcomes:
        ``next`` — ready to select the next waypoint.
    """

    def __init__(self, node: Node) -> None:
        super().__init__(['next'])
        self._pub = node.create_publisher(String, 'patrol_events', 10)

    def execute(self, blackboard: Blackboard) -> str:
        wp_name = blackboard['current_wp_name']
        yasmin.YASMIN_LOG_INFO(
            f'🏠 Arrived at: {wp_name}')

        # Add a sleep so the web viewer can actually catch this state being active!
        time.sleep(1.5)

        # Publish arrival event (compatible with voice_controller)
        msg = String()
        msg.data = f'llegada:{wp_name}'
        self._pub.publish(msg)
        return 'next'


# ──────────────────────────────────────────────────────────────────────
#  State 3.5: ANTI-STUCK — evaluation and recovery logic
# ──────────────────────────────────────────────────────────────────────

class CheckStuckState(State):
    """Evaluates if the path was canceled due to being physically stuck."""
    def __init__(self) -> None:
        super().__init__(['stuck', 'error'])

    def execute(self, blackboard: Blackboard) -> str:
        if blackboard.get('is_stuck', False):
            blackboard['is_stuck'] = False  # Reset flag
            return 'stuck'
        return 'error'


class StuckRecoveryState(State):
    """Executes a recovery maneuver (moves backwards)."""
    def __init__(self, node: Node) -> None:
        super().__init__(['recovered'])
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)

    def execute(self, blackboard: Blackboard) -> str:
        yasmin.YASMIN_LOG_WARN("🚧 Executing stuck recovery (moving backwards 1m/s for 1s)")
        
        # Publish Twist backwards
        twist = Twist()
        twist.linear.x = -1.0
        
        # Wait for the viewer to update visually
        time.sleep(1.0)
        
        self.cmd_pub.publish(twist)
        time.sleep(1.2)  # Give it a bit over 1s backwards
        
        # Stop
        twist.linear.x = 0.0
        self.cmd_pub.publish(twist)
        time.sleep(0.5)
        
        return 'recovered'


# ──────────────────────────────────────────────────────────────────────
#  State 4: HANDLE_ERROR — recovery or abort
# ──────────────────────────────────────────────────────────────────────

class HandleErrorState(State):
    """Handles navigation failures.

    If ``skip_errors`` is enabled in the blackboard, the state
    transitions to *recover* (try the next waypoint). Otherwise
    it aborts the patrol.

    Outcomes:
        ``recover`` — skip this waypoint, try the next one.
        ``abort``   — stop the patrol entirely.
    """

    def __init__(self) -> None:
        super().__init__(['recover', 'abort'])

    def execute(self, blackboard: Blackboard) -> str:
        wp_name = blackboard['current_wp_name'] if 'current_wp_name' in blackboard else '???'
        skip = blackboard['skip_errors'] if 'skip_errors' in blackboard else False

        if skip:
            yasmin.YASMIN_LOG_WARN(
                f'⚠ Navigation failed for {wp_name}, skipping...')
            return 'recover'
        else:
            yasmin.YASMIN_LOG_ERROR(
                f'❌ Navigation failed for {wp_name}, aborting patrol.')
            return 'abort'


# ──────────────────────────────────────────────────────────────────────
#  Build the State Machine
# ──────────────────────────────────────────────────────────────────────

def build_fsm(node: Node) -> StateMachine:
    """Construct and return the patrol FSM.

    Args:
        node: ROS 2 node used for publishers inside states.

    Returns:
        StateMachine: The fully wired YASMIN state machine.
    """
    sm = StateMachine(outcomes=['patrol_done'], handle_sigint=True)

    sm.add_state(
        'SELECT_WAYPOINT',
        SelectWaypointState(),
        transitions={
            'navigate': 'NAVIGATE',
            'finished': 'patrol_done',
        },
    )

    sm.add_state(
        'NAVIGATE',
        NavigateState(),
        transitions={
            SUCCEED:  'WAYPOINT_REACHED',
            ABORT:    'HANDLE_ERROR',
            CANCEL:   'CHECK_STUCK',
        },
    )

    sm.add_state(
        'CHECK_STUCK',
        CheckStuckState(),
        transitions={
            'stuck': 'STUCK_RECOVERY',
            'error': 'HANDLE_ERROR',
        },
    )

    sm.add_state(
        'STUCK_RECOVERY',
        StuckRecoveryState(node),
        transitions={
            'recovered': 'NAVIGATE',
        },
    )

    sm.add_state(
        'WAYPOINT_REACHED',
        WaypointReachedState(node),
        transitions={
            'next': 'SELECT_WAYPOINT',
        },
    )

    sm.add_state(
        'HANDLE_ERROR',
        HandleErrorState(),
        transitions={
            'recover': 'SELECT_WAYPOINT',
            'abort':   'patrol_done',
        },
    )

    return sm


# ──────────────────────────────────────────────────────────────────────
#  Main
# ──────────────────────────────────────────────────────────────────────

def main() -> None:
    """Entry point for the FSM patrol node.

    CLI flags (pass after ``--``):
        ``--random``       Shuffle waypoints instead of sequential order.
        ``--skip-errors``  Skip failed waypoints instead of aborting.
    """
    rclpy.init()

    # ROS 2 + YASMIN logging
    set_ros_loggers()
    yasmin.YASMIN_LOG_INFO('🤖 home_robot_fsm — FSM Patrol starting')

    # Create a lightweight node for publishers
    node = rclpy.create_node('fsm_patrol_node')

    # Parse CLI flags
    use_random = '--random' in sys.argv
    skip_errors = '--skip-errors' in sys.argv

    # Load waypoints from home_robot package
    try:
        wp_dict = load_waypoints()
    except FileNotFoundError as e:
        yasmin.YASMIN_LOG_ERROR(str(e))
        rclpy.shutdown()
        return

    wp_names = list(wp_dict.keys())
    if use_random:
        random_module.shuffle(wp_names)
        yasmin.YASMIN_LOG_INFO('🎲 Random mode activated')
    else:
        yasmin.YASMIN_LOG_INFO('📋 Sequential mode')

    yasmin.YASMIN_LOG_INFO(
        f'Loaded {len(wp_names)} waypoints: {", ".join(wp_names)}')

    # Build FSM
    sm = build_fsm(node)

    # Publish FSM structure for YASMIN Viewer
    YasminViewerPub(sm, 'HOME_ROBOT_FSM_PATROL')

    # Prepare blackboard
    blackboard = Blackboard()
    blackboard['wp_dict'] = wp_dict
    blackboard['wp_names'] = list(wp_names)
    blackboard['wp_index'] = 0
    blackboard['wp_total'] = len(wp_names)
    blackboard['skip_errors'] = skip_errors

    # Execute FSM
    try:
        outcome = sm(blackboard)
        yasmin.YASMIN_LOG_INFO(f'FSM finished with outcome: {outcome}')
    except KeyboardInterrupt:
        yasmin.YASMIN_LOG_WARN('🛑 Patrol interrupted by user')
    except Exception as e:
        yasmin.YASMIN_LOG_ERROR(f'FSM error: {e}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
