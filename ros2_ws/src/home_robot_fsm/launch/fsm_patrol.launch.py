"""
Launch file for the FSM Patrol system.

Starts the YASMIN-based waypoint patrol node and opens the
YASMIN Viewer web interface.

Usage:
    ros2 launch home_robot_fsm fsm_patrol.launch.py
    ros2 launch home_robot_fsm fsm_patrol.launch.py random:=true
    ros2 launch home_robot_fsm fsm_patrol.launch.py random:=true skip_errors:=true

Prerequisites:
    The home_robot navigation stack must already be running:
        ros2 launch home_robot main_nav.launch.py use_slam:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    # ── Arguments ───────────────────────────────────────────────────
    random_arg = DeclareLaunchArgument(
        'random',
        default_value='false',
        description='Shuffle waypoint order (true/false)')

    skip_errors_arg = DeclareLaunchArgument(
        'skip_errors',
        default_value='false',
        description='Skip failed waypoints instead of aborting (true/false)')

    viewer_port_arg = DeclareLaunchArgument(
        'viewer_port',
        default_value='5000',
        description='Port for the YASMIN Viewer web interface')

    random_val = LaunchConfiguration('random')
    skip_errors_val = LaunchConfiguration('skip_errors')
    viewer_port = LaunchConfiguration('viewer_port')

    # ── FSM Patrol Node (sequential) ────────────────────────────────
    fsm_sequential = Node(
        package='home_robot_fsm',
        executable='fsm_patrol',
        name='fsm_patrol',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", random_val, "' == 'false' and '",
                              skip_errors_val, "' == 'false'"]),
        ),
    )

    # ── FSM Patrol Node (random only) ───────────────────────────────
    fsm_random = Node(
        package='home_robot_fsm',
        executable='fsm_patrol',
        name='fsm_patrol',
        output='screen',
        arguments=['--random'],
        condition=IfCondition(
            PythonExpression(["'", random_val, "' == 'true' and '",
                              skip_errors_val, "' == 'false'"]),
        ),
    )

    # ── FSM Patrol Node (sequential + skip errors) ──────────────────
    fsm_sequential_skip = Node(
        package='home_robot_fsm',
        executable='fsm_patrol',
        name='fsm_patrol',
        output='screen',
        arguments=['--skip-errors'],
        condition=IfCondition(
            PythonExpression(["'", random_val, "' == 'false' and '",
                              skip_errors_val, "' == 'true'"]),
        ),
    )

    # ── FSM Patrol Node (random + skip errors) ──────────────────────
    fsm_random_skip = Node(
        package='home_robot_fsm',
        executable='fsm_patrol',
        name='fsm_patrol',
        output='screen',
        arguments=['--random', '--skip-errors'],
        condition=IfCondition(
            PythonExpression(["'", random_val, "' == 'true' and '",
                              skip_errors_val, "' == 'true'"]),
        ),
    )

    # ── YASMIN Viewer (web UI) ──────────────────────────────────────
    #    Opens a web server on the specified port. Access via browser
    #    at http://localhost:<viewer_port>
    yasmin_viewer = Node(
        package='yasmin_viewer',
        executable='yasmin_viewer_node',
        name='yasmin_viewer_node',
        output='screen',
        parameters=[{'port': viewer_port}],
    )

    return LaunchDescription([
        random_arg,
        skip_errors_arg,
        viewer_port_arg,
        yasmin_viewer,
        fsm_sequential,
        fsm_random,
        fsm_sequential_skip,
        fsm_random_skip,
    ])
