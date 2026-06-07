from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    random_val = LaunchConfiguration('random').perform(context)
    skip_errors_val = LaunchConfiguration('skip_errors').perform(context)

    node_args = []
    if random_val == 'true':
        node_args.append('--random')
    if skip_errors_val == 'true':
        node_args.append('--skip-errors')

    return [Node(
        package='home_robot_pddl',
        executable='pddl_patrol',
        name='pddl_patrol',
        output='screen',
        arguments=node_args,
    )]


def generate_launch_description():
    random_arg = DeclareLaunchArgument(
        'random', default_value='false',
        description='Shuffle waypoint order')
    skip_errors_arg = DeclareLaunchArgument(
        'skip_errors', default_value='false',
        description='Skip failed waypoints instead of stopping')

    return LaunchDescription([
        random_arg,
        skip_errors_arg,
        OpaqueFunction(function=launch_setup),
    ])
