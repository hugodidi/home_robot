from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    random_arg = DeclareLaunchArgument(
        'random', default_value='false',
        description='Shuffle waypoint order (true/false)')
    skip_errors_arg = DeclareLaunchArgument(
        'skip_errors', default_value='false',
        description='Skip failed waypoints instead of stopping (true/false)')
    bt_xml_arg = DeclareLaunchArgument(
        'bt_xml', default_value='',
        description='BehaviorTree.CPP XML file to execute')
    groot_port_arg = DeclareLaunchArgument(
        'groot_port', default_value='1667',
        description='Groot2 live monitoring server port')

    random_val = LaunchConfiguration('random')
    skip_errors_val = LaunchConfiguration('skip_errors')
    bt_xml_val = LaunchConfiguration('bt_xml')
    groot_port_val = LaunchConfiguration('groot_port')
    bt_xml_arg_value = PythonExpression(["'--bt-xml=' + '", bt_xml_val, "'"])
    groot_port_arg_value = PythonExpression(["'--groot-port=' + '", groot_port_val, "'"])

    sequential = Node(
        package='home_robot_bt', executable='bt_patrol', name='bt_patrol',
        output='screen',
        arguments=[bt_xml_arg_value, groot_port_arg_value],
        condition=IfCondition(PythonExpression([
            "'", random_val, "' == 'false' and '", skip_errors_val, "' == 'false'"
        ])),
    )
    random = Node(
        package='home_robot_bt', executable='bt_patrol', name='bt_patrol',
        output='screen', arguments=['--random', bt_xml_arg_value, groot_port_arg_value],
        condition=IfCondition(PythonExpression([
            "'", random_val, "' == 'true' and '", skip_errors_val, "' == 'false'"
        ])),
    )
    sequential_skip = Node(
        package='home_robot_bt', executable='bt_patrol', name='bt_patrol',
        output='screen', arguments=['--skip-errors', bt_xml_arg_value, groot_port_arg_value],
        condition=IfCondition(PythonExpression([
            "'", random_val, "' == 'false' and '", skip_errors_val, "' == 'true'"
        ])),
    )
    random_skip = Node(
        package='home_robot_bt', executable='bt_patrol', name='bt_patrol',
        output='screen', arguments=['--random', '--skip-errors', bt_xml_arg_value, groot_port_arg_value],
        condition=IfCondition(PythonExpression([
            "'", random_val, "' == 'true' and '", skip_errors_val, "' == 'true'"
        ])),
    )

    return LaunchDescription([
        random_arg, skip_errors_arg, bt_xml_arg, groot_port_arg,
        sequential, random, sequential_skip, random_skip,
    ])
