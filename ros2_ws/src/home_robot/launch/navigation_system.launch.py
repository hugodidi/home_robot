import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_home_robot = get_package_share_directory('home_robot')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='false')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_home_robot, 'config', 'nav2_params.yaml')
    )
    map_yaml_file = LaunchConfiguration(
        'map',
        default=os.path.join(pkg_home_robot, 'maps', 'mapa_20260214_221450.yaml')
    )

    # Nav2 Navigation Launch
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
            'use_map_topic': use_slam,
            'map': map_yaml_file
        }.items()
    )

    # Map server configuration
    # MODE A: SLAM (remaps to /map_static)
    map_server_slam_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}],
        remappings=[('/map', '/map_static')],
        condition=IfCondition(use_slam)
    )

    # MODE B: Localization (publishes to /map)
    map_server_loc_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}],
        condition=UnlessCondition(use_slam)
    )

    # AMCL: Only for localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(use_slam)
    )

    # Lifecycle managers
    # Localization: Manages map_server + amcl
    lc_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}],
        condition=UnlessCondition(use_slam)
    )

    # SLAM: Manages ONLY map_server
    lc_manager_slam = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}],
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='When true, uses SLAM; when false, uses AMCL + Static Map'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_home_robot, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_home_robot, 'maps', 'mapa_20260214_221450.yaml'),
            description='Full path to map yaml file'),

        nav2_launch,
        map_server_slam_node,
        map_server_loc_node,
        amcl_node,
        lc_manager_loc,
        lc_manager_slam,
    ])
