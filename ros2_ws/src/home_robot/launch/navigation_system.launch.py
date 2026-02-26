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

    # Static map server: publishes /map when not using SLAM, /map_static when using SLAM
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}],
        remappings=[('/map', '/map_static')],
        condition=IfCondition(use_slam)  # Solo remap en modo SLAM
    )
    
    # Map server sin remapping para modo navegación normal
    map_server_localization_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}],
        condition=UnlessCondition(use_slam)  # Sin remap en modo localization
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='When true, Nav2 subscribes to SLAM map (/map); when false, static map_server publishes /map_static and is used for localization'),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_home_robot, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use for all navigation nodes'),

        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_home_robot, 'maps', 'mapa_20260214_221450.yaml'),
            description='Full path to map yaml file to load'),
        nav2_launch,
        map_server_node,
        map_server_localization_node,
        lifecycle_manager_node,
    ])
