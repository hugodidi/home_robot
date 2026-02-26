import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_home_robot = get_package_share_directory('home_robot')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam_params_file = LaunchConfiguration(
        'slam_params_file',
        default=os.path.join(pkg_home_robot, 'config', 'slam_toolbox.yaml')
    )

    # SLAM Toolbox Launch
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_home_robot, 'config', 'slam_toolbox.yaml'),
            description='Full path to the ROS2 parameters file to use for the slam_toolbox node'),

        slam_launch
    ])
