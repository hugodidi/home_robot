import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package
    pkg_share = FindPackageShare('home_robot').find('home_robot')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_slam = LaunchConfiguration('use_slam', default='false')

    rviz_config = os.path.join(pkg_share, 'rviz', 'nav_view.rviz')

    # 1. Base Simulation (Gazebo + Bridge + Robot)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'project.launch.py')
        ),
        launch_arguments={
            'world': os.path.join(pkg_share, 'worlds', 'final_world.sdf'),
        }.items()
    )

    # Odom to TF publisher (critical - must start before Nav2)
    odom_to_tf_node = Node(
        package='home_robot',
        executable='odom_to_tf',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Joint state publisher to provide wheel transforms for RViz (Gazebo already drives motion)
    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static TF to align Gazebo laser frame with our base_scan link (bridge publishes frame
    # turtlebot3_burger/base_scan/hls_lfcd_lds)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_frame_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'base_scan', 'turtlebot3_burger/base_scan/hls_lfcd_lds'],
        output='screen'
    )
    

    # 2. SLAM System (optional, enabled when use_slam:=true)
    slam_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'slam_system.launch.py')
                ),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ],
        condition=IfCondition(use_slam)
    )

    # 3. Navigation System (Delayed 10s to ensure SLAM/TF is stable)
    nav_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'navigation_system.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'use_slam': use_slam
                }.items()
            )
        ]
    )

    # 4. Navigation Service Node (Exposes /go_to_pose service)
    nav_service_node = Node(
        package='home_robot',
        executable='navigation_service',
        name='navigation_service',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 5. RViz (delayed to allow TF/map to appear)
    rviz_launch = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}],
            )
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'use_slam',
            default_value='false',
            description='Enable SLAM to build a live map (map_static remains as background).'),

        base_launch,
        odom_to_tf_node,  # Start TF publishing immediately after base
        joint_state_pub,
        laser_tf,
        slam_launch,
        nav_launch,
        nav_service_node,
        rviz_launch
    ])
