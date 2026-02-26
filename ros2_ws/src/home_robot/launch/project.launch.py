import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'home_robot'
    pkg_share = get_package_share_directory(package_name)

    # Rutas por defecto
    default_world = os.path.join(pkg_share, "worlds", "final_world.sdf")
    default_model = os.path.join(pkg_share, "models", "tb3_burger", "model.sdf")

    # Nav2 bringup path
    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )
    
    # Slam toolbox path
    slam_launch_path = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch',
        'online_async_launch.py'
    )

    # Args
    world_arg = DeclareLaunchArgument("world", default_value=TextSubstitution(text=default_world))
    model_arg = DeclareLaunchArgument("model", default_value=TextSubstitution(text=default_model))
    x_arg = DeclareLaunchArgument("x", default_value="7.0")
    y_arg = DeclareLaunchArgument("y", default_value="2.0")
    z_arg = DeclareLaunchArgument("z", default_value="0.1")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="3.1416")

    # Resource paths para Gazebo
    gz_paths = f"{os.path.join(pkg_share, 'models')}:{os.path.join(pkg_share, 'worlds')}:"
    gz_sim_resource_path = [TextSubstitution(text=gz_paths), EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")]

    # 1. Gazebo Sim
    gazebo_env = {}
    nvidia_egl_vendor = "/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
    if os.path.exists(nvidia_egl_vendor):
        gazebo_env["__EGL_VENDOR_LIBRARY_FILENAMES"] = nvidia_egl_vendor

    gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "ros_gz_sim", "gz_sim.launch.py",
            ["gz_args:=", LaunchConfiguration("world"), " -r"],
        ],
        additional_env=gazebo_env,
        output="screen",
    )

    # 2. Spawn Robot
    spawn = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "maze_world",
            "-name", "turtlebot3_burger",
            "-file", LaunchConfiguration("model"),
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    spawn_delayed = TimerAction(
        period=2.0,
        actions=[spawn],
    )

    # 3. ROS-GZ Bridge (Configuración YAML externa)
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen'
    )

    # 4. Robot State Publisher (URDF completo del TurtleBot3 Burger oficial)
    tb3_desc_share = get_package_share_directory('turtlebot3_description')
    urdf_file = os.path.join(tb3_desc_share, 'urdf', 'turtlebot3_burger.urdf')
    robot_desc = xacro.process_file(urdf_file, mappings={'namespace': ''}).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc
        }]
    )

    # 7. Nodo de servicio de cámara cenital
    overhead_cam_node = Node(
        package=package_name,
        executable='overhead_cam_service',
        output='screen'
    )

    return LaunchDescription([
        world_arg, model_arg,
        x_arg, y_arg, z_arg, yaw_arg,

        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_sim_resource_path),
        gazebo,
        spawn_delayed,
        bridge,
        robot_state_publisher,
        overhead_cam_node,
    ])
