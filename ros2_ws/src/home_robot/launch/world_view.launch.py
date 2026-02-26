import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, TextSubstitution


def generate_launch_description():
    """
    Simple launch file to visualize and explore the Gazebo world without any robot.
    Useful for map creation, world verification, or environment testing.
    """
    pkg_share = get_package_share_directory('home_robot')

    # Default world
    default_world = os.path.join(pkg_share, "worlds", "final_world.sdf")

    # Arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=TextSubstitution(text=default_world),
        description="Path to the Gazebo world file (SDF)"
    )

    # Resource paths for Gazebo
    gz_paths = f"{os.path.join(pkg_share, 'models')}:{os.path.join(pkg_share, 'worlds')}:"
    gz_sim_resource_path = [
        TextSubstitution(text=gz_paths),
        EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")
    ]

    # Launch Gazebo Sim
    gazebo = ExecuteProcess(
        cmd=[
            "ros2", "launch", "ros_gz_sim", "gz_sim.launch.py",
            ["gz_args:=", LaunchConfiguration("world"), " -r"],
        ],
        output="screen",
    )

    return LaunchDescription([
        world_arg,
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gz_sim_resource_path),
        SetEnvironmentVariable(
            name="__EGL_VENDOR_LIBRARY_FILENAMES",
            value="/usr/share/glvnd/egl_vendor.d/10_nvidia.json"
        ),
        gazebo,
    ])
