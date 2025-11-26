import os
import shutil
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node


def is_wsl():
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False


def detect_terminal_prefix():
    """Select a terminal prefix if a GUI terminal is available; otherwise none."""
    if not os.environ.get('DISPLAY'):
        print("No DISPLAY found (headless). Launching nodes without an extra terminal.")
        return ""

    if is_wsl():
        if shutil.which('xterm'):
            print("Current system is WSL, use xterm as terminal")
            return "xterm -e"
        print("Current system is WSL, but xterm is not available. Launching nodes without an extra terminal.")
        return ""

    if shutil.which('gnome-terminal'):
        print("Current system is not WSL, use gnome-terminal as terminal")
        return "gnome-terminal --"

    if shutil.which('xterm'):
        print("Current system is not WSL, fallback to xterm as terminal")
        return "xterm -e"

    print("No supported terminal emulator found. Launching nodes without an extra terminal.")
    return ""


def generate_launch_description():
    prefix = detect_terminal_prefix()

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='task_name',
            default_value='mpc'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/visualize.launch.py']),
            launch_arguments={
                'use_joint_state_publisher': 'false'
            }.items()
        ),
        Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_mpc_mrt',
            name='ballbot_mpc_mrt',
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        ),
        Node(
            package='ocs2_ballbot_ros',
            executable='ballbot_target',
            name='ballbot_target',
            prefix=prefix,
            arguments=[LaunchConfiguration('task_name')],
            output='screen'
        )
    ])
