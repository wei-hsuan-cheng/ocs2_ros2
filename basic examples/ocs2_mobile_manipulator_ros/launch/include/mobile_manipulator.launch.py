# pylint: disable=too-many-return-statements
import os
import re
import shutil
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def is_wsl():
    try:
        with open('/proc/version', 'r') as f:
            version_info = f.read().lower()
            return 'microsoft' in version_info or 'wsl' in version_info
    except FileNotFoundError:
        return False


def detect_terminal_prefix():
    """Select a terminal prefix if a GUI terminal is available; otherwise none.

    In headless/containers (no DISPLAY), return empty prefix so child nodes run
    directly. This avoids xterm failures that prevent nodes from starting.
    """
    # Headless check
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
    # For stability across headless/container runs, avoid wrapping nodes in a terminal
    # prefix. This ensures critical nodes (e.g., dummy MRT) are not tied to a GUI shell.
    prefix = ""

    return LaunchDescription([
        DeclareLaunchArgument(
            name='rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='urdfFile',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='taskFile',
            default_value=''
        ),
        DeclareLaunchArgument(
            name='libFolder',
            default_value='/tmp/ocs2_auto_generated'
        ),
        DeclareLaunchArgument(
            name='debug',
            default_value='false'
        ),
        DeclareLaunchArgument(
            name='solver',
            default_value='ddp',
            description='MPC solver: ddp or sqp'
        ),
        DeclareLaunchArgument(
            name='enableJoystick',
            default_value='false',
            description='Whether to enable joystick control'
        ),
        DeclareLaunchArgument(
            name='enableAutoPosition',
            default_value='false',
            description='Whether to enable automatic marker position updates'
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
            ),
            launch_arguments={
                'urdfFile': LaunchConfiguration('urdfFile'),
                'rviz': LaunchConfiguration('rviz')
            }.items()
        ),
        # Node(
        #     package='ocs2_mobile_manipulator_ros',
        #     executable='mobile_manipulator_mpc',
        #     name='mobile_manipulator_mpc',
        #     prefix=prefix,
        #     condition=IfCondition(LaunchConfiguration("debug")),
        #     output='screen',
        #     parameters=[
        #         {
        #             'taskFile': LaunchConfiguration('taskFile')
        #         },
        #         {
        #             'urdfFile': LaunchConfiguration('urdfFile')
        #         },
        #         {
        #             'libFolder': LaunchConfiguration('libFolder')
        #         }
        #     ]
        # ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_mpc_node',
            name='mobile_manipulator_mpc',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('debug'), "' == 'false' and '", LaunchConfiguration('solver'), "' != 'sqp'"])),
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_sqp_mpc_node',
            name='mobile_manipulator_mpc',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('debug'), "' == 'false' and '", LaunchConfiguration('solver'), "' == 'sqp'"])),
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_dummy_mrt_node',
            name='mobile_manipulator_dummy_mrt_node',
            prefix=prefix,
            output='screen',
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                }
            ]
        ),
        Node(
            package='ocs2_mobile_manipulator_ros',
            executable='mobile_manipulator_marker_target',
            name='mobile_manipulator_marker_target',
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[
                {
                    'taskFile': LaunchConfiguration('taskFile')
                },
                {
                    'urdfFile': LaunchConfiguration('urdfFile')
                },
                {
                    'libFolder': LaunchConfiguration('libFolder')
                },
                {
                    'enableJoystick': LaunchConfiguration('enableJoystick')
                },
                {
                    'enableAutoPosition': LaunchConfiguration('enableAutoPosition')
                },
            ],
            output='screen',
        )
    ])
