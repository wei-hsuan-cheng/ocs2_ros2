import os

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_default = os.path.join(
        get_package_share_directory('ocs2_robotic_assets'),
        'resources/mobile_manipulator/ur5/urdf/ur5.urdf')
    task_default = os.path.join(
        get_package_share_directory('ocs2_mobile_manipulator'),
        'config/ur5/task.info')
    lib_default = '/tmp/ocs2_auto_generated/ur5'

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    solver_arg = DeclareLaunchArgument('solver', default_value='ddp')
    urdf_arg = DeclareLaunchArgument('urdfFile', default_value=urdf_default)
    task_arg = DeclareLaunchArgument('taskFile', default_value=task_default)
    lib_arg = DeclareLaunchArgument('libFolder', default_value=lib_default)
    marker_publish_rate_arg = DeclareLaunchArgument('markerPublishRate', default_value='10.0')
    enable_joystick_arg = DeclareLaunchArgument('enableJoystick', default_value='false')
    enable_auto_position_arg = DeclareLaunchArgument('enableAutoPosition', default_value='false')

    include_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ocs2_mobile_manipulator_ros'),
                         'launch/include/mobile_manipulator_marker.launch.py')
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'debug': LaunchConfiguration('debug'),
            'solver': LaunchConfiguration('solver'),
            'urdfFile': LaunchConfiguration('urdfFile'),
            'taskFile': LaunchConfiguration('taskFile'),
            'libFolder': LaunchConfiguration('libFolder'),
            'markerPublishRate': LaunchConfiguration('markerPublishRate'),
            'enableJoystick': LaunchConfiguration('enableJoystick'),
            'enableAutoPosition': LaunchConfiguration('enableAutoPosition'),
        }.items()
    )

    return launch.LaunchDescription([
        rviz_arg,
        debug_arg,
        solver_arg,
        urdf_arg,
        task_arg,
        lib_arg,
        marker_publish_rate_arg,
        enable_joystick_arg,
        enable_auto_position_arg,
        include_all,
    ])


if __name__ == '__main__':
    generate_launch_description()

