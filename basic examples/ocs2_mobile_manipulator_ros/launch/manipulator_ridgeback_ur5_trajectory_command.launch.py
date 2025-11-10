import os

import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_default = os.path.join(
        get_package_share_directory('ocs2_robotic_assets'),
        'resources/mobile_manipulator/ridgeback_ur5/urdf/ridgeback_ur5.urdf')
    task_default = os.path.join(
        get_package_share_directory('ocs2_mobile_manipulator'),
        'config/ridgeback_ur5/task.info')
    lib_default = os.path.join(
        get_package_share_directory('ocs2_mobile_manipulator'),
        'auto_generated/ridgeback_ur5')

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    urdf_arg = DeclareLaunchArgument('urdfFile', default_value=urdf_default)
    task_arg = DeclareLaunchArgument('taskFile', default_value=task_default)
    lib_arg = DeclareLaunchArgument('libFolder', default_value=lib_default)

    # Trajectory args
    publish_rate_arg = DeclareLaunchArgument('publishRate', default_value='100.0')
    horizon_arg = DeclareLaunchArgument('horizon', default_value='5.0')
    dt_arg = DeclareLaunchArgument('dt', default_value='0.01')
    amplitude_arg = DeclareLaunchArgument('amplitude', default_value='0.20')
    frequency_arg = DeclareLaunchArgument('frequency', default_value='0.2')
    # Plane normal (unit axis) for the trajectory plane
    axis_x_arg = DeclareLaunchArgument('axisX', default_value='1.0')
    axis_y_arg = DeclareLaunchArgument('axisY', default_value='0.0')
    axis_z_arg = DeclareLaunchArgument('axisZ', default_value='1.0')

    include_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ocs2_mobile_manipulator_ros'),
                         'launch/include/mobile_manipulator_trajectory.launch.py')
        ),
        launch_arguments={
            'rviz': LaunchConfiguration('rviz'),
            'debug': LaunchConfiguration('debug'),
            'urdfFile': LaunchConfiguration('urdfFile'),
            'taskFile': LaunchConfiguration('taskFile'),
            'libFolder': LaunchConfiguration('libFolder'),
            'publishRate': LaunchConfiguration('publishRate'),
            'horizon': LaunchConfiguration('horizon'),
            'dt': LaunchConfiguration('dt'),
            'amplitude': LaunchConfiguration('amplitude'),
            'frequency': LaunchConfiguration('frequency'),
            'axisX': LaunchConfiguration('axisX'),
            'axisY': LaunchConfiguration('axisY'),
            'axisZ': LaunchConfiguration('axisZ'),
        }.items()
    )

    return launch.LaunchDescription([
        rviz_arg,
        debug_arg,
        urdf_arg,
        task_arg,
        lib_arg,
        publish_rate_arg,
        horizon_arg,
        dt_arg,
        amplitude_arg,
        frequency_arg,
        axis_x_arg,
        axis_y_arg,
        axis_z_arg,
        include_all,
    ])


if __name__ == '__main__':
    generate_launch_description()

