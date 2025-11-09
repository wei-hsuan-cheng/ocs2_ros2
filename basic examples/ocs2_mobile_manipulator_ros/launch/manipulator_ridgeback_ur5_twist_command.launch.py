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

    # Twist args (surfaced at top)
    publish_rate_arg = DeclareLaunchArgument('publishRate', default_value='100.0')
    horizon_arg = DeclareLaunchArgument('horizon', default_value='1.0')
    dt_arg = DeclareLaunchArgument('dt', default_value='0.01')
    twist_world_arg = DeclareLaunchArgument('twistInWorld', default_value='false')
    vx_arg = DeclareLaunchArgument('vx', default_value='0.0')
    vy_arg = DeclareLaunchArgument('vy', default_value='0.0')
    vz_arg = DeclareLaunchArgument('vz', default_value='0.0')
    wx_arg = DeclareLaunchArgument('wx', default_value='-0.1')
    wy_arg = DeclareLaunchArgument('wy', default_value='0.0')
    wz_arg = DeclareLaunchArgument('wz', default_value='0.0')

    include_all = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ocs2_mobile_manipulator_ros'),
                         'launch/include/mobile_manipulator_twist_command.launch.py')
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
            'twistInWorld': LaunchConfiguration('twistInWorld'),
            'vx': LaunchConfiguration('vx'),
            'vy': LaunchConfiguration('vy'),
            'vz': LaunchConfiguration('vz'),
            'wx': LaunchConfiguration('wx'),
            'wy': LaunchConfiguration('wy'),
            'wz': LaunchConfiguration('wz'),
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
        twist_world_arg,
        vx_arg,
        vy_arg,
        vz_arg,
        wx_arg,
        wy_arg,
        wz_arg,
        include_all,
    ])


if __name__ == '__main__':
    generate_launch_description()

