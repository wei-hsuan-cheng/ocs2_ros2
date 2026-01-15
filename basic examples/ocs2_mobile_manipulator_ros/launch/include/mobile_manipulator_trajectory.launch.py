import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Mirror mobile_manipulator_marker.launch.py arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    urdf_arg = DeclareLaunchArgument('urdfFile', default_value='/tmp/ocs2_auto_generated')
    task_arg = DeclareLaunchArgument('taskFile', default_value='/tmp/ocs2_auto_generated')
    lib_arg = DeclareLaunchArgument('libFolder', default_value='/tmp/ocs2_auto_generated')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    enable_joystick_arg = DeclareLaunchArgument('enableJoystick', default_value='false')
    enable_auto_pos_arg = DeclareLaunchArgument('enableAutoPosition', default_value='false')

    # Trajectory-specific arguments
    publish_rate_arg = DeclareLaunchArgument('publishRate', default_value='100.0')
    horizon_arg = DeclareLaunchArgument('horizon', default_value='1.0')
    dt_arg = DeclareLaunchArgument('dt', default_value='0.01')
    amplitude_arg = DeclareLaunchArgument('amplitude', default_value='0.20')
    frequency_arg = DeclareLaunchArgument('frequency', default_value='0.0')
    # Plane normal (unit axis) for the trajectory plane
    axis_x_arg = DeclareLaunchArgument('axisX', default_value='0.0')
    axis_y_arg = DeclareLaunchArgument('axisY', default_value='0.0')
    axis_z_arg = DeclareLaunchArgument('axisZ', default_value='1.0')

    # Visualization include (same as original)
    visualize_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ocs2_mobile_manipulator_ros'), 'launch/include/visualize.launch.py')
        ),
        launch_arguments={
            'urdfFile': LaunchConfiguration('urdfFile'),
            'rviz': LaunchConfiguration('rviz')
        }.items()
    )

    # MPC node (same as marker)
    mpc_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_mpc_node',
        name='mobile_manipulator_mpc',
        condition=UnlessCondition(LaunchConfiguration('debug')),
        output='screen',
        parameters=[
            {'taskFile': LaunchConfiguration('taskFile')},
            {'urdfFile': LaunchConfiguration('urdfFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
        ]
    )

    # Dummy MRT node (same as marker)
    dummy_mrt_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_dummy_mrt_node',
        name='mobile_manipulator_dummy_mrt_node',
        output='screen',
        parameters=[
            {'taskFile': LaunchConfiguration('taskFile')},
            {'urdfFile': LaunchConfiguration('urdfFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
        ]
    )

    # Replace interactive marker target with trajectory target node
    traj_target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_trajectory_target',
        name='mobile_manipulator_trajectory_target',
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[
            {'taskFile': LaunchConfiguration('taskFile')},
            {'urdfFile': LaunchConfiguration('urdfFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
            {'robotName': 'mobile_manipulator'},
            {'publishRate': LaunchConfiguration('publishRate')},
            {'horizon': LaunchConfiguration('horizon')},
            {'dt': LaunchConfiguration('dt')},
            {'amplitude': LaunchConfiguration('amplitude')},
            {'frequency': LaunchConfiguration('frequency')},
            {'axisX': LaunchConfiguration('axisX')},
            {'axisY': LaunchConfiguration('axisY')},
            {'axisZ': LaunchConfiguration('axisZ')},
        ]
    )

    return LaunchDescription([
        rviz_arg,
        urdf_arg,
        task_arg,
        lib_arg,
        debug_arg,
        enable_joystick_arg,
        enable_auto_pos_arg,
        publish_rate_arg,
        horizon_arg,
        dt_arg,
        amplitude_arg,
        frequency_arg,
        axis_x_arg,
        axis_y_arg,
        axis_z_arg,
        visualize_include,
        mpc_node,
        dummy_mrt_node,
        traj_target_node,
    ])
