import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Mirror mobile_manipulator.launch.py arguments
    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    urdf_arg = DeclareLaunchArgument('urdfFile', default_value='/tmp/ocs2_auto_generated')
    task_arg = DeclareLaunchArgument('taskFile', default_value='/tmp/ocs2_auto_generated')
    lib_arg = DeclareLaunchArgument('libFolder', default_value='/tmp/ocs2_auto_generated')
    debug_arg = DeclareLaunchArgument('debug', default_value='false')
    enable_joystick_arg = DeclareLaunchArgument('enableJoystick', default_value='false')
    enable_auto_pos_arg = DeclareLaunchArgument('enableAutoPosition', default_value='false')

    # Twist-specific arguments
    publish_rate_arg = DeclareLaunchArgument('publishRate', default_value='50.0')
    horizon_arg = DeclareLaunchArgument('horizon', default_value='1.0')
    dt_arg = DeclareLaunchArgument('dt', default_value='0.02')
    twist_world_arg = DeclareLaunchArgument('twistInWorld', default_value='true')
    vx_arg = DeclareLaunchArgument('vx', default_value='0.0')
    vy_arg = DeclareLaunchArgument('vy', default_value='0.0')
    vz_arg = DeclareLaunchArgument('vz', default_value='0.0')
    wx_arg = DeclareLaunchArgument('wx', default_value='0.0')
    wy_arg = DeclareLaunchArgument('wy', default_value='0.0')
    wz_arg = DeclareLaunchArgument('wz', default_value='0.0')

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

    # MPC node (same as original)
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

    # Dummy MRT node (same as original)
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

    # Replace interactive marker target with twist target node
    twist_target_node = Node(
        package='ocs2_mobile_manipulator_ros',
        executable='mobile_manipulator_twist_target',
        name='mobile_manipulator_twist_target',
        condition=IfCondition(LaunchConfiguration('rviz')),  # keep same gating behavior
        output='screen',
        parameters=[
            {'taskFile': LaunchConfiguration('taskFile')},
            {'urdfFile': LaunchConfiguration('urdfFile')},
            {'libFolder': LaunchConfiguration('libFolder')},
            {'robotName': 'mobile_manipulator'},
            {'publishRate': LaunchConfiguration('publishRate')},
            {'horizon': LaunchConfiguration('horizon')},
            {'dt': LaunchConfiguration('dt')},
            {'twistInWorld': LaunchConfiguration('twistInWorld')},
            {'vx': LaunchConfiguration('vx')},
            {'vy': LaunchConfiguration('vy')},
            {'vz': LaunchConfiguration('vz')},
            {'wx': LaunchConfiguration('wx')},
            {'wy': LaunchConfiguration('wy')},
            {'wz': LaunchConfiguration('wz')},
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
        twist_world_arg,
        vx_arg,
        vy_arg,
        vz_arg,
        wx_arg,
        wy_arg,
        wz_arg,
        visualize_include,
        mpc_node,
        dummy_mrt_node,
        twist_target_node,
    ])
