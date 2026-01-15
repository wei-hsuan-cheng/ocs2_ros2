import os

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ocs2_mobile_manipulator_ros'),
                    'launch/ridgeback_ur5/manipulator_ridgeback_ur5_marker.launch.py',
                )
            ),
            launch_arguments={
                'solver': 'sqp',
            }.items(),
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
