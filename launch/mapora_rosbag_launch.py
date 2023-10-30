import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('mapora')

    mapora_rosbag_file_param = os.path.join(path_package, 'params/mapora_rosbag_params.yaml')
    mapora_rosbag_node = Node(
        package='mapora',
        executable='mapora_rosbag_exe',
        parameters=[mapora_rosbag_file_param]
    )


    return launch.LaunchDescription(
        [mapora_rosbag_node])
