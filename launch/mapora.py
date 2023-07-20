import os

from ament_index_python import get_package_share_directory

import launch

from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description."""
    path_package = get_package_share_directory('kaist_mapper')

    path_file_param = os.path.join(path_package, 'param/test_params.yaml')

    name_launch_arg = 'param_file_kaist_mapper'

    launch_arg = DeclareLaunchArgument(
        name_launch_arg,
        default_value=path_file_param,
        description='Path to params file.'
    )

    node = Node(
        package='kaist_mapper',
        executable='kaist_mapper_exe',
        namespace='',
        output='screen',
        parameters=[
            LaunchConfiguration(name_launch_arg)
        ]
    )

    return launch.LaunchDescription([launch_arg,
                                     node])
