import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch_ros.actions.Node(
            package='convex_plane_decomposition_ros',
            executable='save_elevation_map',
            name='save_elevation_map',
            prefix= "",
            output='screen',
            parameters=[
                {
                    'frequency': '0.1'
                },
                {
                    'elevation_topic': '/elevation_mapping/elevation_map_raw'
                },
                {
                    'height_layer': 'elevation'
                },
                {
                    'imageName': get_package_share_directory('convex_plane_decomposition_ros') + '/data/elevationMap'
                }
            ]
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
