import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config_file = get_package_share_directory('convex_plane_decomposition_ros') + "/rviz/config_demo.rviz"
    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='datafile',
            default_value=os.path.join(get_package_share_directory(
                'convex_plane_decomposition_ros'), 'data', 'terrain.png')  
        ),
        launch.actions.DeclareLaunchArgument(
            name='max_height',
            default_value='1.0'
        ),
        launch_ros.actions.Node(
            package='grid_map_demos',
            executable='image_publisher.py',
            name='image_publisher',
            output='screen',
            parameters=[
                {
                    'image_path': launch.substitutions.LaunchConfiguration('datafile')
                },
                {
                    'topic': 'image'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='grid_map_demos',
            executable='image_to_gridmap_demo',
            name='image_to_gridmap_demo',
            output='screen',
            parameters=[
                {
                    'image_topic': '/image'
                },
                {
                    'min_height': 0.0
                },
                {
                    'max_height': launch.substitutions.LaunchConfiguration('max_height')
                },
                {
                    'resolution': 0.04
                }
            ]
        ),
        launch_ros.actions.Node(
            package='convex_plane_decomposition_ros',
            executable='convex_plane_decomposition_ros_add_noise',
            name='convex_plane_decomposition_ros_add_noise',
            output='screen',
            parameters=[
                {
                    'noiseGauss': 0.01
                },
                {
                    'noiseUniform': 0.01
                },
                {
                    'outlier_percentage': 5.0
                },
                {
                    'blur': False
                },
                {
                    'frequency': 30.0
                },
                {
                    'elevation_topic_in': '/image_to_gridmap_demo/grid_map'
                },
                {
                    'elevation_topic_out': '/elevation_mapping/elevation_map_raw'
                },
                {
                    'height_layer': 'elevation'
                },
                {
                    'imageName': get_package_share_directory('convex_plane_decomposition_ros') + '/data/elevationMap'
                }
            ]
        ),
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map2odom_broadcaster',
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ),
        launch_ros.actions.Node(
            package='convex_plane_decomposition_ros',
            executable='convex_plane_decomposition_ros_approximation_demo_node',
            name='convex_plane_decomposition_ros_approximation_demo_node',
            prefix= "",
            output='screen'
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_config_file]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'convex_plane_decomposition_ros'), 'launch/convex_plane_decomposition.launch.py')
            ),
            launch_arguments={
                'node_parameter_file': get_package_share_directory('convex_plane_decomposition_ros') + '/config/demo_node.yaml'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
