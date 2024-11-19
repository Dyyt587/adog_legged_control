from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true','false'],
            description='Use simulation (Gazebo) clock if true'
        ),
        
        DeclareLaunchArgument(
            name='deskewing',
            default_value='false',
            choices=['true','false'],
            description='Enable lidar deskewing'
        ),

        DeclareLaunchArgument(
            name='localize_only',
            default_value='true',
            choices=['true','false'],
            description='Localize only, do not change loaded map'
        ),

        DeclareLaunchArgument(
            name='restart_map',
            default_value='false',
            choices=['true','false'],
            description='Delete previous map and restart'
        ),

        DeclareLaunchArgument(
            name='use_rtabmapviz',
            default_value='true',
            choices=['true','false'],
            description='Start rtabmapviz node'
        ),

        DeclareLaunchArgument(
            name='rtabmap_log_level',
            default_value='INFO',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for rtabmap.'
        ),

        DeclareLaunchArgument(
            name='icp_odometry_log_level',
            default_value='INFO',
            choices=['ERROR', 'WARN', 'INFO', 'DEBUG'],
            description='Set logger level for icp_odometry. Can set to WARN to reduce message output from this node.'
        ),

        # Nodes to launch
        Node(
            package='rtabmap_odom', executable='icp_odometry', output='screen',
            parameters=[{
                'frame_id':'rslidar',
                'odom_frame_id':'odom',
                'wait_for_transform':0.3, # 0.2
                'expected_update_rate':15.0,
                'subscribe_odom_info': 'false',
                'deskewing':LaunchConfiguration('deskewing'),
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', '/rslidar_points'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw')
            ],
            arguments=[
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/VoxelSize', '0.1',
                'Icp/Epsilon', '0.001',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/MaxTranslation', '2',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.01',
                'Odom/ScanKeyFrameThr', '0.6',
                'OdomF2M/ScanSubtractRadius', '0.1',
                'OdomF2M/ScanMaxSize', '15000',
                'OdomF2M/BundleAdjustment', 'false',
                '--ros-args',
                '--log-level',
                [
                    TextSubstitution(text='icp_odometry:='),
                    LaunchConfiguration('icp_odometry_log_level'),
                ],
            ]
            ),
            
        Node(
            package='rtabmap_util', executable='point_cloud_assembler', output='screen',
            parameters=[{
                'max_clouds':10,
                'fixed_frame_id':'rslidar',
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('cloud', 'odom_filtered_input_scan'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw')
            ]),
            
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[{
                'frame_id':'rslidar',
                'subscribe_depth':True,
                'subscribe_rgb':True,
                'subscribe_rgbd':False,
                'subscribe_scan_cloud':True,
                'approx_sync':True, # False
                'wait_for_transform': 0.3, #0.2,
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'assembled_cloud'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/depth/image_raw')
            ],
            arguments=[
                'Mem/IncrementalMemory', 'true',
                'Mem/InitWMWithAllNodes', LaunchConfiguration('localize_only'),
                'RGBD/ProximityMaxGraphDepth', '0',
                'RGBD/ProximityPathMaxNeighbors', '1',
                'RGBD/AngularUpdate', '0.05',
                'RGBD/LinearUpdate', '0.05',
                'RGBD/CreateOccupancyGrid', 'false',
                'Mem/NotLinkedNodesKept', 'false',
                'Mem/STMSize', '90',
                'Mem/LaserScanNormalK', '20',
                'Reg/Strategy', '1',
                'Icp/VoxelSize', '0.1',
                'Icp/PointToPlaneK', '20',
                'Icp/PointToPlaneRadius', '0',
                'Icp/PointToPlane', 'true',
                'Icp/Iterations', '10',
                'Icp/Epsilon', '0.001',
                'Icp/MaxTranslation', '3',
                'Icp/MaxCorrespondenceDistance', '1',
                'Icp/Strategy', '1',
                'Icp/OutlierRatio', '0.7',
                'Icp/CorrespondenceRatio', '0.2',
                '--ros-args',
                '--log-level',
                [
                    TextSubstitution(text='rtabmap:='),
                    LaunchConfiguration('rtabmap_log_level'),
                ],
            ]), 
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=[{
                'frame_id':'rslidar',
                'odom_frame_id':'odom',
                'subscribe_depth':False,
                'subscribe_rgb':False,
                'subscribe_odom_info':True,
                'subscribe_scan_cloud':True,
                'approx_sync':True, # False
                'use_sim_time':LaunchConfiguration('use_sim_time'),
            }],
            remappings=[
                ('scan_cloud', 'odom_filtered_input_scan'),
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('depth/image', '/camera/aligned_depth_to_color/image_raw')

            ],
            condition=IfCondition(LaunchConfiguration('use_rtabmapviz'))
        ),
    ])