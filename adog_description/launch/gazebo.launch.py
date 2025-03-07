import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import ament_index_python
from launch_ros.actions import Node
import xacro

from ament_index_python.packages import get_package_share_directory
 
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import TimerAction

def generate_launch_description():
    
    # this name has to match the robot name in the Xacro file
    robotXacroName='adog1'
    
    # this is the name of our package, at the same time this is the name of the 
    # folder that will be used to define the paths
    package_name = 'unitree_go2_description'
    package_description = 'go2_description'
    rvizFileRelativePath = 'config/gazebo.rviz'
    # this is a relative path to the xacro file defining the model
    modelFileRelativePath = 'xacro/robot.xacro'
    # this is a relative path to the Gazebo world file
    worldFileRelativePath = 'world/empty_world.world'
    
    # this is the absolute path to the model
    pathModelFile = os.path.join(get_package_share_directory(package_name),modelFileRelativePath)

    # this is the absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(package_name),worldFileRelativePath)
    #pathWorldFile = os.path.join("gazebo_maps/3d_maze/eazy_maze.world")
    # get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    pkg_share = os.pathsep + os.path.join(ament_index_python.get_package_prefix(package_name), 'share')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += pkg_share
    else:
        os.environ['GAZEBO_MODEL_PATH'] = pkg_share
    # this is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),
                                                                       'launch','gazebo.launch.py'))
    
    # this is the launch description   
    gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'world': pathWorldFile,'verbose': "false",'pause':"false"}.items())
    #gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch)
    
    # here, we create a gazebo_ros Node 
    spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py',
                          arguments=['-topic','robot_description','-entity', robotXacroName,
                                     '-x', '0',
                                   '-y', '0',
                                   '-z', '0.30',
                                   '-R', '0.0',
                                   '-P', '-0.0',
                                   '-Y', '0'],output='screen')
    
    # Robot State Publisher Node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription,
        'use_sim_time': True}] 
    )

    ros_control_1 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','joint_state_broadcaster'],
        output='screen')
    ros_control_2 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','imu_sensor_broadcaster'],
        output='screen')
   
    ros_control_3 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_effort_controller'],
        output='screen'
    )
    ros_control_pd =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'leg_pd_controller'],
        output='screen'
    )
    ros_control_4_1 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fl_foot_force_sensor_broadcaster'],
        output='screen'
    )
    ros_control_4_2 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fr_foot_force_sensor_broadcaster'],
        output='screen'
    )
    ros_control_4_3 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'rl_foot_force_sensor_broadcaster'],
        output='screen'
    )
    ros_control_4_4 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'rr_foot_force_sensor_broadcaster'],
        output='screen'
    )

    ros_control_5 =    ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'ocs2_quadruped_controller'],
        output='screen'
    )
    # spawner_script = Node(
    #     package='controller_manager',
    #     executable='ros2_control_node',
    #     arguments=[
    #         '-p','urdf_file': os.path.join(get_package_share_directory(package_description), 'urdf',
    #                                               'robot.urdf'),
    #                     'task_file': os.path.join(get_package_share_directory(package_description), 'config', 'ocs2',
    #                                               'task.info'),
    #                     'reference_file': os.path.join(get_package_share_directory(package_description), 'config',
    #                                                    'ocs2', 'reference.info'),
    #                     'gait_file': os.path.join(get_package_share_directory(package_description), 'config',
    #                                               'ocs2', 'gait.info')
                    
    #     ],
    #     output='both')
    
    ocs2_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_quadruped_controller", "--controller-manager", "/controller_manager",'--ros-args',
                   os.path.join('urdf_file:=',get_package_share_directory(package_description), 'urdf',
                                                  'robot.urdf'),
                        os.path.join('task_file:=',get_package_share_directory(package_description), 'config', 'ocs2',
                                                  'task.info'),
                        os.path.join('reference_file:=',get_package_share_directory(package_description), 'config',
                                                       'ocs2', 'reference.info'),
                        os.path.join('gait_file:=',get_package_share_directory(package_description), 'config',
                                                  'ocs2', 'gait.info')
                    ]
    )

	# 获取功能包路径（注意，这个路径是在工作空间的install文件夹里
    pkg_description = get_package_share_directory(package_name)
	# 声明文件路径，os.path.join将口号内的str用\连接，组成路径
    robot_description_xacro = os.path.join(pkg_description, modelFileRelativePath)
    
    rviz_config_path = os.path.join(pkg_description,rvizFileRelativePath)
    
    # 发布关节状态，无关节调节可视化窗口
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        )
    # 发布关节状态，有关节调节可视化窗口
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        )
    # 启动rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz_config_path]
    )
    # ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
    #     output='screen'
    # ),
 
    # ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_effort_controller'],
    #     output='screen'
    # ),
    # ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'test_force_torque_sensor_broadcaster'],
    #     output='screen'
    # ),
    # here we create an empty launch description object
    delayed_spawn_robot = TimerAction(
        period=2.0,
        actions=[spawnModelNode]
    )


    launchDescriptionObject = LaunchDescription()
     
    # we add gazeboLaunch 
    launchDescriptionObject.add_action(gazeboLaunch)
    #launchDescriptionObject.add_action(spawner_script)
    
    #launchDescriptionObject.add_action(delay_robot_after_gazebo)
    
    # we add the two nodes
    launchDescriptionObject.add_action(delayed_spawn_robot)
    # launchDescriptionObject.add_action(append_environment)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    launchDescriptionObject.add_action(ros_control_1)
    launchDescriptionObject.add_action(ros_control_2)
    launchDescriptionObject.add_action(ros_control_pd)
    #launchDescriptionObject.add_action(ros_control_3)

    launchDescriptionObject.add_action(ros_control_4_1)
    launchDescriptionObject.add_action(ros_control_4_2)
    launchDescriptionObject.add_action(ros_control_4_3)
    launchDescriptionObject.add_action(ros_control_4_4)
    #launchDescriptionObject.add_action(ros_control_5)
    launchDescriptionObject.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ros_control_pd,
                on_exit=[ros_control_5],
            )
        ))

    # # launchDescriptionObject.add_action(joint_state_publisher_gui_node)
    launchDescriptionObject.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ros_control_5,
                on_exit=[rviz_node],
            )
        ))
    

    return launchDescriptionObject