import os

from click import launch

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    load_gripper_parameter_name = 'load_gripper'
    load_gripper = LaunchConfiguration(load_gripper_parameter_name)

    franka_xacro_file = os.path.join(get_package_share_directory('franka_gazebo'), 'models',
                                     'panda_arm.urdf')
    robot_description = Command(
        [FindExecutable(name='xacro'), ' ', franka_xacro_file, ' hand:=', load_gripper])

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'verbose': 'true', 'debug': 'true'}.items(),
    )
    
    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}

    params = {'robot_description': robot_description}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'franka'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
    #          'joint_trajectory_controller'],
    #     output='screen'
    # )

    load_hqp_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'hqp_controller'],
        output='screen'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            load_gripper_parameter_name,
            default_value='true',
            description='Use Franka Gripper as end-effector if true. Robot is loaded without '
                        'end-effector otherwise'),
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui'
        # ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_hqp_controller],
            )
        ),

        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_trajectory_controller,
        #         on_exit=[load_joint_position_controller],
        #     )
        # ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
    ])
