import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'swerve_bot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package_name), 'launch', 'rsp.launch.py')
        ]),
        launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
    )


    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('swerve_bot'), 'config', 'controller.yaml']
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    swerve_steering_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_steering_controller', '--param-file', robot_controllers],
        output='screen'
    )

    swerve_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['swerve_velocity_controller', '--param-file', robot_controllers],
        output='screen'
    )

    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'mapping.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    odom_publisher_node = Node(
        package='swerve_bot',
        executable='odom_real_imu_publisher',
        output='screen'
    )

    return LaunchDescription([
        rsp,
        joint_state_broadcaster_spawner,
        swerve_steering_controller_spawner,
        swerve_velocity_controller_spawner,
        node_rviz,
        odom_publisher_node
    ])
