import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'cbf_sim'

    # ================= Robot State Publisher =================
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(package_name),
                'launch',
                'rsp.launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': 'false'
        }.items()
    )

    # ================= Controllers =================
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare('cbf_sim'), 'config', 'controller.yaml']
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

    swerve_drive_controller_node = Node(
        package='cbf_sim',
        executable='swerve_drive_controller',
        output='screen'
    )

    # ================= RViz =================
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'sim_nav_real1.rviz'
    )

    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # ================= Odom + IMU =================
    odom_real_imu_publisher_node = Node(
        package='cbf_sim',
        executable='odom_real_imu_publisher',
        output='screen'
    )

    # # ================= twist_mux =================
    # cmd_vel_mux_node = Node(
    #     package='cbf_sim',
    #     executable='cmd_vel_mux',
    #     output='screen'
    # )
    
    cmdvel_serial_node = Node(
        package='cbf_sim',
        executable='cmdvel_serial',
        output='screen'
    )

    # ================= Launch =================
    return LaunchDescription([
        rsp,
        joint_state_broadcaster_spawner,
        swerve_steering_controller_spawner,
        swerve_velocity_controller_spawner,
        swerve_drive_controller_node,
        # cmd_vel_mux_node,
        # cmdvel_serial_node,
        node_rviz,
        odom_real_imu_publisher_node,
    ])
