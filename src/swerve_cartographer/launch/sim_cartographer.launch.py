import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    cartographer_pkg = get_package_share_directory('swerve_cartographer')

    cartographer_config_dir = LaunchConfiguration(
        'cartographer_config_dir',
        default=os.path.join(cartographer_pkg, 'config')
    )

    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='swerve_2d.lua'
    )

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename
        ]
    )

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('swerve_cartographer'),
                'launch',
                'occupancy_grid.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'resolution': resolution,
            'publish_period_sec': publish_period_sec
        }.items()
    )
        
    robot_pkg = 'swerve_bot'

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(robot_pkg),
                'launch',
                'rsp.launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'use_ros2_control': 'true'
        }.items()
    )

    gazebo_params = os.path.join(
        get_package_share_directory(robot_pkg),
        'config',
        'gazebo_params.yaml'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'swerve_bot',
            '-x', '-4.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '1.57'
        ],
        output='screen'
    )

    controller_yaml = PathJoinSubstitution(
        [FindPackageShare(robot_pkg), 'config', 'controller.yaml']
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_steering_controller',
            '--param-file', controller_yaml
        ],
        output='screen'
    )

    velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'swerve_velocity_controller',
            '--param-file', controller_yaml
        ],
        output='screen'
    )

    swerve_drive_controller = Node(
        package='swerve_bot',
        executable='swerve_drive_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    odom_imu_publisher = Node(
        package='swerve_bot',
        executable='odom_imu_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_config = os.path.join(
        cartographer_pkg,
        'rviz',
        'swerve_cartographer.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true'
        ),
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename
        ),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec
        ),

        rsp,
        gazebo,
        spawn_entity,
        joint_state_broadcaster,
        steering_controller,
        velocity_controller,
        swerve_drive_controller,
        odom_imu_publisher,
        cartographer_node,
        occupancy_grid,
        rviz,
    ])
