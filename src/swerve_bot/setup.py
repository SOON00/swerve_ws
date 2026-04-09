from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'swerve_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goat',
    maintainer_email='shg2281@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = swerve_bot.teleop_keyboard:main',
            'swerve_drive_controller = swerve_bot.swerve_drive_controller:main',
            'odom_publisher = swerve_bot.odom_publisher:main',
            'odom_imu_publisher = swerve_bot.odom_imu_publisher:main',
            'odom_real_publisher = swerve_bot.odom_real_publisher:main',
            'odom_real_imu_publisher = swerve_bot.odom_real_imu_publisher:main',
            'lidar_filter = swerve_bot.lidar_filter:main',
            'cmdvel_serial = swerve_bot.cmdvel_serial:main',
            'initial_pose_publisher = swerve_bot.initial_pose_publisher:main'
        ],
    },
)
