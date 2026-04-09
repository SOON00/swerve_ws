from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cbf_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goat',
    maintainer_email='shg2281@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cbf_filter_paper = cbf_sim.cbf_filter_paper:main',
            'odom_imu_publisher = cbf_sim.odom_imu_publisher:main',
            'swerve_drive_controller = cbf_sim.swerve_drive_controller:main',
            'teleop_keyboard = cbf_sim.teleop_keyboard:main',
            'cbf_filter_swerve = cbf_sim.cbf_filter_swerve:main',
            'cbf_filter_swerve_switching = cbf_sim.cbf_filter_swerve_switching:main',
            'cbf_filter_swerve_uturn = cbf_sim.cbf_filter_swerve_uturn:main',
            'cbf_maze = cbf_sim.cbf_maze:main',
            'nav_waypoints_sim = cbf_sim.nav_waypoints_sim:main',
            'nav_waypoints_real = cbf_sim.nav_waypoints_real:main',
            'cmdvel_serial = cbf_sim.cmdvel_serial:main',
            'odom_real_imu_publisher = cbf_sim.odom_real_imu_publisher:main',
            'cbf_single_odom = cbf_sim.cbf_single_odom:main',
            'heading_lock = cbf_sim.heading_lock:main',
        ],
    },
)
