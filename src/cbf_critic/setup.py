from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cbf_critic'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goat',
    maintainer_email='shg2281@naver.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = cbf_critic.teleop_keyboard:main',
            'swerve_drive_controller = cbf_critic.swerve_drive_controller:main',
            'odom_imu_publisher = cbf_critic.odom_imu_publisher:main',
        ],
    },
)
