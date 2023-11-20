from setuptools import setup
import os
from glob import glob

package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
		(os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Simone Giampà',
    maintainer_email='simone.giampa@mail.polimi.it',
    description='ROS2 Aruco marker detection and pose estimation wrapper',
    license='AIRLAB',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = ros2_aruco.aruco_node:main',
			'cam_feed = ros2_aruco.cam_feed:main',
            'aruco_generate_marker = ros2_aruco.aruco_generate_marker:main'
        ],
    },
)
