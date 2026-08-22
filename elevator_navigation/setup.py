import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'elevator_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Team',
    maintainer_email='robot@campus.edu',
    description='Python multi-floor topological navigation package for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_manager_node = elevator_navigation.state_manager_node:main',
            'elevator_floor_detection_manual = elevator_navigation.elevator_floor_detection_manual:main',
        ],
    },
)