from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'wheelchair2_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob(os.path.join("worlds", "*.[sw][do]*")),
        ),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), []),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "params"),
            glob(os.path.join("params", "*.yaml")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.xacro")),
        ),
        (
            os.path.join("share", package_name, "urdf/gz_sim"),
            glob(os.path.join("urdf", "gz_sim", "*.xacro")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wheelchair2',
    maintainer_email='tarun.ramak@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
