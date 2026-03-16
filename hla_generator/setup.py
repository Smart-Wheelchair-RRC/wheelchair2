from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'hla_generator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ducky',
    maintainer_email='syedazif321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'e_restapi_to_ros = hla_generator.restapi_to_ros:main',
            'e_create_semantics_for_map = hla_generator.create_semantics_for_map:main',
            'e_hla_navigate2pose = hla_generator.e_hla_navigate2pose:main',
        ],
    },
)
