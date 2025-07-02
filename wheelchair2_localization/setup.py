from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wheelchair2_localization'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eccentric Orange',
    maintainer_email='eccentric.orange2@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'invert_livox_data = wheelchair2_localization.invert_livox_data:main',
            'visualize_pcd_map = wheelchair2_localization.vizualize_pcd_map:main',
        ],
    },
)
