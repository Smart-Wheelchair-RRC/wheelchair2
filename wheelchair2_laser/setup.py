from setuptools import setup
import os
import glob

package_name = "wheelchair2_laser"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],  # Ensure this matches your Python module
    install_requires=["setuptools"],
    zip_safe=True,
    description="Wheelchair2 Laser ROS2 package",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "livox_custom_msg_to_pointcloud2 = wheelchair2_laser.livox_custom_msg_to_pointcloud2:main",
        ],
    },
    data_files=[
        (os.path.join("share", package_name), ["package.xml"]),
        # (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        # (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
)
