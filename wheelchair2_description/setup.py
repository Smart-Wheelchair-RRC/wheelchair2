from setuptools import find_packages, setup
import os
from glob import glob

package_name = "wheelchair2_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("share", package_name, "urdf"),
            glob(os.path.join("urdf", "*.xacro")),
        ),
        (
            os.path.join("share", package_name, "urdf/gz_sim"),
            glob(os.path.join("urdf", "gz_sim", "*.xacro")),
        ),
        (
            os.path.join("share", package_name, "meshes"),
            glob(os.path.join("meshes", "*.[ds][at]*")),
        ),
        (
            os.path.join("share", package_name, "models"),
            glob(os.path.join(
                "models",  "model.[sc][do]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="radtop",
    maintainer_email="dhruvpotdar29@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ['pointcloud3d_to_2d=wheelchair2_description.pointcloud3d_to_2d:main'],
    },
)
