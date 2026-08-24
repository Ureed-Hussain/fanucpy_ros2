from glob import glob
import os

from setuptools import find_packages, setup


package_name = "fanucpy_ros2_trajectory_controller"


setup(
    name=package_name,
    version="0.8.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Muhammad Ureed Hussain",
    maintainer_email="157709940+Ureed-Hussain@users.noreply.github.com",
    description="MoveIt-compatible FollowJointTrajectory server for fanucpy ROS 2.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
)
