from setuptools import find_packages, setup


package_name = "fanucpy_ros2_driver"


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
    ],
    install_requires=[
        "setuptools",
        "fanucpy>=0.1.14,<0.2.0",
    ],
    zip_safe=False,
    maintainer="Muhammad Ureed Hussain",
    maintainer_email="157709940+Ureed-Hussain@users.noreply.github.com",
    description="Single-connection ROS 2 driver for fanucpy/MAPPDK.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "fanucpy_driver = fanucpy_ros2_driver.driver_node:main",
        ],
    },
)
