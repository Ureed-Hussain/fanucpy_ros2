from setuptools import find_packages, setup


package_name = "fanucpy_ros2_examples"


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
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Muhammad Ureed Hussain",
    maintainer_email="157709940+Ureed-Hussain@users.noreply.github.com",
    description="Student-facing examples for the fanucpy ROS 2 driver.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "fanucpy_controller_tools = "
            "fanucpy_ros2_examples.controller_tools:main",
            "fanucpy_keyboard_teleop = "
            "fanucpy_ros2_examples.keyboard_teleop:main",
        ],
    },
)
