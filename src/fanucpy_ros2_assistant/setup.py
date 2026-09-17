from glob import glob
import os

from setuptools import find_packages, setup


package_name = "fanucpy_ros2_assistant"


setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml") + glob("config/*.json"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),
        (
            os.path.join("share", package_name, "tasks"),
            glob("tasks/*.json"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Muhammad Ureed Hussain",
    maintainer_email="157709940+Ureed-Hussain@users.noreply.github.com",
    description=(
        "Unified conversational FANUC tasks and guarded vision workflows."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "fanucpy_assistant = fanucpy_ros2_assistant.assistant:main",
        ],
    },
)
