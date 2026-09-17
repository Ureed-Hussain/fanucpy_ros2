from glob import glob

from setuptools import find_packages, setup


package_name = "fanucpy_ros2_task_planner"


setup(
    name=package_name,
    version="0.8.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (
            "share/" + package_name,
            ["package.xml", "README.md", "OLLAMA.md"],
        ),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Muhammad Ureed Hussain",
    maintainer_email="157709940+Ureed-Hussain@users.noreply.github.com",
    description=(
        "Validated natural-language task clients for the fanucpy ROS 2 driver."
    ),
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "fanucpy_prompt = "
            "fanucpy_ros2_task_planner.prompt_control:main",
            "fanucpy_ollama = "
            "fanucpy_ros2_task_planner.ollama_prompt:main",
        ],
    },
)
