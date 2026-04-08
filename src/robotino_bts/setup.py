from setuptools import setup

package_name = "robotino_bts"

setup(
    name=package_name,
    version="0.0.1",
    packages=[
        package_name,
        f"{package_name}.behaviors",
        f"{package_name}.trees",
    ],
    package_dir={"": "."},
    data_files=[
        # Required so ROS knows this is a package
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),

        # Install package.xml
        ("share/" + package_name, ["package.xml"]),

        # Install config files
        ("share/" + package_name + "/config",
         ["robotino_bts/config/known_locations.yaml"]),

        # Install launch files
        ("share/" + package_name + "/launch",
         ["launch/bt_task.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="oscar",
    maintainer_email="oscar@example.com",
    description="Behavior trees for Robotino navigation and perception",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_manager = robotino_bts.task_manager:main",
        ],
    },
)
