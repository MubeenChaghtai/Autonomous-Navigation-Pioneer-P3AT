from glob import glob

from setuptools import find_packages, setup

package_name = "p3at_navigation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/worlds", glob("worlds/*.sdf")),
        ("share/" + package_name + "/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mubeen Naeem Chaghtai",
    maintainer_email="mubeen.chaghtai@gmail.com",
    description="SLAM, AMCL localisation and autonomous navigation for the Pioneer 3-AT in Gazebo Harmonic.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "goal_runner = p3at_navigation.goal_runner:main",
            "generate_world = p3at_navigation.generate_world:main",
        ],
    },
)
