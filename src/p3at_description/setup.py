from glob import glob

from setuptools import setup

package_name = "p3at_description"

setup(
    name=package_name,
    version="1.0.0",
    packages=[],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", glob("urdf/*.xacro")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mubeen Naeem Chaghtai",
    maintainer_email="mubeen.chaghtai@gmail.com",
    description="URDF model of the Pioneer 3-AT with a 2D lidar and Gazebo Harmonic plugins.",
    license="MIT",
)
