from setuptools import find_packages, setup
import os
from glob import glob

package_name = "temperature_publisher"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.xml')))
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mahmoud Taha",
    maintainer_email="mahmoud3taha@gmail.com",
    description="ROS2 python package that publishes temperature readings with sin noise.",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "temperature_publisher_node = temperature_publisher.temperature_publisher_node:main"
        ],
    },
)
