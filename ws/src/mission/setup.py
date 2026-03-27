from setuptools import find_packages, setup
from glob import glob
import os

package_name = "mission"

# Define the directory containing the module files.
# Here, we assume that the folder 'mission' (which contains your .py files)
# is located in the same directory as this setup.py file.
mission_dir = os.path.join(os.path.dirname(__file__), "mission")

# Automatically generate console script entry points for each .py file (excluding __init__.py)
console_scripts = []
for filename in os.listdir(mission_dir):
    if filename.endswith(".py") and filename != "__init__.py":
        # Remove the .py extension to get the module name
        module_name = filename[:-3]
        # Each entry point follows the pattern: "module_name = mission.module_name:main"
        console_scripts.append(f"{module_name} = mission.{module_name}:main")

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/mission/launch", glob("launch/*.launch.py")),
        ("share/mission/config", glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "opencv-python",
        "cv_bridge",
        "rclpy",
        "mavros_msgs",
        "geometry_msgs",
        "sensor_msgs",
        "std_msgs",
    ],
    zip_safe=True,
    maintainer="",
    maintainer_email="",
    description="",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": console_scripts,
    },
)
