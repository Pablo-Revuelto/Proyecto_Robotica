from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'irb120_jazzy_sim'

def collect_files(directory):
    files = []
    for root, _, filenames in os.walk(directory):
        for filename in filenames:
            full_path = os.path.join(root, filename)
            install_dir = os.path.join(
                "share",
                package_name,
                root,
            )
            files.append((install_dir, [full_path]))
    return files

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]


for directory in ["launch", "urdf", "meshes", "worlds", "config"]:
    data_files.extend(collect_files(directory))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mpereira',
    maintainer_email='mpereira@todo.todo',
    description="ABB IRB120 simulation in ROS 2 Jazzy, Gazebo Harmonic and MoveIt",
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "move_to_joint = irb120_jazzy_sim.move_to_joint:main",
            "move_to_clicked = irb120_jazzy_sim.move_to_clicked:main",
            "add_obstacle = irb120_jazzy_sim.add_obstacle:main",
        ],
    },
)
