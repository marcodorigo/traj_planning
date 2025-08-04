from glob import glob
import os
from setuptools import setup

package_name = 'traj_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco',
    maintainer_email='marco3.dorigo@mail.polimi.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rrt_planner_node = traj_planning.rrt_planner_node:main',
            'visualizer_node = traj_planning.visualizer_node:main',
            'HO_reference_finder = traj_planning.HO_reference_finder:main',
            'ACS_reference_finder = traj_planning.ACS_reference_finder:main',
        ],
    },
)
