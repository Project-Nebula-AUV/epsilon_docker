from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robosub'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/robosub']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.py')),
        ('share/' + package_name + '/config',
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RoboSub Team',
    maintainer_email='todo@todo.com',
    description='AUV control stack with pygame simulator and ROS2 node interfaces.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = robosub.nodes.simulator_node:main',
            'submarine_node = robosub.nodes.submarine_node:main',
            'recorder_node  = robosub.nodes.recorder_node:main',
            'web_node       = robosub.nodes.web_node:main',
        ],
    },
)
