from glob import glob

from setuptools import find_packages, setup

package_name = 'epsilon_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/allocation.yaml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epsilon',
    maintainer_email='doug@douglasjohnson.org',
    description='Bridges epsilon hardware to the robosub nav stack.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'sensor_bridge = epsilon_bridge.sensor_bridge:main',
            'thruster_bridge = epsilon_bridge.thruster_bridge:main',
            'arming_helper = epsilon_bridge.arming_helper:main',
            'sysid_runner = epsilon_bridge.sysid_runner:main',
            'sysid_logger = epsilon_bridge.sysid_logger:main',
        ],
    },
)
