from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'epsilon_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Config files
        ('share/' + package_name + '/config', ['config/camera_params.yaml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Logs
        (os.path.join('share', package_name, 'logs'), glob(f'{package_name}/logs/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='1unarzDev',
    maintainer_email='lunarzDev@outlook.com',
    description='Vision processing model for autonomous underwater navigation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera = epsilon_sensors.camera:main',
            'camera_controls = epsilon_sensors.camera_controls:main',
            'imu = epsilon_sensors.imu:main',
            'depth_sensor = epsilon_sensors.depth_sensor:main',
            'depth_fusion = epsilon_sensors.depth_fusion:main',
            'esp32_depth = epsilon_sensors.esp32_depth:main',
        ],
    },
)
