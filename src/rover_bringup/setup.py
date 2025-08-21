from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_bringup'
launch_files = glob(os.path.join('rover_bringup','launch', '*.launch.py'))
config_files = glob(os.path.join('rover_bringup','config', '*.yaml'))

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/config', config_files),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santari',
    maintainer_email='aleksantari@gmail.com',
    description='bringup package lauching Waveshare nodes + EKF',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
