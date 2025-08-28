from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('rover_nav/launch/*.launch.py')),
        ('share/' + package_name + '/maps', glob('rover_nav/maps/*')),
        ('share/' + package_name + '/params', glob('rover_nav/params/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='santari',
    maintainer_email='aleksantari@gmail.com',
    description='Nav2 bringup for rover',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
