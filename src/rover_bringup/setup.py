from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_bringup'
# Search for launch files placed directly in the package's ``launch`` directory
# rather than under an additional ``rover_bringup`` subdirectory.  This aligns
# the installation paths with the typical ROS 2 layout where launch files are
# located at the top level of the package.
launch_files = glob(os.path.join('launch', '*.launch.py'))

# Similarly, gather any configuration files stored directly in a top-level
# ``config`` directory.
config_files = glob(os.path.join('config', '*.yaml'))

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
