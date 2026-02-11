from setuptools import setup
import os
from glob import glob

package_name = 'mav_simulator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhilash Somayajula',
    maintainer_email='abhilash@iitm.ac.in',
    description='MAV Lab - Vessel simulator for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mav_simulate = mav_simulator.mav_simulate:main',
            'mav_control = mav_simulator.mav_control:main'      # <---- ADD THIS LINE
        ],
    },
)
