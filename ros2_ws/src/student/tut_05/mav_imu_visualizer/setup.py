from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mav_imu_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
        (os.path.join('share', package_name, 'web'), glob('web/*.js')),
        (os.path.join('share', package_name, 'web', 'node_modules', 'roslib', 'build'), 
         glob('web/node_modules/roslib/build/*')),
        (os.path.join('share', package_name, 'web', 'node_modules', 'chart.js', 'dist'), 
         glob('web/node_modules/chart.js/dist/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhilash Somayajula',
    maintainer_email='abhilash@iitm.ac.in',
    description='MAV Lab - IMU Data Visualizer',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
