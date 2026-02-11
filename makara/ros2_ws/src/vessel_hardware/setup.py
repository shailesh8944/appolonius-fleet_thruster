from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vessel_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abhilash Somayajula',
    maintainer_email='abhilash@iitm.ac.in',
    description='Package for vessel hardware',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino = vessel_hardware.arduino:main',
        ],
    },
)
