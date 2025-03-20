from setuptools import setup
import os
from glob import glob

package_name = 'plant_photography_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JohnMiller',
    maintainer_email='johnamiller056@gmail.com',
    description='Python nodes for plant photography robot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_control_node = plant_photography_py.main_control_node:main',
            'plant_detection_node = plant_photography_py.plant_detection_node:main',
            'navigation_node = plant_photography_py.navigation_node:main',
            'camera_control_node = plant_photography_py.camera_control_node:main',
        ],
    },
)
