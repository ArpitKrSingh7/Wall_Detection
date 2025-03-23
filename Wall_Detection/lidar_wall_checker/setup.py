from setuptools import setup , find_packages
import os
from glob import glob

package_name = 'lidar_wall_checker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arpit',
    maintainer_email='arpit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_threshold_node = lidar_wall_checker.lidar_threshold_node:main' ,
            'wall_alert_node = lidar_wall_checker.wall_alert_node:main',
            'beep_alert_node = lidar_wall_checker.beep_alert_node:main'
        ],
    },
)
