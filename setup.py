from setuptools import setup
import os
from glob import glob

package_name = 'g09_prii3'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Grupo 09',
    maintainer_email='aferlop5@upv.edu.es',
    description='Nodos ROS2 del Grupo 09 organizados por sprint',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drawer_number = g09_prii3.drawer_number:main',
            'prii3_turtlesim_node = g09_prii3.prii3_turtlesim_node:main',
        ],
    },
)
