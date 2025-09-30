from setuptools import setup
import os
from glob import glob

package_name = 'prii3_turtlesim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #  Aquí añadimos los launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='agustifelo',
    maintainer_email='aferlop5@upv.edu.es',
    description='Paquete de ROS2 para dibujar números con turtlesim',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Este es tu nodo que dibuja el número 9
            'drawer = prii3_turtlesim.prii3_turtlesim_node:main',
        ],
    },
)
