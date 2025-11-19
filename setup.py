from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'g09_prii3'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # ARCHIVOS LAUNCH - incluir cualquier .py en launch
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        
        # WORLDS - instalar bajo share/<pkg>/worlds
        (os.path.join('share', package_name, 'worlds'), 
         glob('mundos_gazebo/*.world')),
        
        # MODELOS - instalar bajo share/<pkg>/models/f1l3 (contenido recursivo)
        (os.path.join('share', package_name, 'models', 'f1l3'), 
         glob('mundos_gazebo/f1l3/**/*', recursive=True)),
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
            'drawer_number_gazebo = g09_prii3.drawer_number_gazebo:main',
            'prii3_turtlesim_node = g09_prii3.prii3_turtlesim_node:main',
            'jetbot_drawer = g09_prii3.jetbot_drawer_node:main',
            'jetbot_obstacle_avoidance = g09_prii3.obstacle_avoidance_node:main',
            'jetbot_potential_fields = g09_prii3.Potential_Fields:main',
            'ml_test = g09_prii3.ml_test:main',
            'nav_f1l3_cordenadas = g09_prii3.nav_f1l3_cordenadas:main',
            'rviz_predefinido_node = g09_prii3.rviz_predefinido_node:main',
        ],
    },
)