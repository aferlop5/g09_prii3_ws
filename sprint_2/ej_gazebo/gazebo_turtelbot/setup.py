from setuptools import setup

package_name = 'gazebo_turtelbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aferlop5',
    maintainer_email='aferlop5@upv.edu.es',
    description='Simulación del TurtleBot3 en Gazebo para el Sprint 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'draw_number = gazebo_turtelbot.draw_number:main',
        ],
    },
)
