from setuptools import setup
import os
from glob import glob
package_name = 'drone_simulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py')),
        (os.path.join('share', package_name), glob('config/*.rviz')), 
        (os.path.join('share', package_name), glob('stl/*.stl'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hlab',
    maintainer_email='patilunmeshrao@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'Drone_sim = drone_simulation.sim:main',
        'trajectoryGen = drone_simulation.trajectoryGen',
        'quadrocoptertrajectory = drone_simulation.quadrocoptertrajectory',
        'LQR = drone_simulation.LQR',
        'obstacle_detector = drone_simulation.obst_detector:main',
        'obstacle_generator = drone_simulation.obst_publisher_node:main',
        'utils = drone_simulation.utils',
        'nonlinear_dynamics = drone_simulation.nonlinear_dynamics'
        ],
    },
)
