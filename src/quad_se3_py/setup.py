from glob import glob
from os.path import join

from setuptools import find_packages, setup

package_name = 'quad_se3_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sachan',
    maintainer_email='239834638+UwUchi@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_node = quad_se3_py.trajectory_node:main',
            'controller_node = quad_se3_py.controller_node:main',
            'dynamics_node = quad_se3_py.dynamics_node:main',
        ],
    },
)
