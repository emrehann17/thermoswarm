#!/usr/bin/env python3

from setuptools import setup

package_name = 'thermoswarm_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ThermoSwarm ROS2 control package (refactor phase 1)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'thermoswarm_three_x500_node=thermoswarm_control.thermoswarm_three_x500_node:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/thermoswarm_control']),
        ('share/' + package_name, ['package.xml']),
    ],
)
