import os
from glob import glob

from setuptools import setup

package_name = 'transporter_odrive'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='transporter',
    maintainer_email='todo@todo.todo',
    description='ODrive USB driver for transporter wheels',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'odrive_node = transporter_odrive.odrive_node:main'
        ],
    },
)
