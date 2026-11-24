import os
from glob import glob

from setuptools import setup

package_name = 'transporter_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='transporter',
    maintainer_email='todo@todo.todo',
    description='Launch files and config for transporter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [

        ],
    },
)
