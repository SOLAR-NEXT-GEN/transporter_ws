import os
from glob import glob

from setuptools import setup

package_name = 'transporter_mobile'

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
    description='Differential drive kinematics for transporter mobile base',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mobile_controller = transporter_mobile.mobile_controller:main'
        ],
    },
)
