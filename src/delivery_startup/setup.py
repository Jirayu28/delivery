import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'delivery_startup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jirayu',
    maintainer_email='jirayu.pkd@gmail.com',
    description='Start all nodes for delivery robot',
    license='Apache-2.0',
)
