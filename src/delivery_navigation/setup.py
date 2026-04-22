import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'delivery_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ✅ install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # ✅ install config files (nav2 params)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),

        # ✅ install maps
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jirayu',
    maintainer_email='jirayu.pkd@gmail.com',
    description='delivery_navigation package',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={'console_scripts': []},
)
