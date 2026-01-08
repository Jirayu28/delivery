from setuptools import setup

package_name = 'delivery_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jirayu',
    maintainer_email='jirayu@todo.todo',
    description='ESP32 bridge for differential drive',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_bridge = delivery_controller.esp32_bridge:main',
        ],
    },
)
