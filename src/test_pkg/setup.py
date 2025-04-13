from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name, 'launch'),glob('launch/*.launch.py')),
        # (os.path.join('share',package_name, 'launch'),['src/launch/demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='math286ros',
    maintainer_email='math286ros@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'USB2CAN_receive_node = test_pkg.USB2CAN_receive_node:main',
            'USB2CAN_send_node    = test_pkg.USB2CAN_send_node:main',
            'GM6020_monitor_node  = test_pkg.GM6020_monitor_node:main',
            'GM6020_control_node  = test_pkg.GM6020_control_node:main',
            'Keyboard_node        = test_pkg.Keyboard_node:main', 
        ],
    },
)
