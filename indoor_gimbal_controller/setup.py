from setuptools import setup
import os
from glob import glob

package_name = 'indoor_gimbal_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jacopo Vivaldi',
    maintainer_email='jacopo.vivaldi@tii.ae',
    description='Topotek UDP driver + operator GUI',
    license='MIT',
    entry_points={
        'console_scripts': [
            'driver_interface = indoor_gimbal_controller.driver_interface:main',
            'ptz_interface = indoor_gimbal_controller.ptz_interface:main',
            'topotek_udp_driver = indoor_gimbal_controller.topotek_udp_driver:main',
            'topotek_operator_gui_ros = indoor_gimbal_controller.topotek_operator_gui_ros:main',
        ],
    },
)
