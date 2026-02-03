from setuptools import find_packages, setup

package_name = 'indoor_gimbal_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacopo',
    maintainer_email='jacopo.vivaldi@tii.ae',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_interface = indoor_gimbal_controller.driver_interface:main',
            'ptz_interface = indoor_gimbal_controller.ptz_interface:main',
            'topotek_udp_driver = indoor_gimbal_controller.topotek_udp_driver:main',
        ],
    },
)


