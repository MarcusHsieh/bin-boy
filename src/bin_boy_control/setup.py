from setuptools import setup
import os
from glob import glob

package_name = 'bin_boy_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=['setuptools', 'pyserial', 'smbus2'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='Kiwi drive control for autonomous trash can',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kiwi_drive_node = bin_boy_control.kiwi_drive_node:main',
            'mpu6050_node = bin_boy_control.mpu6050_node:main',
        ],
    },
)
