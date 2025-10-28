from setuptools import setup
import os
from glob import glob

package_name = 'bin_boy_simulation'

data_files_list = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
    (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
]

# Only add config files if directory exists
if os.path.exists('config'):
    data_files_list.append((os.path.join('share', package_name, 'config'), glob('config/*.yaml')))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files_list,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='Gazebo simulation for bin_boy robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
