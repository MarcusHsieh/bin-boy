from setuptools import setup
import os
from glob import glob

package_name = 'bin_boy_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='Perception and tracking for bin_boy robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'person_tracker = bin_boy_perception.person_tracker:main',
            'mock_person_detector = bin_boy_perception.mock_person_detector:main',
        ],
    },
)
