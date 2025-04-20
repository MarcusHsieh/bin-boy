from setuptools import find_packages, setup

package_name = 'csi_camera_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Removed launch directory installation
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mj',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='ROS2 package to publish CSI camera stream using GStreamer on Jetson Nano',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'csi_camera_node = csi_camera_ros2.csi_camera_node:main',
            # Removed image_viewer_node entry point
        ],
    },
)
