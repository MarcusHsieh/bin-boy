from setuptools import setup

package_name = 'motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','pyserial'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='marcus.j.hsieh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qt_joystick = motor_controller.qt_joystick_gui:main',
            'twist_to_serial = motor_controller.twist_to_esp32_serial:main',
        ],
    },
)
