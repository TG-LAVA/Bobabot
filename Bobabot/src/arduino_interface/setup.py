from setuptools import find_packages, setup

package_name = 'arduino_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['arduino_interface', 'base_interfaces', 'base_interfaces.*'], exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'pyserial'],
    zip_safe=True,
    maintainer='huiyu',
    maintainer_email='hren8555@uni.sydney.edu.au',
    description='Bridge between ROS2 and Arduino using serial.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_bridge = arduino_interface.serial_bridge:main',
            'angle_listener = arduino_interface.angle_listener:main',
            'plot_angle_error = arduino_interface.plot_angle_error:main',
        ],
    },
)
