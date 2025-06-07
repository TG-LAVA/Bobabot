from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_arm_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.urdf')),
    ],
    install_requires=[
        'setuptools',
        'rclpy>=1.0.0',
        'control_msgs',
        'trajectory_msgs',
        'base_interfaces',  
    ],
    zip_safe=True,
    maintainer='yinuo',
    maintainer_email='Lyn2014667478@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'TFviewer = robot_arm_driver.TF_viewer:main',
            'RobotArmPlanner = robot_arm_driver.RobotArmPlanner:main',
            'TrajectoryAdapter = robot_arm_driver.followTrajectory:main',
            'FSMController = robot_arm_driver.bubblebot_fsm_node:main',
            'Calibration = robot_arm_driver.calibration:main',
            'capture = robot_arm_driver.capture:main',
        ],
    },
)
