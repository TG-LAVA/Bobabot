from setuptools import setup
import os
from glob import glob

package_name = 'server_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),  # ✅ 加入 marker
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'sql'), glob('sql/*.sql')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yinuo',
    maintainer_email='Lyn2014667478@outlook.com',
    description='ROS2 database and command server for BubbleBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_tester = server_interface.order_tester:main',
            'ros_db_node = server_interface.ros_db_node:main',
            'generate_workflow_from_last_order = server_interface.generate_workflow_from_last_order:main',
            'dispatch_workflow_server = server_interface.dispatch_workflow_server:main',
        ],
    },
)
