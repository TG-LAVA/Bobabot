from setuptools import find_packages, setup

package_name = 'my_apriltag_pnp'

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
    maintainer='huiyu',
    maintainer_email='hren8555@uni.sydney.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_pnp_node = my_apriltag_pnp.apriltag_pnp_node:main',
        ],
    },
)
