from setuptools import find_packages, setup

package_name = 'robot_movement'

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
    maintainer='oscar',
    maintainer_email='ofc1227@tec.mx',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'line_detector = robot_movement.line_detector:main',
        'data_odom_collector = robot_movement.data_odom_collector:main',
        'pot_fields = robot_movement.pot_fields:main',
        'pid_yaw = robot_movement.pid_yaw:main',
        'odom_delay = robot_movement.odom_delay:main',
    ],
    },
)
