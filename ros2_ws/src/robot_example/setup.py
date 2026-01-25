from setuptools import setup, find_packages

package_name = 'robot_example'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robot Team',
    maintainer_email='admin@robot.local',
    description='Example ROS 2 Python package untuk robot automation system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_talker = robot_example.talker:main',
            'robot_listener = robot_example.listener:main',
            'robot_controller = robot_example.ros2_controller:main',
            'hardware_monitor = robot_example.ros2_hardware_monitor:main',
            'kinematics_processor = robot_example.ros2_kinematics:main',
            'config_manager = robot_example.ros2_config_manager:main',
        ],
    },
)
