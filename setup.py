from setuptools import find_packages, setup

package_name = 'robot_teleop'

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
    maintainer='robo',
    maintainer_email='ksementilli@hotmail.ca',
    description='A ROS2 node to handle the conversion of gamepad input to commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_joint_jog = robot_teleop.teleop_joint_jog:main',
        ],
    },
)
