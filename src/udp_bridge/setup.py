from setuptools import find_packages, setup

package_name = 'udp_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Config-driven UDP/ROS 2 bridge.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'udp_to_ros = udp_bridge.udp_to_ros_node:main',
            'ros_to_udp = udp_bridge.ros_to_udp_node:main',
            'generate_wireshark_dissector = udp_bridge.wireshark:main',
        ],
    },
)
