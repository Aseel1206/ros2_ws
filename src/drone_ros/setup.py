from setuptools import find_packages, setup

package_name = 'drone_ros'

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
    maintainer='aseel',
    maintainer_email='aseel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_node = drone_ros.drone_node:main',
            'publisher_node = drone_ros.publisher_node:main',
            'subscriber_node = drone_ros.subscriber_node:main',
            'waypoint_node = drone_ros.waypoint_node:main',
            'mission_upload = drone_ros.mission_upload:main',
            'waypoint_monitor = drone_ros.waypoint_monitor:main',
        ],
    },
)
