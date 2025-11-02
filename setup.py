from setuptools import find_packages, setup

package_name = 'px4_autonomy'

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
    maintainer='udl',
    maintainer_email='udl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'heartbeat_publisher = px4_autonomy.nodes.heartbeat_publisher:main',
        'arming_node = px4_autonomy.nodes.arming_node:main',
        'mode_switch_node = px4_autonomy.nodes.mode_switch_node:main',
        'takeoff_node = px4_autonomy.nodes.takeoff_node:main',
        'vehicle_interface = px4_autonomy.nodes.vehicle_interface:main',
	'camera_test = px4_autonomy.nodes.vision.camera_test:main',
        'aruco_detector = px4_autonomy.nodes.vision.aruco_detector:main',
        'velocity_controller = px4_autonomy.nodes.velocity_controller:main',
        'visual_servo_landing = px4_autonomy.nodes.vision.visual_servo_landing:main',
        ],
    },
)
