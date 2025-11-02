import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'px4_autonomy'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        ('share/' + package_name + '/launch', glob('launch/*.launch')),
    ],
    install_requires=['setuptools','setuptools_scm'],
    zip_safe=True,
    maintainer='udl',
    maintainer_email='hello@usefuldynamics.io',
    description='TODO: Package description',
    license='CC-BY-NC-4.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'vehicle_interface = px4_autonomy.nodes.offboard.vehicle_interface:main',
        ],
    },
)
