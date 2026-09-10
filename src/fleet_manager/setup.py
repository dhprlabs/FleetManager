import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fleet_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mangal-devanshu',
    maintainer_email='sarojmangal7600@gmail.com',
    description='Decentralized multi-robot fleet manager and identical robot stack',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_broadcaster = fleet_manager.task_broadcaster:main',
            'state_manager = fleet_manager.state_manager:main',
            'task_manager = fleet_manager.task_manager:main',
            'maxsum_allocator = fleet_manager.maxsum_allocator:main',
            'bundle_manager = fleet_manager.bundle_manager:main',
            'intent_broadcaster = fleet_manager.intent_broadcaster:main',
            'pickup_validator = fleet_manager.pickup_validator:main',
            'reservation_manager = fleet_manager.reservation_manager:main',
            'conflict_resolver = fleet_manager.conflict_resolver:main',
            'nav2_bridge = fleet_manager.nav2_bridge:main',
            'task_execution_manager = fleet_manager.task_execution_manager:main',
            'p2p_transport = fleet_manager.p2p_transport:main',
            'fleet_visualizer = fleet_manager.fleet_visualizer:main',
        ],
    },
)
