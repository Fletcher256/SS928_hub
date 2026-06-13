from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'parking_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@local',
    description='ROS2 receivers for SS928 OS08A20 RTSP video and SS-LD-AS01 dToF UDP data',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_suite_node = parking_bridge.sensor_suite_node:main',
            'vision_preprocess_node = parking_bridge.vision_preprocess_node:main',
            'parking_yolo_node = parking_bridge.parking_yolo_node:main',
            'board_yolo_udp_node = parking_bridge.board_yolo_udp_node:main',
            'board_yolo_view_node = parking_bridge.board_yolo_view_node:main',
            'board_yolo_rtsp_view_node = parking_bridge.board_yolo_rtsp_view_node:main',
            'slot_geometry_transform_node = parking_bridge.slot_geometry_transform_node:main',
            'parking_target_pose_node = parking_bridge.parking_target_pose_node:main',
            'parking_planner_node = parking_bridge.parking_planner_node:main',
            'parking_metric_planner_node = parking_bridge.parking_metric_planner_node:main',
            'parking_controller_dry_run_node = parking_bridge.parking_controller_dry_run_node:main',
            'yolo_person_node = parking_bridge.yolo_person_node:main',
            'stm32_udp_bridge = parking_bridge.stm32_udp_bridge:main',
            'dtof_bridge = parking_bridge.dtof_bridge:main',
            'camera_bridge = parking_bridge.camera_bridge:main',
        ],
    },
)
