from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'raicom_control'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml') + glob('config/*.json')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')),
        (os.path.join('share', package_name, 'models'), glob('models/*.*')),
        (os.path.join('share', package_name, 'compat', 'bondcpp'), glob('compat/bondcpp/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='RAICOM Team',
    maintainer_email='189249227+Parco3037@users.noreply.github.com',
    description='Safety-gated control layer for the RAICOM2026 LanderPi robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_guard = raicom_control.cmd_vel_guard:main',
            'scan_normalizer = raicom_control.scan_normalizer:main',
            'bounded_mapping_run = raicom_control.bounded_mapping_run:main',
            'keyboard_input = raicom_control.keyboard_input:main',
            'manual_control_panel = raicom_control.manual_control_panel:main',
            'nav_cmd_bridge = raicom_control.nav_cmd_bridge:main',
            'navigation_acceptance_leg = raicom_control.navigation_acceptance_leg:main',
            'odom_tf_bridge = raicom_control.odom_tf_bridge:main',
            'build_lightweight_urdf = raicom_control.lightweight_urdf:main',
            'home_mission = raicom_control.home_mission:main',
            'aurora_observer = raicom_control.aurora_observer:main',
            'yolo_roi_bridge = raicom_control.yolo_roi_bridge:main',
            'yolo_camera = raicom_control.vision_ros_nodes:camera_main',
            'yolo_infer = raicom_control.vision_ros_nodes:infer_main',
            'scene_mission = raicom_control.scene_mission:main',
            'scene_pose_recorder = raicom_control.scene_pose_recorder:main',
            'scene_point_tool = raicom_control.scene_point_tool:main',
            'watchdog_probe = raicom_control.watchdog_probe:main',
            'field_tool = raicom_control.field_tool_gui:main',
        ],
    },
)
