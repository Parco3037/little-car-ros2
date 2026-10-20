import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rgbd_grasp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='189249227+Parco3037@users.noreply.github.com',
    description='Guarded single-shot RGB-D blue-block grasping for MentorPi / LanderPi.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'blue_block_depth = '
            'rgbd_grasp.blue_block_depth_node:main',

            'blue_block_3d = '
            'rgbd_grasp.blue_block_3d_node:main',

            'target_to_base = '
            'rgbd_grasp.target_to_base_node:main',

            'target_to_ik = '
            'rgbd_grasp.target_to_ik_node:main',

            'grasp_once = '
            'rgbd_grasp.grasp_once_node:main',
        ],
    },
)
