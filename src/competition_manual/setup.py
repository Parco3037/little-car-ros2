from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'competition_manual'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'LICENSE']),
    ('share/' + package_name + '/launch', ['launch/manual.launch.py']),
    ('share/' + package_name + '/sounds', glob('sounds/*.wav')),
]
for root, _, files in os.walk('model'):
    destination = os.path.join('share', package_name, root)
    data_files.append((destination, [os.path.join(root, filename) for filename in files]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Competition team',
    maintainer_email='189249227+Parco3037@users.noreply.github.com',
    description='Xbox manual input adapter for MentorPi',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'xbox_teleop = competition_manual.xbox_teleop:main',
            'joy_reconnect = competition_manual.joy_reconnect:main',
            'voice_node = competition_manual.voice_node:main',
            'auto_advance = competition_manual.auto_advance:main',
        ],
    },
)
