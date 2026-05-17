from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'qbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nadeesha',
    maintainer_email='nadeeshajayamanne@protonmail.com',
    description=(
        'Launches the full QBot gesture-to-behaviour pipeline (Kinect '
        'driver, gesture node, face tracker, behaviour node) with shared '
        'tuning loaded from config/qbot_params.yaml.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_test_node = qbot_bringup.motion_test_node:main',
        ],
    },
)