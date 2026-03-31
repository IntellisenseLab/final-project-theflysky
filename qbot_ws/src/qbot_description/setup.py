from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'qbot_description'

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
            os.path.join('share', package_name, 'sdf'),
            glob('sdf/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ishan',
    maintainer_email='ishan@example.com',
    description='QBot description and simulation assets',
    license='TODO: License declaration',
    entry_points={},
)