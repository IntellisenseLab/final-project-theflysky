from setuptools import find_packages, setup

package_name = 'gesture_node'

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
    maintainer='nadeesha',
    maintainer_email='nadeeshajayamanne@protonmail.com',
    description=(
        'Stream-based gesture recognition for the QBot. Publishes '
        '/gesture/tracking continuously and /gesture/command when '
        'temporal signatures (beckon, stop, circle, point, wave) '
        'are confirmed.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gesture_command_node = gesture_node.gesture_command_node:main',
        ],
    },
)
