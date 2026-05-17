from setuptools import find_packages, setup

package_name = 'behavior_node'

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
        'Behaviour planner for the QBot. Turns gesture commands into '
        'depth-aware Twist segments and Kobuki sound sequences, then '
        're-centres on the user after each action.'
    ),
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pet_behavior_node = behavior_node.pet_behavior_node:main',
        ],
    },
)
