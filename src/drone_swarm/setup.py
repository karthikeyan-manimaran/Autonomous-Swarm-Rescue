from setuptools import find_packages, setup

package_name = 'drone_swarm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='karthikeyan',
    maintainer_email='karthikeyan@example.com',

    description='Multi-UAV swarm coordination system using PX4 and ROS 2',

    license='Apache License 2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'swarm_commander = drone_swarm.swarm_commander:main',
        ],
    },
)
