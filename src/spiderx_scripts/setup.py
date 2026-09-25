from setuptools import find_packages, setup

package_name = 'spiderx_scripts'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jagadeswar',
    maintainer_email='jagadeswar@gmail.com',
    description='SpiderX diagnostic ROS 2 nodes (lidar sectors, joint angles per leg).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_lidar = spiderx_scripts.read_lidar:main',
            'read_joint_states = spiderx_scripts.read_joint_states:main',
        ],
    },
)
