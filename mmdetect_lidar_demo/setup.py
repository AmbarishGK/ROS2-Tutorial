from setuptools import setup

package_name = 'mmdetect_lidar_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mab',
    maintainer_email='you@example.com',
    description='ROS2 demo node running MMDetection3D on LiDAR and publishing 3D bounding boxes as markers.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mmdetect_lidar_demo = mmdetect_lidar_demo.lidar_detection_node:main',
        ],
    },
)
