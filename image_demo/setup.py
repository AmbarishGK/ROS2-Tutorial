from setuptools import setup

package_name = 'image_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_demo.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/yolo_image.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mab',
    maintainer_email='example@example.com',
    description='Image demo package with video publisher and YOLO detector.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_filler_publisher = image_demo.image_filler_publisher:main',
            'image_relay = image_demo.image_relay:main',
            'video_publisher = image_demo.video_publisher:main',
            'video_yolo = image_demo.video_yolo:main',
        ],
    },
)
