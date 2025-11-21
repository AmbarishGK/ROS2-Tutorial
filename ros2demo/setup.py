from setuptools import find_packages, setup

package_name = 'ros2demo'

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
    maintainer='mab',
    maintainer_email='ambarishgk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'ros2demo_publisher = ros2demo.ros2demo_publisher:main',
        'ros2demo_subscriber = ros2demo.ros2demo_subscriber:main',
    ],
},

)
