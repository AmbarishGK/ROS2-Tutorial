from setuptools import find_packages, setup

package_name = 'image_demo'

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
            'image_filler_publisher = image_demo.image_filler_publisher:main',
            'image_relay = image_demo.image_relay:main',
            'video_publisher = image_demo.video_publisher:main',
        ],
    },
)
