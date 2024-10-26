from setuptools import find_packages, setup

package_name = 'webcam_video_streamer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tzu-Ching Yang',
    maintainer_email='shingekinocore@gmail.com',
    description='ROS2 streamer that create topic with name according to stream type',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_node = webcam_video_streamer.stream_node:main',
        ],
    },
)
