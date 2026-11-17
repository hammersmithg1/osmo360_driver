from glob import glob
from setuptools import find_namespace_packages, setup

setup(
    name='osmo360_driver',
    version='0.1.0',
    packages=find_namespace_packages(include=['osmo360_camera', 'osmo360_camera.*', 'osmo360_driver']),
    package_data={'osmo360_camera.vendor': ['NOTICE', 'LICENSE.osmo-client', 'README.preview']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/osmo360_driver']),
        ('share/osmo360_driver', ['package.xml', 'README.md', 'LICENSE']),
        ('share/osmo360_driver/launch', glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    extras_require={'video': ['av>=11', 'numpy>=1.24']},
    python_requires='>=3.10',
    maintainer='hammersmithg1',
    maintainer_email='hammersmithg1@users.noreply.github.com',
    description='DJI Osmo 360 USB video, native IMU and ROS 2 driver',
    license='MIT',
    entry_points={'console_scripts': ['osmo360 = osmo360_driver.node:main']},
)
