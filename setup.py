from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mr_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oakley',
    maintainer_email='oakley.j.thomas@gmail.com',
    description='A Mixed Reality ROS2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_transform = mr_pkg.world_transformation:main',
            'virtual_perception = mr_pkg.virtual_perception:main',
            'webcam_publisher = mr_pkg.webcam_publisher:main',
            'image_overlay = mr_pkg.image_overlay:main'
        ],
    },
)
