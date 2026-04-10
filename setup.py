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
        (os.path.join('share', package_name, 'config'), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oakley thomas',
    maintainer_email='oakley.j.thomas@gmail.com',
    description='A Mixed Reality ROS2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dual_ekf_localization = mr_pkg.dual_ekf_localization:main',
            'autoware_localization = mr_pkg.autoware_localization:main',
            'carla_localization = mr_pkg.carla_localization:main',
        ],
    },
)
