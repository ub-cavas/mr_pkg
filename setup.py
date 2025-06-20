from setuptools import find_packages, setup

package_name = 'mr_pkg'

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
    maintainer='oakley',
    maintainer_email='oakley.j.thomas@gmail.com',
    description='A Mixed Reality ROS2 Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'world_transform = mr_pkg.world_transformation:main'
            'virtual_perception = mr_pkg.virtual_perception:main'
        ],
    },
)
