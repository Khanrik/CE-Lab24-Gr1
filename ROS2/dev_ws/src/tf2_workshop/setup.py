from setuptools import find_packages, setup

package_name = 'tf2_workshop'

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
    maintainer='henrik',
    maintainer_email='202307381@post.au.dk',
    description='ROS2 TF2 Tutorial',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'broadcaster = tf2_workshop.broadcaster:main',
            'listener = tf2_workshop.listener:main'
        ],
    },
)
