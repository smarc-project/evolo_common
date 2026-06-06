from setuptools import find_packages, setup
import glob
import os

package_name = 'evolo_accoustic_com'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kaveh',
    maintainer_email='najafian@kth.se',
    description='ROS2 node for scheduling and forwarding acoustic/serial succor commands.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'succor_command_scheduler = evolo_accoustic_com.succor_command_scheduler:main',
        ],
    },
)
