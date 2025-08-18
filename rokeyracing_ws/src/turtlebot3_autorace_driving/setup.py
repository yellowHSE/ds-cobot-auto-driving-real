from glob import glob

from setuptools import find_packages
from setuptools import setup


package_name = 'turtlebot3_autorace_driving'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name+'/launch',glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'control_lane = turtlebot3_autorace_driving.control_lane:main',
		'control_moving = turtlebot3_autorace_driving.control_moving:main',
        ],
    },
)
