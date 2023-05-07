from setuptools import find_packages, setup
from glob import glob

package_name = 'fire_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fyp',
    maintainer_email='fyp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_fire = fire_detector.detect_fire:main',
            'detect_fire_3d = fire_detector.detect_fire_3d:main',
            'follow_fire = fire_detector.follow_fire:main',
        ],
    },
)
