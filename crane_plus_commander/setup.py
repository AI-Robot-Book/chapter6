import os
from glob import glob
from setuptools import setup

package_name = 'crane_plus_commander'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MASUTANI Yasuhiro',
    maintainer_email='ai-robot-book@googlegroups.com',
    description='Commander for crane_plus_control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'commander1 = crane_plus_commander.commander1:main',
            'commander2 = crane_plus_commander.commander2:main',
            'commander3 = crane_plus_commander.commander3:main',
            'commander4 = crane_plus_commander.commander4:main',
            'commander5 = crane_plus_commander.commander5:main',
            'commander6 = crane_plus_commander.commander6:main',
            'test_client = crane_plus_commander.test_client:main',
         ],
    },
)
