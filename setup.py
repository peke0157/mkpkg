from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'mypkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hiroto Miura',
    maintainer_email='hiroto117@todo.com',
    description='ロボットシステム学授業',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pitch_server = mypkg.pitch_server:main',
            'pitch_client = mypkg.pitch_client:main',
            'pitch_checker = mypkg.pitch_checker:main',
        ],
    },
)
