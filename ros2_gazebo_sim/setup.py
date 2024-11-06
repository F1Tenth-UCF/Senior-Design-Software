from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ros2_gazebo_sim'

def get_all_files(dir):
    return [f for f in glob(f'{dir}/*') if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*launch.xml')),
        (os.path.join('share', package_name, 'world'), get_all_files('world')),
        *[ # Recursively add all files in subdirectories of world, and do so while preserving the structure of the subdirectories
            (os.path.join('share', package_name, *os.path.split(f)), get_all_files(f))
            for f in [f for f in glob('world/**', recursive=True) if os.path.isdir(f)]
        ]
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
