from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'dummy_detection_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'test.*']),
    install_requires=[
        'setuptools',
        'pyyaml',  # for YAML parsing
    ],
    python_requires='>=3.8,<4',
    zip_safe=True,
    maintainer='Marco Wallner',
    maintainer_email='marco.wallner@ait.ac.at',
    author='Marco Wallner',
    author_email='marco.wallner@ait.ac.at',
    url='https://gitlab.ait.ac.at/as/general/caripu/ws_led_strip_hmi',
    description=(
        'Publishes mock vision_msgs/Detection3DArray messages based on '
        'preconfigured person trajectories (persons.yml) for testing LED-strip '
        'projection and visualization.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dummy_publisher = dummy_detection_publisher.dummy_publisher:main',
        ],
    },
    data_files=[
        # ament index marker
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package manifest
        ('share/' + package_name, ['package.xml']),
        # YAML configuration
        ('share/' + package_name + '/cfg', glob(os.path.join('cfg', '*.yml'))),
        # launch files
        ('share/' + package_name + '/launch', ['launch/dummy_publisher.launch.py']),
        # mesh assets
        ('share/' + package_name + '/meshes', glob(os.path.join('meshes', '*.dae'))),
        ('share/' + package_name + '/meshes', glob(os.path.join('meshes', '*.jpg'))),
    ],
)
