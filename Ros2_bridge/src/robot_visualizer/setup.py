import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_visualizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'model', 'model_phonebot'),
            glob('model/model_phonebot/*.xml') +
            glob('model/model_phonebot/*.urdf')),
        (os.path.join('share', package_name, 'model', 'model_phonebot', 'meshes'),
            glob('model/model_phonebot/meshes/*')),
        (os.path.join('share', package_name, 'model', 'model_phonebot_fred_v2'),
            glob('model/model_phonebot_fred_v2/*.xml') +
            glob('model/model_phonebot_fred_v2/*.urdf')),
        (os.path.join('share', package_name, 'model', 'model_phonebot_fred_v2', 'meshes'),
            glob('model/model_phonebot_fred_v2/meshes/*')),
        (os.path.join('share', package_name, 'model', 'model_phonebot_fred_v2', 'assets'),
            glob('model/model_phonebot_fred_v2/assets/*')),
        (os.path.join('share', package_name, 'model', 'model_phonebot_fred_v2_torque_version'),
            glob('model/model_phonebot_fred_v2_torque_version/*.xml') +
            glob('model/model_phonebot_fred_v2_torque_version/*.urdf')),
        (os.path.join('share', package_name, 'model', 'model_phonebot_fred_v2_torque_version', 'meshes'),
            glob('model/model_phonebot_fred_v2_torque_version/meshes/*')),
        (os.path.join('share', package_name, 'model', 'model_phonebot_fred_v2_torque_version', 'assets'),
            glob('model/model_phonebot_fred_v2_torque_version/assets/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hrc',
    maintainer_email='houruochen@g.ucla.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phonebot_visualizer = robot_visualizer.visualizer_node:main',
            'phonebot_visualizer_official = robot_visualizer.official_visualizer_node:main',
        ],
    },
)
