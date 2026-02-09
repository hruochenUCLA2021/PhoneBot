from setuptools import find_packages, setup

package_name = 'phonebot_bridge'

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
    maintainer='hrc',
    maintainer_email='houruochen@g.ucla.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'phonebot_udp_bridge_periodic = phonebot_bridge.udp_bridge_periodic:main',
            'phonebot_udp_bridge_immediate = phonebot_bridge.udp_bridge_immediate:main',
            'phonebot_example_motor_pub = phonebot_bridge.example_motor_pub:main',
        ],
    },
)
