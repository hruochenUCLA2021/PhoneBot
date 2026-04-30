from setuptools import find_packages, setup

package_name = 'pc_policy_runner_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/pc_policy_runner.yaml']),
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
            'pc_policy_runner = pc_policy_runner_python.pc_policy_runner_node:main',
        ],
    },
)
