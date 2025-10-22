from setuptools import setup, find_packages

package_name = 'watchdog'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/watchdog_params.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/watchdog.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Fam Shihata',
    maintainer_email='fam@awadlouis.com',
    description='A ROS2 watchdog node for F1TENTH system monitoring and safety',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Canonical entry point
            'watchdog_node = watchdog.watchdog_node:main',
            # Aliases for backward compatibility (map old names to the same main)
            'Watchdog_Node = watchdog.watchdog_node:main',
            'watchdogNode = watchdog.watchdog_node:main',
        ],
    },
)
