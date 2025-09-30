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
        ('share/' + package_name + '/msg', [
            'msg/SanityWarning.msg',
            'msg/SensorHealth.msg', 
            'msg/SanitySummary.msg',
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
            'watchdog_node = watchdog.watchdog_node:main',
            'fsm_integration_node = watchdog.fsm_integration:main',
            # 'Watchdog_Node = watchdog.watchdogNode:main'   Deprecated entry point
        ],
    },
)
