from setuptools import setup

package_name = 'watchdog'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohammedazab',
    maintainer_email='mohammed3zab@outlook.com',
    description='Watchdog node for system monitoring',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'Watchdog_Node = watchdog.watchdogNode:main'
        ],
    },
)
