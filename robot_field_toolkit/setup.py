from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_field_toolkit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rjsgud',
    maintainer_email='rjsgud@todo.todo',
    description='MoMa 현장 셋업·유지보수용 진단 툴킷 (고장 주입, 감시, 알람)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fault_injector = robot_field_toolkit.nodes.fault_injector:main',
            'motion_monitor = robot_field_toolkit.nodes.motion_monitor:main',
            'arm_controller = robot_field_toolkit.nodes.arm_controller:main',
            'safety_interlock = robot_field_toolkit.nodes.safety_interlock:main',
        ],
    },
)
