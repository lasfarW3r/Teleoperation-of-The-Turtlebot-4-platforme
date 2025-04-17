
from setuptools import find_packages, setup

package_name = 'vel_sender'

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
    maintainer='yw3r',
    maintainer_email='yw3r@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_joystick = vel_sender.yasir_sending_velocity:main',
            'camera = vel_sender.camera:main',
            'send_keyboard = vel_sender.teleop_key:main',
        ],
    },
)
