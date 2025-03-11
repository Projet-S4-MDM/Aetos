from setuptools import setup
from setuptools import find_packages
import os
from glob import glob

package_name = 'websocket_listener'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='ton_nom',
    maintainer_email='ton_email@example.com',
    description='Un nœud ROS2 qui écoute des données WebSocket',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = websocket_listener.listener:main',
            'serveur = websocket_listener.serveur:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
)
