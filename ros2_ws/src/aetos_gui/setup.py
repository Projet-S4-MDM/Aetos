from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aetos_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*.ui')),  
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aetos',
    maintainer_email='papn1801@usherbrooke.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = aetos_gui.main:main',
        ],
    },
)
