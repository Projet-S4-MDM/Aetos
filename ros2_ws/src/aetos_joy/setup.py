from setuptools import find_packages, setup

package_name = 'aetos_joy'

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
    maintainer='nikopapi',
    maintainer_email='papn1801@usherbrooke.ca',
    description='Joystick input processing and velocity mapping',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_demux = aetos_joy.joy_demux:main',
        ],
    },
)
