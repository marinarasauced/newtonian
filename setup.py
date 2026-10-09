
from glob import glob
from setuptools import find_packages, setup


PACKAGE = 'newtonian'


setup(
    name=PACKAGE,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marina NELSON',
    maintainer_email='marinarasauced@outlook.com',
    description='Newton simulator with runtime configuration wrapped for ROS 2.',
    license='MIT license',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim = src.node:main',
        ],
    },
)
