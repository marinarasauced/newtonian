
from glob import glob
import os
import xml.etree.ElementTree as ET
from setuptools import find_packages, setup


PACKAGE = 'newtonian'
VERSION = ET.parse('package.xml').getroot().findtext('version')


def tree(source: str) -> list[tuple[str, list[str]]]:
    """
    """
    return [(os.path.join('share', PACKAGE, root), [os.path.join(root, f) for f in files]) for root, _, files in os.walk(source)]


setup(
    name=PACKAGE,
    version=VERSION,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE]),
        ('share/' + PACKAGE, ['package.xml']),
        ('share/' + PACKAGE + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE + '/launch', glob('launch/*.launch.py')),
        *tree(source='descriptions'),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marina NELSON',
    maintainer_email='marinarasauced@outlook.com',
    description='Newton simulator with runtime configuration wrapped for ROS 2.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sim = newtonian.node:main',
        ],
    },
)
