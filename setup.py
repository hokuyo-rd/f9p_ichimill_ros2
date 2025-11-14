from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'f9p_ichimill'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jane Doe',
    maintainer_email='jane.doe@example.com',
    description='The f9p_ichimill package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'f9p_driver = scripts.f9p_driver:main',
            'ichimill_connect = scripts.ichimill_connect:main',
            'ntripcaster_connect = scripts.ntripcaster_connect:main',
            'fix2csv = scripts.fix2csv:main',
        ],
    },
)