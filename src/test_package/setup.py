import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'test_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/test', glob('test/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adınız Soyadınız',
    maintainer_email='sizin.email@adresiniz.com',
    description='TITRA mülakat testi için UAV kontrol paketi.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_node = test_package.uav_node:main',  # Doğru module path
        ],
    },
)