import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'test_package'

setup(
    name=package_name,
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/test',   glob('test/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adınız Soyadınız',
    maintainer_email='sizin.email@adresiniz.com',
    description='TITRA mülakat testi için UAV kontrol paketi.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    
    # 3. Çalıştırılabilir betikleri (düğümleri) tanımlar.
    # Bu bölüm, 'ros2 run' komutunun çalışmasını sağlar.
    entry_points={
        'console_scripts': [
            'uav_node = mulakat_test_titra.uav_node:main',
        ],
    },
)