import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'mulakat_test_titra'

setup(
    name=package_name,
    version='0.0.0',
    # 1. Kurulacak Python paketlerini bulur.
    # 'test' dizinini hariç tutarak, projedeki 'mulakat_test_titra' adlı Python paketini otomatik olarak bulur.
    packages=find_packages(exclude=['test']),
    
    # 2. Kurulacak ek dosyaları (veri dosyaları) tanımlar.
    # Bu bölüm, CMake'deki install() komutlarının yerini alır.
    data_files=[,
        # package.xml dosyasını kurar.
        ('share/' + package_name, ['package.xml']),
        # launch dizinindeki tüm.launch.py uzantılı dosyaları kurar.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # test dizinini ve içeriğini kurar (isteğe bağlı ama iyi bir pratik).
        (os.path.join('share', package_name, 'test'), glob(os.path.join('test', '*'))),
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