from setuptools import find_packages, setup

package_name = 'mavros_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adınız Soyadınız',
    maintainer_email='sizin.email@adresiniz.com',
    description='MAVROS ile iletişimi basitleştiren arayüz paketi.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Bu paket bir kütüphane olduğu için çalıştırılabilir bir script'i yok.
        ],
    },
)
