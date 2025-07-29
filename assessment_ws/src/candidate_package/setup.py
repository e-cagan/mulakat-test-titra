from setuptools import find_packages, setup

package_name = 'candidate_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'rclpy',
    'mavros_msgs',
    'geometry_msgs'
    ],
    zip_safe=True,
    maintainer='Adınız Soyadınız',
    maintainer_email='sizin.email@adresiniz.com',
    description='Adayın görevlerini yerine getireceği paket.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_task = candidate_package.main:main',
        ],
    },
)
