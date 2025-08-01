from setuptools import setup, find_packages
from pathlib import Path

package_name = 'test_package'
this_dir = Path(__file__).parent

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    include_package_data=True,

    package_data={  # Python modüllerinizin içine ekle
        'test_package': ['launch/*.py'],
    },
    data_files=[    # install sonunda share/ içini oluştur
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', [str(this_dir / 'launch/px4_sim.launch.py')]),
        (f'share/{package_name}/test', [
            str(this_dir / 'test/test_task_1_arm_disarm.py'),
            str(this_dir / 'test/test_task_2_takeoff_land.py'),
        ]),
    ],

    install_requires=['setuptools'],
    tests_require=[
        'pytest',
        'launch_testing',
        'launch_testing_ros',
    ],

    zip_safe=True,
    maintainer='Beyza Beril',
    maintainer_email='…',
    description='Aday kodunu doğrulayan simülasyon test paketi.',
    license='Apache-2.0',
    entry_points={},
)
