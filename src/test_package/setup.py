from pathlib import Path
from setuptools import setup, find_packages

package_name = "test_package"
this_dir = Path(__file__).parent

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    # ---- launch dosyasını paket verisi olarak ekle
    package_data={
        "test_package": ["launch/px4_sim.launch.py"],
    },
    include_package_data=True,          # <─  paket verilerini kopyala
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            [str(Path(__file__).parent / 'launch' / 'px4_sim.launch.py')]),
        #  test dosyalarını da install et → CTest copy’lemezse burada bulunur
        ('share/' + package_name + '/test', ['test/test_task1_arm_disarm.py',
                                            'test/test_task2_takeoff_land.py']),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Beyza Beril",
    maintainer_email="beyzaberilyalcinkaya@hotmail.com",
    description="Aday kodunu doğrulayan simülasyon test paketi",
    license="Apache-2.0",
    tests_require=["pytest"],
)
