# Gerekli minimum CMake versiyonunu belirtir.
cmake_minimum_required(VERSION 3.8)

# Proje adını tanımlar. Bu ad, package.xml dosyasındaki paket adıyla eşleşmelidir.
project(mulakat_test_titra)

# ament_cmake: ROS 2 paketleri için temel derleme araçlarını bulur.
find_package(ament_cmake REQUIRED)
# ament_cmake_python: Python tabanlı paketler için gerekli araçları bulur.
find_package(ament_cmake_python REQUIRED)
# ament_cmake_pytest: Pytest ile testleri çalıştırmak için gerekli araçları bulur.
find_package(ament_cmake_pytest REQUIRED)

# package.xml dosyasını kurulum dizinine kopyalar.
# Bu, ROS 2'nin paketinizi ve bağımlılıklarını tanıması için zorunludur.
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Python launch dosyalarını içeren 'launch' dizinini kurulum dizinine kopyalar.
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Python test dosyalarını ve ilgili kaynakları içeren 'test' dizinini kopyalar.
install(
    DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
)

# Bu blok, sadece testler çalıştırıldığında (`colcon test`) aktif olur.
if(BUILD_TESTING)
  # Pytest testlerinizi CMake'e ekler.
  # colcon test komutu bu sayede hangi testin çalışacağını bilir.
  # Projenizde birden fazla test dosyası varsa, her biri için ayrı bir satır ekleyebilirsiniz.
  add_pytest_test(test/test_uav.py)
endif()

# Paketin kurulumunu tamamlamak için ament_package() fonksiyonu çağrılır.
# Bu satır her ament_cmake paketi için zorunludur ve dosyanın en sonunda yer almalıdır.
ament_package()