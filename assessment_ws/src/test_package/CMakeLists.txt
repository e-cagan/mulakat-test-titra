cmake_minimum_required(VERSION 3.5)
project(test_package)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

# Python modülünü kur
ament_python_install_package(${PROJECT_NAME})

# --------- PYTEST TESTLERİ -----------
if(BUILD_TESTING)
  # Görev 1
  ament_add_pytest_test(test_task_1_arm_disarm
    PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
  )

  # Görev 2
  ament_add_pytest_test(test_task_2_takeoff_land
    PYTHON_EXECUTABLE ${PYTHON_EXECUTABLE}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
endif()

ament_package()
