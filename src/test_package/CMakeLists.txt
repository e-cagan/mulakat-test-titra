cmake_minimum_required(VERSION 3.5)
project(test_package)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_pytest REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_pytest_test(test_task_1_arm_disarm
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  ament_add_pytest_test(test_task_2_takeoff_land
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test
  )
endif()

ament_package()
