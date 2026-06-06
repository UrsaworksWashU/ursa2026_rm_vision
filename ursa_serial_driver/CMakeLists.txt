cmake_minimum_required(VERSION 3.10)
project(ursa_serial_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ursa_serial_driver.cpp
  src/crc.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN ursa_serial_driver::UrsaSerialDriver
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
