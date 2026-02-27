cmake_minimum_required(VERSION 3.8)
project(cloud_robot)

##################################################
# compiler settings
##################################################

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################################################
# dependencies
##################################################

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

##################################################
# pigpio
##################################################

find_path(PIGPIO_INCLUDE_DIR pigpio.h REQUIRED)
find_library(PIGPIO_LIBRARY pigpio REQUIRED)

##################################################
# includes
##################################################

include_directories(
  include
  ${PIGPIO_INCLUDE_DIR}
)

##################################################
# servo hardware interface
##################################################

add_library(servo_hw SHARED
  src/servo_hw.cpp
)

ament_target_dependencies(servo_hw
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(servo_hw
  ${PIGPIO_LIBRARY}
)

##################################################
# diffdrive hardware interface
##################################################

add_library(diffdrive_hw SHARED
  src/diffdrive_hw.cpp
)

ament_target_dependencies(diffdrive_hw
  rclcpp
  hardware_interface
  pluginlib
)

target_link_libraries(diffdrive_hw
  ${PIGPIO_LIBRARY}
)

##################################################
# plugin export
##################################################

pluginlib_export_plugin_description_file(
  hardware_interface plugin.xml
)

##################################################
# install C++ libs
##################################################

install(
  TARGETS
  servo_hw
  diffdrive_hw

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

##################################################
# install headers
##################################################

install(
  DIRECTORY include/
  DESTINATION include
)

##################################################
# install plugin.xml
##################################################

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

##################################################
# install python package
##################################################

ament_python_install_package(${PROJECT_NAME})

##################################################
# export
##################################################

ament_export_include_directories(
  include
  ${PIGPIO_INCLUDE_DIR}
)

ament_export_libraries(
  servo_hw
  diffdrive_hw
)

ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
)

##################################################

ament_package()
