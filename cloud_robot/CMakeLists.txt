cmake_minimum_required(VERSION 3.8)
project(cloud_robot)

set(CMAKE_CXX_STANDARD 17)
set(BUILD_SHARED_LIBS ON)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Library
add_library(${PROJECT_NAME} SHARED
  src/servo_hw.cpp
  src/diffdrive_hw.cpp
  src/track.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    /usr/local/include
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link pigpio
target_link_libraries(${PROJECT_NAME} pigpiod_if2)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

# Install
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
# Instalar URDF/XACRO
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
