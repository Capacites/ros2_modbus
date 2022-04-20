cmake_minimum_required(VERSION 3.5)
project(ros_modbus)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE debug)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ros_modbus_msgs REQUIRED)
find_library(modbus REQUIRED)
find_library(yaml-cpp REQUIRED)

# main code is a node class + standalone executable
add_executable(modbus_node src/modbus_node.cpp src/modbus_interface.cpp)
target_include_directories(modbus_node PRIVATE include)
target_link_libraries(modbus_node ${rclcpp_LIBRARIES} yaml-cpp modbus)
ament_target_dependencies(modbus_node std_msgs ros_modbus_msgs)


# Install
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS src/modbus_node.py DESTINATION lib/${PROJECT_NAME})

install(TARGETS modbus_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()
