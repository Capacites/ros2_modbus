cmake_minimum_required(VERSION 3.5)
project(ros_modbus_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME} "msg/Modbus.msg" "msg/State.msg" DEPENDENCIES std_msgs)

ament_package()
