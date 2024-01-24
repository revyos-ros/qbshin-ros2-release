cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_softhand_industry_msgs)

# Dependency Settings

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        msg/ConnectionState.msg
        msg/State.msg
        msg/ResourceData.msg
        msg/StateStamped.msg
        msg/DeviceConnectionInfo.msg
    DEPENDENCIES
        std_msgs
)

include_directories(include
)

ament_export_include_directories(include)
ament_export_dependencies(std_msgs)
ament_package()

# Installation
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    msg/
  DESTINATION
    share/${PROJECT_NAME}/msg
)