cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_softhand_industry_srvs)

# Dependency Settings

find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_srvs REQUIRED)
find_package(qb_softhand_industry_msgs REQUIRED)
set(CMAKE_CXX_STANDARD 14)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
        srv/Trigger.srv
        srv/SetCommand.srv
        srv/GetMeasurements.srv
        srv/SetCommands.srv
    DEPENDENCIES
      builtin_interfaces
        
)

include_directories(include
)

ament_export_include_directories(include)
ament_export_dependencies(std_srvs qb_softhand_industry_msgs)
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
    srv/
  DESTINATION
    share/${PROJECT_NAME}/srv
)