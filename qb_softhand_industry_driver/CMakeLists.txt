cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_softhand_industry_driver VERSION 1.0.2 LANGUAGES CXX)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qb_softhand_industry_srvs REQUIRED)

# Dependency Settings

include_directories(include
)

# Build Settings
add_executable(qb_softhand_industry_communication_handler
  src/main.cpp
  src/qb_softhand_industry_communication_handler.cpp
)

ament_target_dependencies(qb_softhand_industry_communication_handler
  PUBLIC
    "qb_softhand_industry_srvs"
    "rclcpp"
    "qb_softhand_industry_msgs"
)

target_link_libraries(qb_softhand_industry_communication_handler
  PUBLIC
  ${${PROJECT_NAME}_SOURCE_DIR}/libs/libqbrobotics_driver_IN.a
  ${${PROJECT_NAME}_SOURCE_DIR}/libs/libSerial.a
)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp qb_softhand_industry_srvs)
ament_package()

# Installation
install(
  TARGETS qb_softhand_industry_communication_handler
  RUNTIME DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION
    include/${PROJECT_NAME}
)

install(
  DIRECTORY
    launch/
  DESTINATION
    share/${PROJECT_NAME}/launch
)

link_directories(libs)

# C++ Settings
if (CMAKE_VERSION VERSION_LESS "3.1")
  target_compile_options(qb_softhand_industry_communication_handler
    PUBLIC
      "-std=c++17"
  )
else ()
  set_property(
    TARGET
    qb_softhand_industry_communication_handler
    PROPERTY CXX_STANDARD 17
  )

  set_property(
    TARGET
      qb_softhand_industry_communication_handler
    PROPERTY CXX_STANDARD_REQUIRED ON
  )
endif ()