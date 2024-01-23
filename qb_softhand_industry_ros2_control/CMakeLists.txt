cmake_minimum_required(VERSION 3.16)
project(qb_softhand_industry_ros2_control LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  transmission_interface
  qb_softhand_industry_msgs
  qb_softhand_industry_srvs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  qb_softhand_industry_ros2_control
  SHARED
  hardware/qb_softhand_industry_ros2_control.cpp
)
target_compile_features(qb_softhand_industry_ros2_control PUBLIC cxx_std_17)
target_include_directories(qb_softhand_industry_ros2_control PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/qb_softhand_industry_ros2_control>
)
ament_target_dependencies(
  qb_softhand_industry_ros2_control PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "QB_SOFTHAND_INDUSTRY_ROS2_CONTROL_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface qb_softhand_industry_ros2_control.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/qb_softhand_industry_ros2_control
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/qb_softhand_industry_ros2_control
)
install(TARGETS qb_softhand_industry_ros2_control
  EXPORT export_qb_softhand_industry_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_qb_softhand_industry_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
