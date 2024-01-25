cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(qb_softhand_industry_description VERSION 1.0.2 LANGUAGES CXX)
find_package(ament_cmake REQUIRED)

# Dependency Settings

ament_package()

# Installation

install(
  DIRECTORY
    urdf/
  DESTINATION
    share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY
    config/
  DESTINATION
    share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    rviz/
  DESTINATION
    share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY
    meshes/
  DESTINATION
    share/${PROJECT_NAME}/meshes
)
