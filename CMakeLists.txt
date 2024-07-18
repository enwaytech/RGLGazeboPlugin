cmake_minimum_required(VERSION 3.21)
project(RGLGazeboPlugin)

find_package(ament_cmake REQUIRED)

if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}/install")
endif()

add_subdirectory(external)
add_subdirectory(RGLServerPlugin)
add_subdirectory(RGLVisualize)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in")

ament_package()
