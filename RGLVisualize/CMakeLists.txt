cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

# This is required for Qt
if(POLICY CMP0100)
    cmake_policy(SET CMP0100 NEW)
endif()

project(RGLVisualize)

set(CMAKE_AUTOMOC ON)

# ROS 2 Lyrical / Ubuntu resolute: versionless gz-*-vendor packages
# (gz-msgs, gz-rendering, gz-gui are Ionic-era and Qt6-based).
find_package(gz_cmake_vendor REQUIRED)
find_package(gz-cmake REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
gz_find_package(gz-msgs REQUIRED)
find_package(gz_rendering_vendor REQUIRED)
gz_find_package(gz-rendering REQUIRED)
find_package(gz_gui_vendor REQUIRED)
gz_find_package(gz-gui REQUIRED)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

qt6_add_resources(resources_RCC RGLVisualize.qrc)

add_library(RGLVisualize SHARED
    RGLVisualize.cc
    ${resources_RCC}
)

target_link_libraries(RGLVisualize
    PRIVATE
    gz-msgs::core
    gz-rendering::core
    gz-gui::core
)

## Install libraries

install(TARGETS RGLVisualize
    DESTINATION RGLVisualize
)
