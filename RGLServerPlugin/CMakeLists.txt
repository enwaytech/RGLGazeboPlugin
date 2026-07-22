cmake_minimum_required(VERSION 3.21)
project(RGLServerPlugin)
set(CMAKE_CXX_STANDARD 20)

# ROS 2 Lyrical / Ubuntu resolute ships Gazebo via versionless gz-*-vendor packages
# (gz-sim 10, gz-plugin 4). Use the vendor mechanism + versionless targets.
find_package(gz_cmake_vendor REQUIRED)
find_package(gz-cmake REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
gz_find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
gz_find_package(gz-sim REQUIRED)

include_directories(include)

add_library(RGLServerPluginManager SHARED src/RGLServerPluginManager.cc src/Mesh.cc src/Utils.cc src/Scene.cc)
target_include_directories(RGLServerPluginManager PRIVATE RobotecGPULidar)
target_link_libraries(RGLServerPluginManager
    PRIVATE gz-plugin::register
    PRIVATE gz-sim::core
    PRIVATE RobotecGPULidar
)
set_target_properties(RGLServerPluginManager PROPERTIES INSTALL_RPATH "$ORIGIN")

add_library(RGLServerPluginInstance SHARED src/RGLServerPluginInstance.cc src/Lidar.cc src/Utils.cc src/LidarPatternLoader.cc)
target_include_directories(RGLServerPluginInstance PRIVATE RobotecGPULidar)
target_link_libraries(RGLServerPluginInstance
    PRIVATE gz-plugin::register
    PRIVATE gz-sim::core
    PRIVATE RobotecGPULidar
)
set_target_properties(RGLServerPluginInstance PROPERTIES INSTALL_RPATH "$ORIGIN")

## Install libraries

install(TARGETS RGLServerPluginInstance RGLServerPluginManager
    DESTINATION RGLServerPlugin
)
# RobotecGPULidar needs to be installed manually as it is a dependent shared library
install(IMPORTED_RUNTIME_ARTIFACTS RobotecGPULidar
    DESTINATION RGLServerPlugin
)
