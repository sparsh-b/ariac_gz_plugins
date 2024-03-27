cmake_minimum_required(VERSION 3.8)
project(ariac_gz_plugins)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-sensors8 REQUIRED)

set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  ros_gz_bridge
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(AriacCameraPlugin SHARED src/ariac_camera_plugin.cpp)

target_include_directories(AriacCameraPlugin PUBLIC include)

target_link_libraries(AriacCameraPlugin
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim8::gz-sim8
  gz-sensors8::gz-sensors8
)

ament_target_dependencies(AriacCameraPlugin 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_export_libraries(AriacCameraPlugin)

install(
  TARGETS 
  AriacCameraPlugin
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
