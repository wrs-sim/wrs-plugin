if(NOT ENABLE_GUI)
  return()
endif()

if(NOT BUILD_SCENE_EFFECTS_PLUGIN OR NOT BUILD_WRS2018 OR NOT BUILD_HAIRO_WORLD_PLUGIN)
  return()
endif()

if(NOT BUILD_WRS2020)
  return()
endif()

install(DIRECTORY
  project
  share
  DESTINATION ${CHOREONOID_SHARE_SUBDIR}/WRS2020)

add_subdirectory(src)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  add_subdirectory(controller)
else()
  message(STATUS "ament_cmake not found.")
endif()