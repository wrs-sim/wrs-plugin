if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_WRS_PLUGIN "Building WRSPlugin" ON)
if(NOT BUILD_WRS_PLUGIN)
  return()
endif()

if(UNIX)
  add_custom_target(
    symlink_to_wrs ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR} ${CNOID_BINARY_SHARE_DIR}/wrs)
endif()

install(DIRECTORY
  model
  project
  sample/WRS2020/model
  sample/WRS2024PRE/model
  sample/WRS2025/model
  share
  DESTINATION ${CNOID_SHARE_SUBDIR}/wrs)

add_subdirectory(src)
add_subdirectory(sample)
add_subdirectory(sample/WRS2020/controller)
add_subdirectory(sample/WRS2025/controller)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  add_subdirectory(controller)
else()
  message(STATUS "ament_cmake not found.")
endif()
