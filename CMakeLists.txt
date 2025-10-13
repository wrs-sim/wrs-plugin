if(NOT ENABLE_GUI)
  return()
endif()

option(BUILD_WRS_PLUGIN "Building WRSPlugin" ON)
if(NOT BUILD_WRS_PLUGIN)
  return()
endif()

add_subdirectory(sample)