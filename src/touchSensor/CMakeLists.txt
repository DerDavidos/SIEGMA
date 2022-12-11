add_library(touchSensor_lib INTERFACE)
target_sources(touchSensor_lib INTERFACE touchSensor.c)
target_include_directories(touchSensor_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
