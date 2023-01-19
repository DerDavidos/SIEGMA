add_library(limitSwitch_lib INTERFACE)
target_sources(limitSwitch_lib INTERFACE limitSwitch.c)
target_include_directories(limitSwitch_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
