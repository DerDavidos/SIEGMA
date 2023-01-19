add_library(tmc2209_lib INTERFACE)
target_sources(tmc2209_lib INTERFACE tmc2209.c)
target_include_directories(tmc2209_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
