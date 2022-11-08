add_library(serialUART_lib INTERFACE)
target_sources(serialUART_lib INTERFACE
        SerialUART.c)
target_include_directories(serialUART_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
