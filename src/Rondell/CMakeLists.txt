add_library(rondell_lib INTERFACE)
target_sources(rondell_lib INTERFACE
        rondell.c)
target_include_directories(rondell_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})