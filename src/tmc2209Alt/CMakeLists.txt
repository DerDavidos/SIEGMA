add_library(tmc2209Alt_lib INTERFACE)
target_sources(tmc2209Alt_lib INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/tmc2209Alt.c
        ${CMAKE_CURRENT_LIST_DIR}/common.c
        ${CMAKE_CURRENT_LIST_DIR}/tmc_interface.c
        ${CMAKE_CURRENT_LIST_DIR}/tmchal.h)
target_include_directories(tmc2209Alt_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
