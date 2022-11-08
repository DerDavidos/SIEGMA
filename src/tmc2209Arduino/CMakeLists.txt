add_library(tmc2209Arduino_lib INTERFACE)
target_sources(tmc2209Arduino_lib INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/TMC2209.cpp)
target_include_directories(tmc2209Arduino_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR}
        serialUART_lib)
