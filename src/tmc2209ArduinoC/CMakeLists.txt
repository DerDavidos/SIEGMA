add_library(tmc2209ArduinoC_lib INTERFACE)
target_sources(tmc2209ArduinoC_lib INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/TMC2209.c
        ${CMAKE_CURRENT_LIST_DIR}/SerialUART.c)
target_include_directories(tmc2209ArduinoC_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
