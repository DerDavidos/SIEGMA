add_library(motor_lib INTERFACE)
target_sources(motor_lib INTERFACE motor.c)
target_include_directories(motor_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR}
        serialUART_lib
        tmc2209_lib)
