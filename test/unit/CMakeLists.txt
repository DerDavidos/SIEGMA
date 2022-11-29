add_executable(unit-test_tmc2209
        ${CMAKE_CURRENT_LIST_DIR}/test_tmc2209.c)
target_include_directories(unit-test_tmc2209 PUBLIC
        ${CMAKE_SOURCE_DIR}/extern
        ${CMAKE_SOURCE_DIR}/src/tmc2209
        ${CMAKE_SOURCE_DIR}/src/SerialUART)
target_link_libraries(unit-test_tmc2209 unity)
add_test(unit-test_tmc2209 unit-test_tmc2209)
