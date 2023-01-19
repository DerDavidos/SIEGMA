add_executable(unit-test_dispenser
        ${CMAKE_CURRENT_LIST_DIR}/test_dispenser.c

        ${CMAKE_CURRENT_LIST_DIR}/dummies/hardware/gpio.c
        ${CMAKE_CURRENT_LIST_DIR}/dummies/pico/time.c
        ${CMAKE_CURRENT_LIST_DIR}/dummies/serialUART/serialUART.h
        ${CMAKE_CURRENT_LIST_DIR}/dummies/tmc2209/tmc2209.c

        ${CMAKE_SOURCE_DIR}/src/dispenser/dispenser.c
        ${CMAKE_SOURCE_DIR}/src/motor/motor.c
        ${CMAKE_SOURCE_DIR}/src/limitSwitch/limitSwitch.c
)
target_include_directories(unit-test_dispenser PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/dummies
        ${CMAKE_CURRENT_LIST_DIR}/dummies/serialUART
        ${CMAKE_CURRENT_LIST_DIR}/dummies/tmc2209

        ${CMAKE_SOURCE_DIR}/extern

        ${CMAKE_SOURCE_DIR}/src/dispenser
        ${CMAKE_SOURCE_DIR}/src/limitSwitch
        ${CMAKE_SOURCE_DIR}/src/motor
        )
target_link_libraries(unit-test_dispenser
        unity
        )
add_test(unit-test_dispenser unit-test_dispenser)
