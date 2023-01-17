 add_executable(limitSwitches integrationTest_limitSwitche.c) # creates executable
 target_include_directories(limitSwitches PUBLIC ${CMAKE_SOURCE_DIR}/src)
 target_link_libraries(limitSwitches
         limitSwitch_lib
         pico_stdlib
         dispenser_lib
         serialUART_lib
         tmc2209_lib
         motor_lib)
 make_to_output_file(limitSwitches)

 add_executable(motor integrationTest_motor.c) # creates executable
 target_include_directories(motor PUBLIC ${CMAKE_SOURCE_DIR}/src)
 target_link_libraries(motor
         dispenser_lib
         tmc2209_lib
         pico_stdlib
         serialUART_lib
         motor_lib
         limitSwitch_lib)
 make_to_output_file(motor)
