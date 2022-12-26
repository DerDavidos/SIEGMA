 add_executable(limitSwitches limitSwitches.c) # creates executable
 target_include_directories(limitSwitches PUBLIC ${CMAKE_SOURCE_DIR}/src)
 target_link_libraries(limitSwitches
         limitSwitch_lib
         pico_stdlib)
 make_to_output_file(limitSwitches)

 add_executable(dispenser dispenser.c) # creates executable
 target_include_directories(dispenser PUBLIC ${CMAKE_SOURCE_DIR}/src)
 target_link_libraries(dispenser
         dispenser_lib
         tmc2209_lib
         pico_stdlib
         serialUART_lib
         motor_lib)
 make_to_output_file(dispenser)
