cmake_minimum_required(VERSION 3.13)
include(helpers.cmake)

add_definitions(-DARDUINO_ARCH_RP2040)

if (UNIT_TEST)
    message(NOTICE "INFO: Building Unit tests")

    project(SIEGMA C CXX ASM)

    fetch_git_submodules()

    include(CTest)

    add_subdirectory(extern/unity)
    add_subdirectory(test/unit)
else ()

    if (DEBUG)
        add_definitions(-DDEBUG)
    endif()

    if (LEFT)
        add_definitions(-DLEFT)
    elseif (RONDELL)
        add_definitions(-DRONDELL)
    elseif (RIGHT)
        add_definitions(-DRIGHT)
    endif()

    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
    fetch_git_submodules()

    include(extern/pico-sdk/pico_sdk_init.cmake)

    project(SIEGMA C CXX ASM)

    pico_sdk_init()

    if (TARGET tinyusb_device)
        include_directories(src)
        add_subdirectory(src/tmc2209)
        add_subdirectory(src/SerialUART)
        add_subdirectory(src/dispenser)
        add_subdirectory(src/limitSwitch)
        add_subdirectory(src/Rondell)
        add_subdirectory(src/motor)

        add_executable(main src/main.c) # creates executable
        target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(main
                pico_stdlib
                hardware_adc
                rondell_lib
                serialUART_lib
                tmc2209_lib
                dispenser_lib
                limitSwitch_lib
                motor_lib
                )
        make_to_output_file(main)
    elseif (PICO_ON_DEVICE)
        message(WARNING "not building Project because TinyUSB submodule is not initialized in the SDK")
    endif ()
endif ()
