cmake_minimum_required(VERSION 3.13)
include(helpers.cmake)

add_definitions(-DARDUINO_ARCH_RP2040)

if (UNIT_TEST)
    message(NOTICE "INFO: Building Unit tests")

    project(SIEGMA C CXX ASM)

    fetch_git_submodules()

    include(CTest)

    add_subdirectory(extern/unity)
else ()
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
    fetch_git_submodules()

    include(extern/pico-sdk/pico_sdk_init.cmake)

    project(SIEGMA C CXX ASM)

    pico_sdk_init()

    if (TARGET tinyusb_device)
        include_directories(src)
#        add_subdirectory(src/tmc2209Arduino)
        add_subdirectory(src/tmc2209ArduinoC)

        add_executable(main src/main.c) # creates executable
        target_include_directories(main PUBLIC ${CMAKE_SOURCE_DIR}/src)
        target_link_libraries(main
                pico_stdlib
#                tmc2209Arduino_lib
                tmc2209ArduinoC_lib)
        make_to_output_file(main)
    elseif (PICO_ON_DEVICE)
        message(WARNING "not building Project because TinyUSB submodule is not initialized in the SDK")
    endif ()
endif ()
