function(fetch_git_submodules)
    find_package(Git QUIET)
    if (GIT_FOUND AND EXISTS "${CMAKE_CURRENT_LIST_DIR}/.git")
        # Update Submodules
        option(GIT_SUBMODULE "Check submodules build" ON)
        if (GIT_SUBMODULE)
            message(STATUS "Submodule update")
            SET(GIT_SUBMOD_RESULT "128")
            SET(SUBMODULE_TRIES 0)
            while (GIT_SUBMOD_RESULT EQUAL "128" AND NOT SUBMODULE_TRIES EQUAL 10)
                execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
                if (GIT_SUBMOD_RESULT EQUAL "128")
                    message(NOTICE "INFO: Could not update submodule, trying again...")
                endif ()
                MATH(EXPR SUBMODULE_TRIES "${SUBMODULE_TRIES}+1")
            endwhile ()
            if (NOT GIT_SUBMOD_RESULT EQUAL "0")
                message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}")
            endif ()
        endif ()
    endif ()
endfunction()


MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list *.h)
    list(FILTER new_list EXCLUDE REGEX .*/pico-sdk/.* )
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

MACRO(SUBDIRLIST result curdir)
    FILE(GLOB children RELATIVE ${curdir} ${curdir}/*)
    SET(dirlist "")
    FOREACH(child ${children})
        IF(IS_DIRECTORY ${curdir}/${child})
            LIST(APPEND dirlist ${child})
        ENDIF()
    ENDFOREACH()
    SET(${result} ${dirlist})
ENDMACRO()

function(make_to_output_file target)
    # enable usb output, disable uart output
    pico_enable_stdio_usb(${target} 1)
    pico_enable_stdio_uart(${target} 0)
    # create map/bin/hex/uf2 file etc.
    pico_add_uf2_output(${target})
    # move u2f files after build to out directory
    file(RELATIVE_PATH relative_path ${CMAKE_SOURCE_DIR} ${CMAKE_CURRENT_LIST_DIR})
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_BINARY_DIR}/${relative_path}/${target}.uf2
            ${CMAKE_SOURCE_DIR}/out/${relative_path}/${target}.uf2)
endfunction()
