################################################################################
# Configurations to build the Mesh Bootloader
################################################################################

option(BUILD_BOOTLOADER OFF "Build the Mesh Bootloader?")

if (BUILD_BOOTLOADER)
    if (NOT CMAKE_BUILD_TYPE STREQUAL "MinSizeRel")
        message(STATUS "Build type is \"${CMAKE_BUILD_TYPE}\". "
            "The bootloader must be compiled with \"MinSizeRel\" in order to fit.")
        set(CMAKE_BUILD_TYPE "MinSizeRel")
        message(STATUS "Bootloader Build type is now \"${CMAKE_BUILD_TYPE}\". ")
    endif ()

    option(BOOTLOADER_DEBUG_ENABLE OFF
        "Build bootloader with additional debug information (RTT logging)?")

    message(STATUS "Mesh Bootloader build enabled")

    set(BOOTLOADER_SOURCE_FILES_LINT
        "${CMAKE_CURRENT_SOURCE_DIR}/src/bootloader.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/bootloader_app_bridge.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/bootloader_info.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/bootloader_rtc.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/bootloader_util.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/dfu_bank.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/dfu_mesh.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/dfu_transfer_mesh.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/dfu_util.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/fifo.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_packet.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_flash.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/radio_control.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/rand.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/transport.c")

    set(BOOTLOADER_SOURCE_FILES
        ${BOOTLOADER_SOURCE_FILES_LINT}
        # This files are not linted as they are 3 parties files.
        "${CMAKE_CURRENT_SOURCE_DIR}/src/uECC.c"
        "${SDK_ROOT}/components/libraries/sha256/sha256.c" CACHE INTERNAL "")

    set(BOOTLOADER_SERIAL_SOURCE_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/src/serial_handler_uart.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_aci.c" CACHE INTERNAL "")

    set(BOOTLOADER_INCLUDE_DIRS
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
        "${SDK_ROOT}/components/libraries/sha256" CACHE INTERNAL "")

    set(BOOTLOADER_DEFINES
        -DNORDIC_SDK_VERSION=17
        -DBOOTLOADER
        -DRBC_MESH_PACKET_POOL_SIZE=32
        -DuECC_CURVE=uECC_secp256r1
        -DuECC_PLATFORM=uECC_arm CACHE STRING "Bootloader compile definitions")

    set(__target "mesh_bootloader_${TOOLCHAIN}_${PLATFORM}")
    set(__target_serial "mesh_bootloader_serial_${TOOLCHAIN}_${PLATFORM}")

    ################################################################################
    # Without serial interface
    ################################################################################

    add_executable(${__target}
        ${BOOTLOADER_SOURCE_FILES}
        ${${PLATFORM}_SOURCE_FILES})

    target_include_directories(${__target} PUBLIC
        ${BOOTLOADER_INCLUDE_DIRS}
        ${${SOFTDEVICE}_INCLUDE_DIRS}
        ${${PLATFORM}_INCLUDE_DIRS}
        ${${BOARD}_INCLUDE_DIRS})

    target_compile_definitions(${__target} PUBLIC
        ${USER_DEFINITIONS}
        ${BOOTLOADER_DEFINES}
        ${${PLATFORM}_DEFINES}
        ${${BOARD}_DEFINES})

    target_compile_options(${__target} PUBLIC
        ${${ARCH}_DEFINES})

    set_target_link_options(${__target}
        ${CMAKE_CURRENT_SOURCE_DIR}/linker/bootloader_${PLATFORM})

    if (BOOTLOADER_DEBUG_ENABLE)
        target_link_libraries(${__target} PUBLIC
            rtt_${PLATFORM})
        target_compile_definitions(${__target} PUBLIC
            -DRTT_LOG=1)
    endif (BOOTLOADER_DEBUG_ENABLE)

    create_hex(${__target})

    ################################################################################
    # With serial interface
    ################################################################################

    add_executable(${__target_serial}
        ${BOOTLOADER_SOURCE_FILES}
        ${BOOTLOADER_SERIAL_SOURCE_FILES}
        ${${PLATFORM}_SOURCE_FILES})

    target_include_directories(${__target_serial} PUBLIC
        ${BOOTLOADER_INCLUDE_DIRS}
        ${${SOFTDEVICE}_INCLUDE_DIRS}
        ${${PLATFORM}_INCLUDE_DIRS}
        ${${BOARD}_INCLUDE_DIRS})

    target_compile_definitions(${__target_serial} PUBLIC
        ${USER_DEFINITIONS}
        -DRBC_MESH_SERIAL=1
        ${BOOTLOADER_DEFINES}
        ${${PLATFORM}_DEFINES}
        ${${BOARD}_DEFINES})

    target_compile_options(${__target_serial} PUBLIC
        ${${ARCH}_DEFINES})

    target_link_libraries(${__target_serial} PUBLIC
        rtt_${PLATFORM})

    set_target_link_options(${__target_serial}
        ${CMAKE_CURRENT_SOURCE_DIR}/linker/bootloader_${PLATFORM})

    if (BOOTLOADER_DEBUG_ENABLE)
        target_link_libraries(${__target_serial} PUBLIC
            rtt_${PLATFORM})
        target_compile_definitions(${__target_serial} PUBLIC
            -DRTT_LOG=1)
    endif (BOOTLOADER_DEBUG_ENABLE)

    create_hex(${__target_serial})

    # Enable "poor mans LTO" for ARMCC
    if (TOOLCHAIN STREQUAL "armcc")
        set(__feedback_file "${CMAKE_CURRENT_BINARY_DIR}/feedback.txt")
        set(__feedback_deps_file "${CMAKE_CURRENT_BINARY_DIR}/feedback.deps")

        file(TO_NATIVE_PATH ${__feedback_file} __feedback_file_native)
        if (NOT EXISTS ${__feedback_file})
            execute_process(
                COMMAND ${CMAKE_COMMAND} -E touch ${__feedback_file} ${__feedback_deps_file}
                VERBATIM)
        endif (NOT EXISTS ${__feedback_file})

        set_source_files_properties(${BOOTLOADER_SOURCE_FILES} PROPERTIES
            COMPILE_FLAGS "--feedback=${__feedback_file_native}"
            OBJECT_DEPENDS ${__feedback_deps_file})

        set_source_files_properties(${BOOTLOADER_SERIAL_SOURCE_FILES} PROPERTIES
            COMPILE_FLAGS "--feedback=${__feedback_file_native}"
            OBJECT_DEPENDS ${__feedback_deps_file})

        target_link_libraries(${__target} PRIVATE "--feedback=\"${__feedback_file_native}\"")
        target_link_libraries(${__target_serial} PRIVATE "--feedback=\"${__feedback_file_native}\"")

        add_custom_target(recompile_bootloaders
            COMMAND ${CMAKE_COMMAND} -E touch ${__feedback_deps_file}
            COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${__target}
            COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ${__target_serial}
            USES_TERMINAL
            VERBATIM)
    endif (TOOLCHAIN STREQUAL "armcc")

    ################################################################################
    # PC-Lint target
    ################################################################################
    set(LINT_FEATURE_INCLUSION_DEFINES
        "-DRBC_MESH_SERIAL=1")

    add_pc_lint(mesh_bootloader_${PLATFORM}
        "${BOOTLOADER_SOURCE_FILES_LINT};${BOOTLOADER_SERIAL_SOURCE_FILES}"
        "${BOOTLOADER_INCLUDE_DIRS};${${PLATFORM}_INCLUDE_DIRS};${${SOFTDEVICE}_INCLUDE_DIRS};${${BOARD}_INCLUDE_DIRS}"
        "${BOOTLOADER_DEFINES};${${PLATFORM}_DEFINES};${${BOARD}_DEFINES};${LINT_FEATURE_INCLUSION_DEFINES}")

endif (BUILD_BOOTLOADER)
