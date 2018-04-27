set(target "pb_remote_client_${PLATFORM}_${SOFTDEVICE}")

add_executable(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${MBTLE_SOURCE_DIR}/examples/nrf_mesh_sdk.c"
    "${MBTLE_SOURCE_DIR}/examples/common/src/rtt_input.c"
    ${PB_REMOTE_CLIENT_SOURCE_FILES}
    ${PROV_PROVISIONER_SOURCE_FILES}
    ${PROV_COMMON_SOURCE_FILES}
    ${ACCESS_SOURCE_FILES}
    ${WEAK_SOURCE_FILES}
    ${${PLATFORM}_SOURCE_FILES})

target_include_directories(${target} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/examples"
    "${CMAKE_SOURCE_DIR}/examples/common/include"
    "${SDK_ROOT}/components/drivers_nrf/delay"
    ${PB_REMOTE_CLIENT_INCLUDE_DIRS}
    ${PROV_INCLUDE_DIRS}
    ${ACCESS_INCLUDE_DIRS}
    ${${BOARD}_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS})

set_target_link_options(${target}
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/${PLATFORM}_${SOFTDEVICE})

target_compile_options(${target} PUBLIC
    ${${ARCH}_DEFINES})

target_compile_definitions(${target} PUBLIC
    ${USER_DEFINITIONS}
    ${${PLATFORM}_DEFINES}
    ${${SOFTDEVICE}_DEFINES}
    ${${BOARD}_DEFINES})

target_link_libraries(${target}
    mesh_core_${PLATFORM}
    rtt_${PLATFORM}
    uECC_${PLATFORM})

create_hex(${target})
add_flash_target(${target})

get_property(core_include_dirs TARGET mesh_core_${PLATFORM} PROPERTY INCLUDE_DIRECTORIES)
get_property(target_include_dirs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
add_pc_lint(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${target_include_dirs};${core_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES}")
add_ses_project(${target})
