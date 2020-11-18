# Common options, defines, includes and source files for the DFU example targets
add_compile_options(${${ARCH}_DEFINES})
set(DFU_DEFINITIONS
    ${USER_DEFINITIONS}
    -DUSE_APP_CONFIG
    -DCONFIG_APP_IN_CORE
    ${${PLATFORM}_DEFINES}
    ${${SOFTDEVICE}_DEFINES}
    ${${BOARD}_DEFINES})

include_directories(
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MBTLE_SOURCE_DIR}/examples"
    "${CMAKE_SOURCE_DIR}/examples/common/include"
    ${BLE_SOFTDEVICE_SUPPORT_INCLUDE_DIRS}
    ${CONFIG_SERVER_INCLUDE_DIRS}
    ${HEALTH_SERVER_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${BOARD}_INCLUDE_DIRS}
    ${SERIAL_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

link_libraries(
    rtt_${PLATFORM}
    uECC_${PLATFORM})

set(COMMON_MESH_FILES
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_provisionee.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/simple_hal.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_app_utils.c"
    ${BLE_SOFTDEVICE_SUPPORT_SOURCE_FILES}
    ${ACCESS_SOURCE_FILES}
    ${CONFIG_SERVER_SOURCE_FILES}
    ${HEALTH_SERVER_SOURCE_FILES}
    ${PROV_PROVISIONEE_SOURCE_FILES}
    ${PROV_COMMON_SOURCE_FILES}
    ${PROV_BEARER_ADV_SOURCE_FILES}
    ${DFU_SOURCE_FILES}
    ${WEAK_SOURCE_FILES}
    ${MESH_STACK_SOURCE_FILES}
    ${MESH_CORE_SOURCE_FILES}
    ${MESH_BEARER_SOURCE_FILES}
    ${MESH_APP_TIMER_SOURCE_FILES}
    ${${PLATFORM}_SOURCE_FILES}
    ${${nRF5_SDK_VERSION}_SOURCE_FILES})

####### DFU example with an active serial connection:
set(target "dfu_${PLATFORM}_${SOFTDEVICE}")

if ((NOT PLATFORM MATCHES "nrf52810") AND (NOT PLATFORM MATCHES "nrf52820"))

add_executable(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    ${SERIAL_SOURCE_FILES}
    ${COMMON_MESH_FILES})

set_target_link_options(${target}
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/${PLATFORM}_${SOFTDEVICE})

target_compile_definitions(${target} PUBLIC ${DFU_DEFINITIONS})
target_compile_definitions(${target} PUBLIC NRF_MESH_SERIAL_ENABLE)

create_hex(${target})
add_flash_target(${target})

get_property(target_include_dirs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
add_pc_lint(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${target_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES};-DNRF_MESH_SERIAL_ENABLE=1")

add_ses_project(${target})

endif()

####### DFU example without an active serial connection:
set(target "dfu_no_serial_${PLATFORM}_${SOFTDEVICE}")

add_executable(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    ${COMMON_MESH_FILES})

set_target_link_options(${target}
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/${PLATFORM}_${SOFTDEVICE})

target_compile_definitions(${target} PUBLIC ${DFU_DEFINITIONS})

create_hex(${target})
add_flash_target(${target})

get_property(target_include_dirs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
add_pc_lint(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${target_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES};-DNRF_MESH_SERIAL_ENABLE=0")

add_ses_project(${target})
