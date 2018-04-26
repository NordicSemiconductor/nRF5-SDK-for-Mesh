set(target "light_switch_proxy_server_${PLATFORM}_${SOFTDEVICE}")

add_executable(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${CMAKE_SOURCE_DIR}/mesh/stack/src/mesh_stack.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_provisionee.c"
    "${MBTLE_SOURCE_DIR}/examples/common/src/rtt_input.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/simple_hal.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_app_utils.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_adv.c"
    "${CMAKE_SOURCE_DIR}/mesh/prov/src/nrf_mesh_prov_bearer_gatt.c"
    "${SDK_ROOT}/components/libraries/util/app_util_platform.c"
    "${SDK_ROOT}/components/ble/common/ble_advdata.c"
    "${SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c"
    ${WEAK_SOURCE_FILES}
    ${MESH_CORE_SOURCE_FILES}
    ${MESH_BEARER_SOURCE_FILES}
    ${MESH_GATT_SOURCE_FILES}
    ${SIMPLE_ON_OFF_SERVER_SOURCE_FILES}
    ${CONFIG_SERVER_SOURCE_FILES}
    ${HEALTH_SERVER_SOURCE_FILES}
    ${ACCESS_SOURCE_FILES}
    ${PROV_PROVISIONEE_SOURCE_FILES}
    ${PROV_COMMON_SOURCE_FILES}
    ${${PLATFORM}_SOURCE_FILES}
    ${${nRF5_SDK_VERSION}_SOURCE_FILES})

target_include_directories(${target} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../include"
    "${CMAKE_SOURCE_DIR}/examples/common/include"
    "${CMAKE_SOURCE_DIR}/external/rtt/include"
    "${SDK_ROOT}/components/drivers_nrf/delay"
    "${SDK_ROOT}/components/ble/common"
    "${SDK_ROOT}/components/softdevice/common"
    "${SDK_ROOT}/components/libraries/experimental_section_vars"
    "${SDK_ROOT}/components/libraries/experimental_log"
    "${SDK_ROOT}/components/libraries/experimental_log/src"
    "${SDK_ROOT}/components/libraries/strerror"
    ${SIMPLE_ON_OFF_SERVER_INCLUDE_DIRS}
    ${CONFIG_SERVER_INCLUDE_DIRS}
    ${HEALTH_SERVER_INCLUDE_DIRS}
    ${MESH_GATT_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${BOARD}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

set_target_link_options(${target}
    ${CMAKE_CURRENT_SOURCE_DIR}/linker/${PLATFORM}_${SOFTDEVICE})

target_compile_options(${target} PUBLIC
    ${${ARCH}_DEFINES})

target_compile_definitions(${target} PUBLIC
    ${USER_DEFINITIONS}
    -DUSE_APP_CONFIG=1
    -DMESH_FEATURE_GATT=1
    -DGATT_PROXY=1
    ${${PLATFORM}_DEFINES}
    ${${SOFTDEVICE}_DEFINES}
    ${${BOARD}_DEFINES})

target_link_libraries(${target}
    rtt_${PLATFORM}
    uECC_${PLATFORM})

create_hex(${target})
add_flash_target(${target})

get_property(target_include_dirs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
add_pc_lint(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${target_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES}")
add_ses_project(${target})
