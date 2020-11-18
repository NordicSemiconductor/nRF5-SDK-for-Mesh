set(target "light_ctl_lc_server_${PLATFORM}_${SOFTDEVICE}")

add_executable(${target}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/app_light_ctl.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/app_light_lightness.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/app_light_lc.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/app_transition.c"
    "${CMAKE_SOURCE_DIR}/mesh/stack/src/mesh_stack.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_provisionee.c"
    "${MBTLE_SOURCE_DIR}/examples/common/src/rtt_input.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_app_utils.c"
    ${MODEL_CONFIG_FILE_SOURCE_FILES}
    ${PWM_UTILS_SOURCE_FILES}
    ${BLE_SOFTDEVICE_SUPPORT_SOURCE_FILES}
    ${WEAK_SOURCE_FILES}
    ${MESH_CORE_SOURCE_FILES}
    ${MESH_BEARER_SOURCE_FILES}
    ${MESH_GATT_SOURCE_FILES}
    ${CONFIG_SERVER_SOURCE_FILES}
    ${HEALTH_SERVER_SOURCE_FILES}
    ${GENERIC_ONOFF_SERVER_SOURCE_FILES}
    ${GENERIC_PONOFF_SERVER_SOURCE_FILES}
    ${GENERIC_LEVEL_SERVER_SOURCE_FILES}
    ${LIGHT_CTL_SETUP_SERVER_SOURCE_FILES}
    ${LIGHT_LIGHTNESS_SETUP_SERVER_SOURCE_FILES}
    ${LIGHT_LC_SETUP_SERVER_SOURCE_FILES}
    ${ACCESS_SOURCE_FILES}
    ${MESH_APP_TIMER_SOURCE_FILES}
    ${PROV_PROVISIONEE_SOURCE_FILES}
    ${PROV_COMMON_SOURCE_FILES}
    ${PROV_BEARER_ADV_SOURCE_FILES}
    ${PROV_BEARER_GATT_SOURCE_FILES}
    ${${PLATFORM}_SOURCE_FILES}
    ${${nRF5_SDK_VERSION}_SOURCE_FILES})

target_include_directories(${target} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_SOURCE_DIR}/examples/common/include"
    "${CMAKE_SOURCE_DIR}/examples/common/include"
    "${CMAKE_SOURCE_DIR}/external/rtt/include"
    ${PWM_UTILS_INCLUDE_DIRS}
    ${BLE_SOFTDEVICE_SUPPORT_INCLUDE_DIRS}
    ${CONFIG_SERVER_INCLUDE_DIRS}
    ${HEALTH_SERVER_INCLUDE_DIRS}
    ${GENERIC_ONOFF_SERVER_INCLUDE_DIRS}
    ${GENERIC_PONOFF_SERVER_INCLUDE_DIRS}
    ${GENERIC_LEVEL_SERVER_INCLUDE_DIRS}
    ${LIGHT_CTL_SETUP_SERVER_INCLUDE_DIRS}
    ${LIGHT_LIGHTNESS_SETUP_SERVER_INCLUDE_DIRS}
    ${LIGHT_LC_SETUP_SERVER_INCLUDE_DIRS}
    ${SCENE_SETUP_SERVER_INCLUDE_DIRS}
    ${SENSOR_CLIENT_INCLUDE_DIRS}
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
    -DUSE_APP_CONFIG
    -DCONFIG_APP_IN_CORE
    ${${PLATFORM}_DEFINES}
    ${${SOFTDEVICE}_DEFINES}
    ${${BOARD}_DEFINES})

target_link_libraries(${target}
    rtt_${PLATFORM}
    uECC_${PLATFORM})

create_hex(${target})
add_flash_target(${target})


get_property(target_include_dirs TARGET ${target} PROPERTY INCLUDE_DIRECTORIES)
get_property(target_compile_defs TARGET ${target} PROPERTY COMPILE_DEFINITIONS)

set(lint_source_files
    "${CMAKE_CURRENT_SOURCE_DIR}/src/main.c")

add_pc_lint(${target}
    "${lint_source_files}"
    "${target_include_dirs}"
    "${target_compile_defs}")

add_ses_project(${target})
