set(LIGHT_LC_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_client.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_setup_server.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_fsm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_light_pi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_state_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_occupancy_sensor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_ambient_light_sensor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_sensor_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_mc.c")

set(LIGHT_LC_CLIENT_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_client.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_state_utils.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")

set(LIGHT_LC_CLIENT_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MODEL_COMMON_INCLUDE_DIRS}" CACHE INTERNAL "")

set(LIGHT_LC_SETUP_SERVER_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_setup_server.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_fsm.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_light_pi.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_state_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_occupancy_sensor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_ambient_light_sensor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_sensor_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/light_lc_mc.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")

set(LIGHT_LC_SETUP_SERVER_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MODEL_COMMON_INCLUDE_DIRS}" CACHE INTERNAL "")

set(target_include_dirs
    ${LIGHT_LC_CLIENT_INCLUDE_DIRS}
    ${LIGHT_LC_SETUP_SERVER_INCLUDE_DIRS}
    ${LIGHT_LIGHTNESS_SETUP_SERVER_INCLUDE_DIRS}
    ${SCENE_SETUP_SERVER_INCLUDE_DIRS}
    ${SENSOR_CLIENT_INCLUDE_DIRS}
    ${GENERIC_PONOFF_CLIENT_INCLUDE_DIRS}
    ${GENERIC_ONOFF_SERVER_INCLUDE_DIRS}
    ${GENERIC_LEVEL_SERVER_INCLUDE_DIRS}
    ${CONFIG_SERVER_INCLUDE_DIRS}
    ${HEALTH_SERVER_INCLUDE_DIRS}
    ${ACCESS_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/examples/templates)

add_pc_lint(light_lc_server_${PLATFORM}
    "${LIGHT_LC_SOURCE_FILES}"
    "${target_include_dirs};${CMAKE_SOURCE_DIR}/mesh/test/include"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DLIGHT_LC_SETUP_SERVER_INSTANCES_MAX=1;-DSCENE_SETUP_SERVER_INSTANCES_MAX=0")

add_pc_lint(light_lc_server_with_scene_${PLATFORM}
    "${LIGHT_LC_SOURCE_FILES}"
    "${target_include_dirs};${CMAKE_SOURCE_DIR}/mesh/test/include"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DLIGHT_LC_SETUP_SERVER_INSTANCES_MAX=1;-DSCENE_SETUP_SERVER_INSTANCES_MAX=1")
