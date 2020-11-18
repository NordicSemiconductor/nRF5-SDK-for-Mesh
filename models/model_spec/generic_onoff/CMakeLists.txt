set(GENERIC_ONOFF_CLIENT_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_client.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")
set(GENERIC_ONOFF_CLIENT_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MODEL_COMMON_INCLUDE_DIRS}" CACHE INTERNAL "")

# Use this if Generic OnOff Server is used as part of the extended model, where extended model
# stores the states
set(GENERIC_ONOFF_SERVER_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_server.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")
# Use this if Generic OnOff Server is used as root only model on an independent element
set(GENERIC_ONOFF_SERVER_WITH_MC_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_server.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_mc.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")
set(GENERIC_ONOFF_SERVER_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MODEL_COMMON_INCLUDE_DIRS}" CACHE INTERNAL "")

set(target_include_dirs
    ${GENERIC_ONOFF_CLIENT_INCLUDE_DIRS}
    ${GENERIC_ONOFF_SERVER_INCLUDE_DIRS}
    ${ACCESS_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/examples/templates)

set(target_include_dirs_with_scene
    "${target_include_dirs}"
    ${SCENE_SETUP_SERVER_INCLUDE_DIRS})

add_pc_lint(generic_onoff_${PLATFORM}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_client.c;${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_server.c;${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_mc.c"
    "${target_include_dirs};${CMAKE_SOURCE_DIR}/mesh/test/include"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DGENERIC_ONOFF_SERVER_INSTANCES_MAX=1;-DSCENE_SETUP_SERVER_INSTANCES_MAX=0")
add_pc_lint(generic_onoff_with_scene_${PLATFORM}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_client.c;${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_server.c;${CMAKE_CURRENT_SOURCE_DIR}/src/generic_onoff_mc.c"
    "${target_include_dirs_with_scene};${CMAKE_SOURCE_DIR}/mesh/test/include"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DGENERIC_ONOFF_SERVER_INSTANCES_MAX=1;-DSCENE_SETUP_SERVER_INSTANCES_MAX=1")
