set(GENERIC_LEVEL_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_level_client.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_level_server.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_level_mc.c")

    set(GENERIC_LEVEL_CLIENT_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_level_client.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")
set(GENERIC_LEVEL_CLIENT_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MODEL_COMMON_INCLUDE_DIRS}" CACHE INTERNAL "")

# Use this if Generic Level Server is used as part of the extended model, where extended model
# stores the states
set(GENERIC_LEVEL_SERVER_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_level_server.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")
# Use this if Generic Level Server is used as root only model on an independent element
set(GENERIC_LEVEL_SERVER_WITH_MC_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_level_server.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_level_mc.c"
    "${MODEL_COMMON_SOURCE_FILES}" CACHE INTERNAL "")
set(GENERIC_LEVEL_SERVER_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MODEL_COMMON_INCLUDE_DIRS}" CACHE INTERNAL "")

set(target_include_dirs
    ${GENERIC_LEVEL_CLIENT_INCLUDE_DIRS}
    ${GENERIC_LEVEL_SERVER_INCLUDE_DIRS}
    ${SCENE_SETUP_SERVER_INCLUDE_DIRS}
    ${ACCESS_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/examples/templates)

add_pc_lint(generic_level_${PLATFORM}
    "${GENERIC_LEVEL_SOURCE_FILES}"
    "${target_include_dirs};${CMAKE_SOURCE_DIR}/mesh/test/include"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DGENERIC_LEVEL_SERVER_INSTANCES_MAX=1;-DSCENE_SETUP_SERVER_INSTANCES_MAX=0")
add_pc_lint(generic_level_with_scene_${PLATFORM}
    "${GENERIC_LEVEL_SOURCE_FILES}"
    "${target_include_dirs};${CMAKE_SOURCE_DIR}/mesh/test/include"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DGENERIC_LEVEL_SERVER_INSTANCES_MAX=1;-DSCENE_SETUP_SERVER_INSTANCES_MAX=1")
