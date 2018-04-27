set(CONFIG_CLIENT_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config_client.c" CACHE INTERNAL "")

set(CONFIG_CLIENT_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "")

set(CONFIG_SERVER_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/composition_data.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/config_server.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/packed_index_list.c" CACHE INTERNAL "")

set(CONFIG_SERVER_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include" CACHE INTERNAL "")

get_property(core_include_dirs TARGET mesh_core_${PLATFORM} PROPERTY INCLUDE_DIRECTORIES)
set(target_include_dirs
    ${CONFIG_CLIENT_INCLUDE_DIRS}
    ${CONFIG_SERVER_INCLUDE_DIRS}
    ${ACCESS_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/mesh/app)

add_pc_lint(config_${PLATFORM}
    "${CONFIG_CLIENT_SOURCE_FILES};${CONFIG_SERVER_SOURCE_FILES}"
    "${target_include_dirs};${core_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES}")
