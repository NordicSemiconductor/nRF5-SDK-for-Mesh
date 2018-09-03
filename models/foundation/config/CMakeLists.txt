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

set(target_include_dirs
    ${CONFIG_CLIENT_INCLUDE_DIRS}
    ${CONFIG_SERVER_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../health/include  # This is required because of mesh_stack.h
    ${ACCESS_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/examples/templates)

add_pc_lint(config_${PLATFORM}
    "${CONFIG_CLIENT_SOURCE_FILES};${CONFIG_SERVER_SOURCE_FILES}"
    "${target_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES}")
