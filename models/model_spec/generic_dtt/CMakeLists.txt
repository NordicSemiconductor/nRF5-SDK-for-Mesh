set(GENERIC_DTT_CLIENT_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_dtt_client.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/src/model_common.c" CACHE INTERNAL "")
set(GENERIC_DTT_CLIENT_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/include" CACHE INTERNAL "")

set(GENERIC_DTT_SERVER_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_dtt_server.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/src/model_common.c" CACHE INTERNAL "")
set(GENERIC_DTT_SERVER_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../common/include" CACHE INTERNAL "")

set(target_include_dirs
    ${GENERIC_DTT_CLIENT_INCLUDE_DIRS}
    ${GENERIC_DTT_SERVER_INCLUDE_DIRS}
    ${ACCESS_INCLUDE_DIRS}
    ${MESH_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/examples/templates)

add_pc_lint(generic_dtt_${PLATFORM}
    "${CMAKE_CURRENT_SOURCE_DIR}/src/generic_dtt_client.c;${CMAKE_CURRENT_SOURCE_DIR}/src/generic_dtt_server.c;${CMAKE_CURRENT_SOURCE_DIR}/../common/src/model_common.c"
    "${target_include_dirs};${CMAKE_SOURCE_DIR}/mesh/test/include"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES}")
