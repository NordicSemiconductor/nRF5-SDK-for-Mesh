set(MESH_FRIEND_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/friend.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/friend_queue.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/friend_sublist.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/core_tx_friend.c
    CACHE INTERNAL "")

set(MESH_FRIEND_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MESH_BEARER_INCLUDE_DIRS}
    CACHE INTERNAL "")

set(lint_include_dirs
    ${MESH_FRIEND_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api # nrf_mesh_dfu_types.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../test/include # sdk_config.h
    ${SDK_ROOT}/components/libraries/util
    ${SDK_ROOT}/components/libraries/experimental_section_vars
    ${SDK_ROOT}/components/softdevice/common
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

add_pc_lint(friend_${PLATFORM}
    "${MESH_FRIEND_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DMESH_FEATURE_FRIEND_ENABLED=1;-DFRIEND_DEBUG=0;-DFRIEND_TEST_HOOK=0")
