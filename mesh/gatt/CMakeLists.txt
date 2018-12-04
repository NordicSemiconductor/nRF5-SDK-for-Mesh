# TODO: Because of the weak linkage, compiling GATT as a library isn't working.

set(MESH_GATT_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_gatt.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proxy.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/proxy_filter.c
    CACHE INTERNAL "")

set(MESH_GATT_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "")

set(lint_include_dirs
    ${MESH_GATT_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../dfu/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../test/include # sdk_config.h
    ${SDK_ROOT}/components/libraries/util
    ${SDK_ROOT}/components/libraries/experimental_section_vars
    ${SDK_ROOT}/components/softdevice/common
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

add_pc_lint(gatt_${PLATFORM}
    "${MESH_GATT_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};-DMESH_FEATURE_GATT_PROXY_ENABLED=1;-DMESH_FEATURE_PB_GATT_ENABLED=1")
