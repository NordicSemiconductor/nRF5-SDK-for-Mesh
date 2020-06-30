# TODO: Because of the weak linkage, compiling DFU as a library isn't working.

set(DFU_SOURCE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/nrf_mesh_dfu.c CACHE INTERNAL "")

set(DFU_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "")

set(lint_include_dirs
    ${DFU_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/api
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../prov/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../bearer/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../serial/api
    ${CMAKE_CURRENT_SOURCE_DIR}/../serial/include
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

add_pc_lint(dfu_${PLATFORM}
    "${DFU_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES}")
