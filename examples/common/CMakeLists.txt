set(COMMON_EXAMPLES_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_app_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_provisionee.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_softdevice_init.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/rtt_input.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/simple_hal.c")

set(COMMON_EXAMPLES_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE INTERNAL "")

set(lint_include_dirs
    ${COMMON_EXAMPLES_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/mesh/stack/api
    ${CMAKE_SOURCE_DIR}/mesh/core/include
    ${CMAKE_SOURCE_DIR}/mesh/core/api
    ${CMAKE_SOURCE_DIR}/mesh/access/api
    ${CMAKE_SOURCE_DIR}/mesh/access/include
    ${CMAKE_SOURCE_DIR}/mesh/prov/api
    ${CMAKE_SOURCE_DIR}/mesh/prov/include
    ${CMAKE_SOURCE_DIR}/mesh/bearer/api
    ${CMAKE_SOURCE_DIR}/mesh/bearer/include
    ${CMAKE_SOURCE_DIR}/mesh/dfu/api
    ${CMAKE_SOURCE_DIR}/mesh/dfu/include
    ${CMAKE_SOURCE_DIR}/models/config/include
    ${CMAKE_SOURCE_DIR}/models/health/include
    ${CMAKE_SOURCE_DIR}/external/rtt/include
    ${CMAKE_SOURCE_DIR}/examples/templates
    ${SDK_ROOT}/components/libraries/util
    ${SDK_ROOT}/components/drivers_nrf/delay
    ${CMAKE_SOURCE_DIR}/mesh/test/include # sdk_config.h
    ${${BOARD}_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

add_pc_lint(examples_common_${PLATFORM}
    "${COMMON_EXAMPLES_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES};-D__WEAK=")
