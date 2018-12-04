set(COMMON_EXAMPLES_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_app_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_provisionee.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/rtt_input.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/simple_hal.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_adv.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/ble_softdevice_support.c")

set(COMMON_EXAMPLES_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include")

set(BLE_SOFTDEVICE_SUPPORT_SOURCE_FILES
    "${CMAKE_SOURCE_DIR}/examples/common/src/mesh_adv.c"
    "${CMAKE_SOURCE_DIR}/examples/common/src/ble_softdevice_support.c"
    "${SDK_ROOT}/components/libraries/util/app_util_platform.c"
    "${SDK_ROOT}/components/ble/common/ble_advdata.c"
    "${SDK_ROOT}/components/libraries/experimental_section_vars/nrf_section_iter.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_soc.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh_ble.c"
    "${SDK_ROOT}/components/softdevice/common/nrf_sdh.c"
    "${SDK_ROOT}/components/ble/common/ble_conn_params.c"
    "${SDK_ROOT}/components/libraries/atomic/nrf_atomic.c"
    "${SDK_ROOT}/components/ble/common/ble_srv_common.c"
    CACHE INTERNAL "")

set(BLE_SOFTDEVICE_SUPPORT_INCLUDE_DIRS
    "${SDK_ROOT}/components/ble/common"
    "${SDK_ROOT}/components/softdevice/common"
    "${SDK_ROOT}/components/libraries/strerror"
    "${SDK_ROOT}/components/libraries/atomic"
    CACHE INTERNAL "")

set(lint_include_dirs
    ${COMMON_EXAMPLES_INCLUDE_DIRS}
    ${BLE_SOFTDEVICE_SUPPORT_INCLUDE_DIRS}
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
    ${CMAKE_SOURCE_DIR}/mesh/gatt/api
    ${CMAKE_SOURCE_DIR}/mesh/gatt/include
    ${CMAKE_SOURCE_DIR}/models/foundation/config/include
    ${CMAKE_SOURCE_DIR}/models/foundation/health/include
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
