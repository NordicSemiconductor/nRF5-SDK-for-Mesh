set(COMMON_EXAMPLES_SOURCE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_error_weak.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_onoff.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_light_lightness.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_light_lc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_level.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_light_ctl.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_scene.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_dtt.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_sensor.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_sensor_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_transition.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/assertion_handler_weak.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ble_dfu_support.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/ble_softdevice_support.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_adv.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_app_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mesh_provisionee.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/pwm_utils.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/rtt_input.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/simple_hal.c")

set(COMMON_EXAMPLES_INCLUDE_DIRS
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${SDK_ROOT}/models/model_spec/common/include"
    "${SDK_ROOT}/components/libraries/pwm"
    "${SDK_ROOT}/modules/nrfx"
    "${SDK_ROOT}/modules/nrfx/drivers/include"
    "${SDK_ROOT}/modules/nrfx/drivers"
    "${SDK_ROOT}/modules/nrfx/hal"
    "${SDK_ROOT}/modules/nrfx/mdk"
    "${SDK_ROOT}/integration/nrfx"
    "${SDK_ROOT}/integration/nrfx/legacy")

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

set(BLE_DFU_SUPPORT_SOURCE_FILES
    "${CMAKE_SOURCE_DIR}/examples/common/src/ble_dfu_support.c"
    "${SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu.c"
    "${SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu_bonded.c"
    "${SDK_ROOT}/components/ble/ble_services/ble_dfu/ble_dfu_unbonded.c"
    "${SDK_ROOT}/components/ble/ble_advertising/ble_advertising.c"
    "${SDK_ROOT}/components/ble/common/ble_conn_state.c"
    "${SDK_ROOT}/components/libraries/atomic_fifo/nrf_atfifo.c"
    "${SDK_ROOT}/components/libraries/atomic_flags/nrf_atflags.c"
    "${SDK_ROOT}/components/libraries/bootloader/dfu/nrf_dfu_svci.c"
    "${SDK_ROOT}/components/libraries/fds/fds.c"
    "${SDK_ROOT}/components/libraries/fstorage/nrf_fstorage.c"
    "${SDK_ROOT}/components/libraries/fstorage/nrf_fstorage_sd.c"
    "${SDK_ROOT}/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c"
    "${SDK_ROOT}/components/ble/peer_manager/gatt_cache_manager.c"
    "${SDK_ROOT}/components/ble/peer_manager/gatts_cache_manager.c"
    "${SDK_ROOT}/components/ble/peer_manager/id_manager.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_manager.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_data_storage.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_database.c"
    "${SDK_ROOT}/components/ble/peer_manager/peer_id.c"
    "${SDK_ROOT}/components/ble/peer_manager/pm_buffer.c"
    "${SDK_ROOT}/components/ble/peer_manager/security_dispatcher.c"
    "${SDK_ROOT}/components/ble/peer_manager/security_manager.c"
    CACHE INTERNAL "")

set(BLE_DFU_SUPPORT_INCLUDE_FILES
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${SDK_ROOT}/components/ble/ble_advertising"
    "${SDK_ROOT}/components/ble/ble_services/ble_dfu"
    "${SDK_ROOT}/components/ble/peer_manager"
    "${SDK_ROOT}/components/libraries/atomic_fifo"
    "${SDK_ROOT}/components/libraries/atomic_flags"
    "${SDK_ROOT}/components/libraries/bootloader"
    "${SDK_ROOT}/components/libraries/bootloader/ble_dfu"
    "${SDK_ROOT}/components/libraries/bootloader/dfu"
    "${SDK_ROOT}/components/libraries/fds"
    "${SDK_ROOT}/components/libraries/fstorage"
    "${SDK_ROOT}/components/libraries/mutex"
    "${SDK_ROOT}/components/libraries/pwr_mgmt"
    "${SDK_ROOT}/components/libraries/svc"
    CACHE INTERNAL "")

set(PWM_UTILS_SOURCE_FILES
    "${CMAKE_SOURCE_DIR}/examples/common/src/pwm_utils.c"
    "${SDK_ROOT}/components/libraries/pwm/app_pwm.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_timer.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_ppi.c"
    "${SDK_ROOT}/modules/nrfx/drivers/src/nrfx_gpiote.c"
    "${SDK_ROOT}/integration/nrfx/legacy/nrf_drv_ppi.c"
    CACHE INTERNAL "")

set(PWM_UTILS_INCLUDE_DIRS
    "${SDK_ROOT}/modules/nrfx/drivers/include/"
    "${SDK_ROOT}/integration/nrfx/legacy/"
    "${SDK_ROOT}/components/libraries/pwm/"
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
    ${CMAKE_SOURCE_DIR}/models/model_spec/common/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/generic_dtt/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/generic_level/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/generic_onoff/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/generic_ponoff/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/light_lc/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/light_lightness/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/light_ctl/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/scene/include
    ${CMAKE_SOURCE_DIR}/models/model_spec/sensor/include
    ${CMAKE_SOURCE_DIR}/external/rtt/include
    ${CMAKE_SOURCE_DIR}/examples/templates
    ${SDK_ROOT}/components/libraries/pwm/
    ${SDK_ROOT}/components/libraries/util
    ${SDK_ROOT}/components/drivers_nrf/delay
    ${SDK_ROOT}/integration/nrfx/legacy
    ${SDK_ROOT}/modules/nrfx/drivers/include
    ${CMAKE_SOURCE_DIR}/mesh/test/include # sdk_config.h
    ${${BOARD}_INCLUDE_DIRS}
    ${${SOFTDEVICE}_INCLUDE_DIRS}
    ${${PLATFORM}_INCLUDE_DIRS}
    ${${nRF5_SDK_VERSION}_INCLUDE_DIRS})

set(additional_defines_for_lint
    -DLIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX=1
    -DLIGHT_LC_SETUP_SERVER_INSTANCES_MAX=1
    -DLIGHT_LIGHTNESS_DEFAULT_RANGE_MAX=0xFFFE
    -DLIGHT_CTL_SETUP_SERVER_INSTANCES_MAX=1
    -DGENERIC_LEVEL_SERVER_INSTANCES_MAX=1
    -DGENERIC_ONOFF_SERVER_INSTANCES_MAX=1
    -DGENERIC_DTT_SERVER_INSTANCES_MAX=1
    -DSCENE_SETUP_SERVER_INSTANCES_MAX=1)

add_pc_lint(examples_common_${PLATFORM}
    "${COMMON_EXAMPLES_SOURCE_FILES}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES};-D__WEAK=;${additional_defines_for_lint}")


set(COMMON_EXAMPLES_SOURCE_FILES_NO_SCENE_LINT
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_onoff.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_light_lightness.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_light_lc.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_level.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_light_ctl.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_scene.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/app_dtt.c")

set(additional_defines_for_no_scene_lint
    -DLIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX=1
    -DLIGHT_LC_SETUP_SERVER_INSTANCES_MAX=1
    -DLIGHT_LIGHTNESS_DEFAULT_RANGE_MAX=0xFFFE
    -DLIGHT_CTL_SETUP_SERVER_INSTANCES_MAX=1
    -DGENERIC_LEVEL_SERVER_INSTANCES_MAX=1
    -DGENERIC_ONOFF_SERVER_INSTANCES_MAX=1
    -DGENERIC_DTT_SERVER_INSTANCES_MAX=1
    -DSCENE_SETUP_SERVER_INSTANCES_MAX=0)
add_pc_lint(examples_common_no_scene_${PLATFORM}
    "${COMMON_EXAMPLES_SOURCE_FILES_NO_SCENE_LINT}"
    "${lint_include_dirs}"
    "${${PLATFORM}_DEFINES};${${SOFTDEVICE}_DEFINES};${${BOARD}_DEFINES};-D__WEAK=;${additional_defines_for_no_scene_lint}")
