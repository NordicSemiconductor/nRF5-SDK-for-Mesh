/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ble_softdevice_support.h"

/* Module configuration */
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_gatt.h"
#include "sdk_config.h"

/* nRF5 SDK */
#include "app_error.h"
#include "app_timer.h"

/* SoftDevice */
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

/* BLE */
#include "ble_conn_params.h"
#include "ble_hci.h"

/* Mesh */
#include "nrf_mesh.h"
#include "mesh_adv.h"
#include "log.h"

/** Configuration required to run the support module */
STATIC_ASSERT(NRF_SDH_ENABLED, "NRF_SDH_ENABLED not enabled.");
STATIC_ASSERT(NRF_SDH_BLE_ENABLED, "NRF_SDH_BLE_ENABLED not enabled.");
STATIC_ASSERT(NRF_SDH_SOC_ENABLED, "NRF_SDH_SOC_ENABLED not enabled.");

#if MESH_FEATURE_GATT_ENABLED
/** Configuration required to run Mesh GATT feature */
STATIC_ASSERT(NRF_BLE_CONN_PARAMS_ENABLED, "NRF_BLE_CONN_PARAMS_ENABLED not enabled.");
STATIC_ASSERT(NRF_SDH_BLE_SERVICE_CHANGED, "NRF_SDH_BLE_SERVICE_CHANGED not enabled.");
STATIC_ASSERT(NRF_SDH_BLE_PERIPHERAL_LINK_COUNT > 0, "Shall have at least one peripheral link.");
STATIC_ASSERT(NRF_SDH_BLE_GATT_MAX_MTU_SIZE >= 69, "Maximum GATT MTU size shall be not less than 69 bytes.");
#endif

#define GAP_CONN_PARAMS_INIT(_name) \
    do { \
        (_name).min_conn_interval = MIN_CONN_INTERVAL; \
        (_name).max_conn_interval = MAX_CONN_INTERVAL; \
        (_name).slave_latency     = SLAVE_LATENCY; \
        (_name).conn_sup_timeout  = CONN_SUP_TIMEOUT; \
    } while (0)

static void on_sd_evt(uint32_t sd_evt, void * p_context)
{
    UNUSED_VARIABLE(p_context);

    (void) nrf_mesh_on_sd_evt(sd_evt);
}

#if MESH_FEATURE_GATT_ENABLED
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                         BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
    else if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_SUCCEEDED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully updated connection parameters\n");
    }
}

static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
#endif

void ble_stack_init(void)
{
    uint32_t err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

#if MESH_FEATURE_GATT_ENABLED
    /* Set the default configuration (as defined through sdk_config.h). */
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(MESH_SOFTDEVICE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    /* Update GAP device name length. */
    ble_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cfg.gap_cfg.device_name_cfg.write_perm);
    cfg.gap_cfg.device_name_cfg.vloc        = BLE_GATTS_VLOC_STACK;
    cfg.gap_cfg.device_name_cfg.p_value     = NULL;
    cfg.gap_cfg.device_name_cfg.current_len = 0;
    cfg.gap_cfg.device_name_cfg.max_len     = strlen(GAP_DEVICE_NAME);
    APP_ERROR_CHECK(sd_ble_cfg_set(BLE_GAP_CFG_DEVICE_NAME, &cfg, ram_start));

    /* Enable BLE stack. */
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
#else /* if !MESH_FEATURE_GATT_ENABLED */
#if defined(_lint)
    uint32_t ram_start = 0ul;
#elif defined ( __CC_ARM )
    extern uint32_t Image$$RW_IRAM1$$Base;
    const volatile uint32_t ram_start = (uint32_t) &Image$$RW_IRAM1$$Base;
#elif defined   ( __GNUC__ )
    extern uint32_t __data_start__;
    volatile uint32_t ram_start = (uint32_t) &__data_start__;
#endif
    uint32_t app_ram_base = ram_start;

#if NRF_SD_BLE_API_VERSION == 1
    ble_enable_params_t ble_enable_params = {{0}};
    err_code = sd_ble_enable(&ble_enable_params);
#elif (NRF_SD_BLE_API_VERSION >= 2) && (NRF_SD_BLE_API_VERSION < 5)
    ble_enable_params_t ble_enable_params = {{0}};
    err_code = sd_ble_enable(&ble_enable_params, &app_ram_base);
#elif NRF_SD_BLE_API_VERSION == 5 || NRF_SD_BLE_API_VERSION == 6 || NRF_SD_BLE_API_VERSION == 7
    ble_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.common_cfg.vs_uuid_cfg.vs_uuid_count = 0;
    APP_ERROR_CHECK(sd_ble_cfg_set(BLE_COMMON_CFG_VS_UUID, &cfg, app_ram_base));

    memset(&cfg, 0, sizeof(cfg));
    cfg.gatts_cfg.attr_tab_size.attr_tab_size = BLE_GATTS_ATTR_TAB_SIZE_MIN;
    APP_ERROR_CHECK(sd_ble_cfg_set(BLE_GATTS_CFG_ATTR_TAB_SIZE, &cfg, app_ram_base));
    err_code = sd_ble_enable(&app_ram_base);
#else
    #error Unsupported NRF_SD_BLE_API_VERSION
#endif
    if (app_ram_base != ram_start)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_WARN,
              "sd_ble_enable: app_ram_base should be adjusted to 0x%0x\n", app_ram_base);
    }
    APP_ERROR_CHECK(err_code);
#endif /* !MESH_FEATURE_PB_GATT_ENABLED && !MESH_FEATURE_GATT_PROXY_ENABLED */

    /* Register Mesh handler for SoC events. */
    NRF_SDH_SOC_OBSERVER(mesh_observer, NRF_SDH_BLE_STACK_OBSERVER_PRIO, on_sd_evt, NULL);
}

#if MESH_FEATURE_GATT_ENABLED
void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    ble_gap_conn_params_t  gap_conn_params;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) GAP_DEVICE_NAME,
                                          strlen(GAP_DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    GAP_CONN_PARAMS_INIT(gap_conn_params);

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    ble_gap_conn_params_t  gap_conn_params;

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    GAP_CONN_PARAMS_INIT(gap_conn_params);

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = &gap_conn_params;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}
#endif /* MESH_FEATURE_GATT_ENABLED */
