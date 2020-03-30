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

/* Module configuration */
#include "nrf_mesh_config_examples.h"
#include "sdk_config.h"

#if BLE_DFU_SUPPORT_ENABLED

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* nRF5 SDK */
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_power.h"

/* BLE */
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"

/* Mesh */
#include "mesh_stack.h"
#include "proxy.h"
#include "mesh_adv.h"
#include "mesh_provisionee.h"
#include "flash_manager.h"
#include "nrf_mesh_events.h"

/** Configuration required to run the support module */
STATIC_ASSERT(NRF_PWR_MGMT_CONFIG_AUTO_SHUTDOWN_RETRY == 1, "Blocked shutdown procedure shall be enabled.");
STATIC_ASSERT(NRF_SDH_BLE_VS_UUID_COUNT > 0, "At least vendor-specific UUID shall be allowed.");
STATIC_ASSERT(NRF_SDH_BLE_SERVICE_CHANGED == 1, "The Service Changed characteristic shall be enabled.");

static bool m_waiting_mesh_config;

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt);
static nrf_mesh_evt_handler_t m_mesh_evt_handler = {
    .evt_cb = mesh_evt_handler,
};

static void mesh_evt_handler(const nrf_mesh_evt_t * p_evt)
{
    if (NRF_MESH_EVT_CONFIG_STABLE == p_evt->type && m_waiting_mesh_config)
    {
        m_waiting_mesh_config = false;
    }
}

static bool dfu_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    if (NRF_PWR_MGMT_EVT_PREPARE_DFU == event)
    {
        /* Don't let to reboot device until the flash is stable. */
        return !m_waiting_mesh_config;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(dfu_shutdown_handler, 0);

static void disconnect_cb(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    (void) sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
}

static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
            /* Prevent device from advertising on disconnect. */
    #if MESH_FEATURE_GATT_PROXY_ENABLED
            if (mesh_stack_is_device_provisioned())
            {
                (void) proxy_stop();
            }
            else
            {
                (void) mesh_provisionee_prov_listen_stop();
            }
    #endif

            /* Disconnect all other devices that currently are connected. */
            (void) ble_conn_state_for_each_connected(disconnect_cb, NULL);

            /* Let the mesh stack store its configuration before going to DFU mode. */
            m_waiting_mesh_config = true;
            nrf_mesh_evt_handler_add(&m_mesh_evt_handler);
            mesh_config_power_down();
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            m_waiting_mesh_config = false;
            NRF_MESH_ASSERT(false);
            break;

        default:
            break;
    }
}

void ble_dfu_support_init(void)
{
    ret_code_t err_code = ble_dfu_buttonless_async_svci_init();
    if (err_code != NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "Unable to initialize Buttonless DFU Service. Is the bootloader flashed?\n");
    }
    APP_ERROR_CHECK(err_code);
}

void ble_dfu_support_service_init(void)
{
    uint32_t                  err_code;
    ble_dfu_buttonless_init_t dfu_init = {0};

    dfu_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfu_init);
    APP_ERROR_CHECK(err_code);
}

#endif /* BLE_DFU_SUPPORT_ENABLED */
