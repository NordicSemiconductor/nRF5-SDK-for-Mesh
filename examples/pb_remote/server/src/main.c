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

#include <stdint.h>
#include <string.h>

/* HAL */
#include "nrf.h"
#include "boards.h"
#include "simple_hal.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_prov_bearer_adv.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "pb_remote_server.h"

#include "mesh_app_utils.h"
#include "mesh_stack.h"
#include "mesh_provisionee.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_configure.h"
#include "example_common.h"
#include "ble_softdevice_support.h"

#include "app_timer.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define REMOTE_SERVER_ELEMENT_INDEX (0)
#define PROVISIONER_ADDRESS         (0x0001)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/


/*****************************************************************************
 * Static variables
 *****************************************************************************/
typedef enum
{
    DEVICE_STATE_UNPROVISIONED,
    DEVICE_STATE_PROVISIONED
} device_state_t;

static pb_remote_server_t         m_remote_server;
static nrf_mesh_prov_bearer_adv_t m_prov_bearer_adv;
static dsm_handle_t               m_netkey_handle;
static dsm_handle_t               m_appkey_handle;
static dsm_handle_t               m_provisioner_address_handle;
static bool                       m_device_provisioned;


static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    ERROR_CHECK(pb_remote_server_init(&m_remote_server, REMOTE_SERVER_ELEMENT_INDEX));
}

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(HAL_LED_MASK, false);
    hal_led_blink_ms(HAL_LED_MASK_HALF,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void unicast_address_print(void)
{
    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    unicast_address_print();
    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
    /* TODO: This should be handled by the configuration server model. */
    mesh_key_index_t net_key_index;
    uint32_t count = 1;
    ERROR_CHECK(dsm_subnet_get_all(&net_key_index, &count));
    m_netkey_handle = dsm_net_key_index_to_subnet_handle(net_key_index);

    uint8_t appkey[NRF_MESH_KEY_SIZE] = {0};
    ERROR_CHECK(dsm_appkey_add(0, m_netkey_handle, appkey, &m_appkey_handle));
    ERROR_CHECK(dsm_address_publish_add(PROVISIONER_ADDRESS, &m_provisioner_address_handle));
    ERROR_CHECK(access_model_application_bind(m_remote_server.model_handle, m_appkey_handle));
    ERROR_CHECK(access_model_publish_address_set(m_remote_server.model_handle, m_provisioner_address_handle));
    ERROR_CHECK(access_model_publish_application_set(m_remote_server.model_handle, m_appkey_handle));
    ERROR_CHECK(access_model_publish_ttl_set(m_remote_server.model_handle, 6));
    ERROR_CHECK(pb_remote_server_enable(&m_remote_server));
    ERROR_CHECK(pb_remote_server_prov_bearer_set(&m_remote_server, nrf_mesh_prov_bearer_adv_interface_get(&m_prov_bearer_adv)));
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority     = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc         = DEV_BOARD_LF_CLK_CFG,
        .models.models_init_cb = models_init_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reboot device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Provisionee + Remote Provisioning Server Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    ble_stack_init();

    mesh_init();
}

static void start(void)
{
    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA_PB_REMOTE;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_PBR_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
        ERROR_CHECK(pb_remote_server_enable(&m_remote_server));
        ERROR_CHECK(pb_remote_server_prov_bearer_set(&m_remote_server, nrf_mesh_prov_bearer_adv_interface_get(&m_prov_bearer_adv)));
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
}

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
