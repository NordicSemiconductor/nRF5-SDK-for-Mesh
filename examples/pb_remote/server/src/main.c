/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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
#include "nrf_mesh_sdk.h"
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

#include "nrf_mesh_node_config.h"
/*****************************************************************************
 * Definitions
 *****************************************************************************/

/**
 * Static authentication data. This data must match the data provided to the provisioner node.
 */
#define STATIC_AUTH_DATA { 0xc7, 0xf7, 0x9b, 0xec, 0x9c, 0xf9, 0x74, 0xdd, 0xb9, 0x62, 0xbd, 0x9f, 0xd1, 0x72, 0xdd, 0x73 }
#define REMOTE_SERVER_ELEMENT_INDEX (0)
#define PROVISIONER_ADDRESS (0x0001)
#define LED_PIN_NUMBER (BSP_LED_0)
#define LED_PIN_MASK   (1u << LED_PIN_NUMBER)
typedef enum
{
    DEVICE_STATE_UNPROVISIONED,
    DEVICE_STATE_PROVISIONED
} device_state_t;

/*****************************************************************************
 * Static data
 *****************************************************************************/

static pb_remote_server_t m_remote_server;
static nrf_mesh_prov_bearer_adv_t m_prov_bearer_adv;

static dsm_handle_t m_netkey_handle;
static dsm_handle_t m_appkey_handle;
static dsm_handle_t m_provisioner_address_handle;

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void configuration_setup(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    ERROR_CHECK(pb_remote_server_init(&m_remote_server, REMOTE_SERVER_ELEMENT_INDEX));
    hal_led_mask_set(LEDS_MASK, true);
}

static void provisioning_complete(void * p_unused)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");
    hal_led_mask_set(LEDS_MASK, false);
    hal_led_blink_ms(LED_PIN_MASK, 200, 4);
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

int main(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Provisionee + Remote Provisioning Server Demo -----\n");

    hal_leds_init();

    static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
    static nrf_mesh_node_config_params_t config_params =
        {.prov_caps = NRF_MESH_PROV_OOB_CAPS_DEFAULT(ACCESS_ELEMENT_COUNT)};
    config_params.p_static_data = static_auth_data;
    config_params.complete_callback = provisioning_complete;
    config_params.setup_callback = configuration_setup;
    config_params.irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST;

#if defined(S110)
    config_params.lf_clk_cfg = NRF_CLOCK_LFCLKSRC_XTAL_20_PPM;
#elif SD_BLE_API_VERSION >= 5
    config_params.lf_clk_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
    config_params.lf_clk_cfg.accuracy = NRF_CLOCK_LF_ACCURACY_20_PPM;
#else
    config_params.lf_clk_cfg.source = NRF_CLOCK_LF_SRC_XTAL;
    config_params.lf_clk_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM;
#endif

    ERROR_CHECK(nrf_mesh_node_config(&config_params));

    while (true)
    {
        (void)sd_app_evt_wait();
    }
}
