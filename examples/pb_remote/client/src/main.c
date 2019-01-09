/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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
#include "nrf_sdm.h"
#include "boards.h"
#include "SEGGER_RTT.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_prov_bearer_adv.h"
#include "log.h"

#include "access.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "pb_remote_client.h"
#include "pb_remote_server.h"
#include "rtt_input.h"
#include "mesh_app_utils.h"
#include "mesh_stack.h"
#include "ble_softdevice_support.h"
#include "nrf_mesh_config_examples.h"
#include "example_common.h"
#include "nrf_mesh_configure.h"

/**
 * Static authentication data. This data must match the data provided to the provisioner node.
 */
#define STATIC_AUTH_DATA { 0xc7, 0xf7, 0x9b, 0xec, 0x9c, 0xf9, 0x74, 0xdd, 0xb9, 0x62, 0xbd, 0x9f, 0xd1, 0x72, 0xdd, 0x73 }

#define NET_KEY                  {0x50, 0x42, 0x5f, 0x52, 0x45, 0x4d, 0x4f, 0x54, 0x45, 0x5f, 0x4e, 0x45, 0x54, 0x4b, 0x45, 0x59}
#define IV_INDEX                 (0)
#define UNPROV_START_ADDRESS     (0x1337)
#define PROVISIONER_ADDRESS      (0x0001)
#define RTT_INPUT_POLL_PERIOD_MS (100)
#define ATTENTION_DURATION_S     (5)

typedef enum
{
    DEVICE_STATE_NONE,
    DEVICE_STATE_PB_ADV_MODE,
    DEVICE_STATE_PB_REMOTE_MODE
} device_state_t;

const char USAGE_STRING[] =
    "\n--------------------------------\n"
    "1) Provision first available device with PB-ADV\n"
    "2) Set current client publish handle (corresponding to a known server)\n"
    "\t 2.1) <address handle>\n"
    "3) Start remote scanning\n"
    "4) Cancel the remote scanning\n"
    "5) Start remote provisioning\n"
    "\t 5.1) Device number\n";

/* Provisioning encryption key storage (this is not how you should store your keys). */
static const uint8_t              m_netkey[NRF_MESH_KEY_SIZE] = NET_KEY;
static uint8_t                    m_public_key[NRF_MESH_PROV_PUBKEY_SIZE];
static uint8_t                    m_private_key[NRF_MESH_PROV_PRIVKEY_SIZE];
static nrf_mesh_prov_ctx_t        m_prov_ctx;
static nrf_mesh_prov_bearer_adv_t m_prov_bearer_adv;
static uint16_t                   m_next_unprov_address = UNPROV_START_ADDRESS;
static uint16_t                   m_num_elements_of_last_guy = 0;
static pb_remote_client_t         m_remote_client;
static device_state_t             m_device_state;
static uint8_t                    m_uuid_list[PB_REMOTE_SERVER_UUID_LIST_SIZE][NRF_MESH_UUID_SIZE];
static dsm_handle_t               m_devkey_handles[DSM_DEVICE_MAX];
static dsm_handle_t               m_netkey_handles[DSM_SUBNET_MAX];
static dsm_handle_t               m_appkey_handles[DSM_APP_MAX];
static dsm_handle_t               m_device_address_handles[DSM_NONVIRTUAL_ADDR_MAX];
static uint8_t                    m_next_unprov_index = 0;

static void prov_evt_handler(const nrf_mesh_prov_evt_t * p_evt);
static void remote_client_event_cb(const pb_remote_event_t * p_evt);


static void provisioner_start(void)
{
    nrf_mesh_prov_oob_caps_t capabilities = NRF_MESH_PROV_OOB_CAPS_DEFAULT(ACCESS_ELEMENT_COUNT);

    ERROR_CHECK(nrf_mesh_prov_generate_keys(m_public_key, m_private_key));
    ERROR_CHECK(nrf_mesh_prov_init(&m_prov_ctx, m_public_key, m_private_key, &capabilities, prov_evt_handler));
    ERROR_CHECK(nrf_mesh_prov_bearer_add(&m_prov_ctx, nrf_mesh_prov_bearer_adv_interface_get(&m_prov_bearer_adv)));
    ERROR_CHECK(nrf_mesh_prov_bearer_add(&m_prov_ctx, pb_remote_client_bearer_interface_get(&m_remote_client)));
    ERROR_CHECK(nrf_mesh_prov_scan_start(prov_evt_handler));
}

static void start_provisioning(const uint8_t * p_uuid, nrf_mesh_prov_bearer_type_t bearer_type)
{
    nrf_mesh_prov_provisioning_data_t prov_data =
        {
            .netkey            = NET_KEY,
            .netkey_index      = 0,
            .iv_index          = IV_INDEX,
            .address           = m_next_unprov_address,
            .flags.iv_update   = false,
            .flags.key_refresh = false
        };

    ERROR_CHECK(nrf_mesh_prov_provision(&m_prov_ctx, p_uuid, ATTENTION_DURATION_S, &prov_data, bearer_type));
}

static void prov_evt_handler(const nrf_mesh_prov_evt_t * p_evt)
{
    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED:
            if (m_device_state == DEVICE_STATE_PB_ADV_MODE)
            {
                start_provisioning(p_evt->params.unprov.device_uuid, NRF_MESH_PROV_BEARER_ADV);
                m_device_state = DEVICE_STATE_NONE;
            }
            break;

        case NRF_MESH_PROV_EVT_LINK_ESTABLISHED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link established\n");
            break;

        case NRF_MESH_PROV_EVT_LINK_CLOSED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Local provisioning link closed\n");
            break;

        case NRF_MESH_PROV_EVT_COMPLETE:
            ERROR_CHECK(dsm_address_publish_add(m_next_unprov_address,
                                                &m_device_address_handles[m_next_unprov_index]));
            ERROR_CHECK(dsm_devkey_add(p_evt->params.complete.p_prov_data->address,
                                       m_netkey_handles[0],
                                       p_evt->params.complete.p_devkey,
                                       &m_devkey_handles[1 + m_next_unprov_index]));
            __LOG(LOG_SRC_APP,
                  LOG_LEVEL_INFO,
                  "Provisioning complete! Added %04X as handle %u\n",
                  p_evt->params.complete.p_prov_data->address,
                  m_device_address_handles[m_next_unprov_index]);
            m_next_unprov_index++;
            m_next_unprov_address += m_num_elements_of_last_guy;

            hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
            break;

        case NRF_MESH_PROV_EVT_CAPS_RECEIVED:
        {
            uint32_t status = nrf_mesh_prov_oob_use(p_evt->params.oob_caps_received.p_context,
                                                    NRF_MESH_PROV_OOB_METHOD_STATIC,
                                                    0,
                                                    NRF_MESH_KEY_SIZE);
            if (status != NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Provisioning OOB selection rejected, error code %d\n",
                      status);
            }
            else
            {
                m_num_elements_of_last_guy = p_evt->params.oob_caps_received.oob_caps.num_elements;
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Using static authentication\n");
            }
            break;
        }

        case NRF_MESH_PROV_EVT_STATIC_REQUEST:
        {
            /* Request for static authentication data. This data is used to authenticate the two nodes. */
            uint8_t static_data[16] = STATIC_AUTH_DATA;
            ERROR_CHECK(nrf_mesh_prov_auth_data_provide(p_evt->params.static_request.p_context, static_data, 16));
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Static authentication data provided\n");
            break;
        }

        default:
            break;
    }
}

static void remote_client_event_cb(const pb_remote_event_t * p_evt)
{
    switch (p_evt->type)
    {
        case PB_REMOTE_EVENT_TX_FAILED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Communication with server failed\n");
            break;

        case PB_REMOTE_EVENT_LINK_CLOSED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The remote link has closed\n");
            break;

        case PB_REMOTE_EVENT_REMOTE_UUID:
            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Got remote uuid", p_evt->remote_uuid.p_uuid, NRF_MESH_UUID_SIZE);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device ID: %u\n", p_evt->remote_uuid.device_id);
            memcpy(&m_uuid_list[p_evt->remote_uuid.device_id][0], p_evt->remote_uuid.p_uuid, NRF_MESH_UUID_SIZE);
            break;

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Got event %u\n", p_evt->type);
            break;
    }
}

static void user_input_handler(int key)
{
    static enum { UIS_IDLE, UIS_PUBLISH_HANDLE, UIS_DEVICE_NUMBER } s_state = UIS_IDLE;
    uint32_t status;

    switch (s_state)
    {
        case UIS_IDLE:
            switch (key)
            {
                case '1':
                    m_device_state = DEVICE_STATE_PB_ADV_MODE;
                    break;

                case '2':
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Please enter a valid publish handle: \n");
                    s_state = UIS_PUBLISH_HANDLE;
                    break;

                case '3':
                    status = pb_remote_client_remote_scan_start(&m_remote_client);
                    if (status != NRF_SUCCESS)
                    {
                        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Error %u: Could not start remote scanning\n", status);
                    }
                    break;

                case '4':
                    status = pb_remote_client_remote_scan_cancel(&m_remote_client);
                    if (status != NRF_SUCCESS)
                    {
                        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Error %u: Could not cancel remote scanning\n", status);
                    }
                    break;

                case '5':
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Please enter a valid device number: \n");
                    s_state = UIS_DEVICE_NUMBER;
                    break;

                case '\n':
                case '\r':
                case -1:
                    break;

                default:
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, USAGE_STRING);
                    break;
            }
            break;

        case UIS_PUBLISH_HANDLE:
            if ((key >= '0') && (key <= '9'))
            {
                dsm_handle_t handle = (dsm_handle_t) key - '0';
                status = access_model_publish_address_set(m_remote_client.model_handle, handle);
                if (status != NRF_SUCCESS)
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Error %u: Could not set publish address\n", status);
                }
                else
                {
                    __LOG(LOG_SRC_APP,LOG_LEVEL_INFO, "Handle set \n");
                }
                s_state = UIS_IDLE;
            }
            break;

        case UIS_DEVICE_NUMBER:
            if ((key >= '0') && (key < ('0' + PB_REMOTE_SERVER_UUID_LIST_SIZE)))
            {
                start_provisioning(m_uuid_list[((uint8_t) key - '0')], NRF_MESH_PROV_BEARER_MESH);
                s_state = UIS_IDLE;
            }
            break;

        default:
            NRF_MESH_ASSERT(false);
    }
}

static void models_init_cb(void)
{
    ERROR_CHECK(pb_remote_client_init(&m_remote_client, 0, remote_client_event_cb));
    ERROR_CHECK(access_model_application_bind(m_remote_client.model_handle, m_appkey_handles[0]));
    ERROR_CHECK(access_model_publish_application_set(m_remote_client.model_handle, m_appkey_handles[0]));
    ERROR_CHECK(access_model_publish_ttl_set(m_remote_client.model_handle, 6));
}

static void mesh_init(void)
{
    bool device_provisioned;
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority     = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc         = DEV_BOARD_LF_CLK_CFG,
        .models.models_init_cb = models_init_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &device_provisioned));

    if (!device_provisioned)
    {
        uint8_t appkey[NRF_MESH_KEY_SIZE] = {0};
        dsm_local_unicast_address_t local_address = {PROVISIONER_ADDRESS, 1};
        ERROR_CHECK(dsm_local_unicast_addresses_set(&local_address));
        ERROR_CHECK(dsm_subnet_add(0, m_netkey, &m_netkey_handles[0]));
        ERROR_CHECK(dsm_appkey_add(0, m_netkey_handles[0], appkey, &m_appkey_handles[0]));
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Provisioner + Remote Provisioning Client Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

    ble_stack_init();

    mesh_init();
}

static void start(void)
{
    rtt_input_enable(user_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    provisioner_start();

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, USAGE_STRING);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
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
