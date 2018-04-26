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

#include <stdio.h>

/* HAL */
#include "boards.h"
#include "nrf_delay.h"
#include "simple_hal.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "flash_manager.h"
#include "mesh_stack.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "nrf_mesh_configure.h"
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"
#include "mesh_softdevice_init.h"

/* Models */
#include "simple_on_off_client.h"

/* Logging and RTT */
#include "rtt_input.h"
#include "log.h"

/* Example specific includes */
#include "enocean_switch_example.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_configure.h"
#include "enocean.h"
#include "flash_helper.h"

/* Configure switch debounce timeout for EnOcean switch here */
#define SWITCH_DEBOUNCE_INTERVAL_US (MS_TO_US(500))
/* This examples shows interaction with only one EnOcean device*/
#define MAX_ENOCEAN_DEVICES_SUPPORTED (1)

#define RTT_INPUT_POLL_PERIOD_MS (100)

#define LED_BLINK_INTERVAL_MS       (200)
#define LED_BLINK_SHORT_INTERVAL_MS (50)
#define LED_BLINK_CNT_START         (2)
#define LED_BLINK_CNT_RESET         (3)
#define LED_BLINK_CNT_PROV          (4)
#define LED_BLINK_CNT_NO_REPLY      (6)

#define LIGHT_SWITCH_CLIENTS  (PTM215B_NUMBER_OF_SWITCHES)

typedef struct
{
    bool     a0;
    uint32_t a0_ts;
    bool     a1;
    uint32_t a1_ts;
    bool     b0;
    uint32_t b0_ts;
    bool     b1;
    uint32_t b1_ts;
} app_switch_status_t;

/** Single advertiser instance. May periodically transmit one packet at a time. */
static simple_on_off_client_t m_clients[LIGHT_SWITCH_CLIENTS];
static bool m_device_provisioned;

static app_secmat_flash_t m_app_secmat_flash[MAX_ENOCEAN_DEVICES_SUPPORTED];
static enocean_commissioning_secmat_t m_app_secmat[MAX_ENOCEAN_DEVICES_SUPPORTED];
static uint8_t  m_enocean_dev_cnt;
static app_switch_status_t m_switch_state;

static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);
static void app_start(void);

static nrf_mesh_evt_handler_t m_mesh_core_event_handler = { .evt_cb = app_mesh_core_event_cb };

/* Try to store app data. If busy, ask user to manually initiate operation. */
static void app_data_store_try(void)
{
    uint32_t status = app_flash_data_store(APP_DATA_ENTRY_HANDLE, &m_app_secmat_flash[0], sizeof(m_app_secmat_flash));
    if (status == NRF_SUCCESS)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Storing: Enocean security material\n");
    }
    else if (status == NRF_ERROR_NOT_SUPPORTED)
    {
       __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Cannot store: Persistent storage not enabled\n");
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Flash busy. Cannot store. Press Button 2 to try again. \n");
    }
}

/* Forward the ADV packets to the Enocean module */
static void rx_callback(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    enocean_packet_process(p_rx_data);
}

static void app_switch_debounce(enocean_switch_status_t * p_status)
{
    /* Toggle state on the unicast server address using 1st on/off client */
    if (p_status->a0 && timer_diff(timer_now(), m_switch_state.a0_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.a0_ts = timer_now();
        m_switch_state.a0 = !m_switch_state.a0;

        if (simple_on_off_client_set(&m_clients[0], m_switch_state.a0) == NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Client message sent: new val: %d\n", m_switch_state.a0);
        }
    }
    /* Toggle state on the second unicast server address using 2nd on/off client */
    else if (p_status->a1 && timer_diff(timer_now(), m_switch_state.a1_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.a1_ts = timer_now();
        m_switch_state.a1 = !m_switch_state.a1;

        if (simple_on_off_client_set(&m_clients[1], m_switch_state.a1) == NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Client message sent: new val: %d\n", m_switch_state.a1);
        }
    }

    /* Toggle state on the nodes subscribed to the Odd group address using 3rd on/off client */
    if (p_status->b0 && timer_diff(timer_now(), m_switch_state.b0_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.b0_ts = timer_now();
        m_switch_state.b0 = !m_switch_state.b0;

        if (simple_on_off_client_set_unreliable(&m_clients[2], m_switch_state.b0, 2) == NRF_SUCCESS)
        {
            hal_led_pin_set(BSP_LED_2, m_switch_state.b0);
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Client message sent: new val: %d\n", m_switch_state.b0);
        }
    }
    /* Toggle state on the nodes subscribed to the Even group address using 3th on/off client */
    else if (p_status->b1 && timer_diff(timer_now(), m_switch_state.b1_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
    {
        m_switch_state.b1_ts = timer_now();
        m_switch_state.b1 = !m_switch_state.b1;

        if (simple_on_off_client_set_unreliable(&m_clients[3], m_switch_state.b1, 2) == NRF_SUCCESS)
        {
            hal_led_pin_set(BSP_LED_3, m_switch_state.b1);
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Client message sent: new val: %d\n", m_switch_state.b1);
        }
    }
}

/* This example translates the messages from the PTM215B switches to on/off client model messages.
The mapping of the switches and corresponding client model messages is as follows:

Pressing Switch 1, will toggle LED1 on the corresponding server.
Pressing Switch 2, will toggle LED1 on the corresponding server.
Pressing Switch 3, will toggle LED1 on the servers with Odd addresses.
Pressing Switch 4, will toggle LED1 on the servers with Even addresses.
*/
static void app_enocean_cb(enocean_evt_t * p_evt)
{
    if  (p_evt->type == ENOCEAN_EVT_DATA_RECEIVED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sw data A0: %d A1: %d B0: %d B1: %d Action: %d\n",
              p_evt->params.data.status.a0,p_evt->params.data.status.a1,p_evt->params.data.status.b0,
              p_evt->params.data.status.b1,p_evt->params.data.status.action);

        app_switch_debounce(&p_evt->params.data.status);
    }
    else if (p_evt->type == ENOCEAN_EVT_SECURITY_MATERIAL_RECEIVED)
    {
        if (m_enocean_dev_cnt < MAX_ENOCEAN_DEVICES_SUPPORTED)
        {
            m_app_secmat_flash[m_enocean_dev_cnt].seq = p_evt->params.secmat.seq;
            memcpy(&m_app_secmat_flash[m_enocean_dev_cnt].ble_gap_addr[0],
                   p_evt->p_ble_gap_addr, BLE_GAP_ADDR_LEN);
            memcpy(&m_app_secmat_flash[m_enocean_dev_cnt].key[0],
                   p_evt->params.secmat.p_key, PTM215B_COMM_PACKET_KEY_SIZE);

            app_data_store_try();

            m_app_secmat[m_enocean_dev_cnt].p_seq =  &m_app_secmat_flash[m_enocean_dev_cnt].seq;
            m_app_secmat[m_enocean_dev_cnt].p_ble_gap_addr = &m_app_secmat_flash[m_enocean_dev_cnt].ble_gap_addr[0];
            m_app_secmat[m_enocean_dev_cnt].p_key = &m_app_secmat_flash[m_enocean_dev_cnt].key[0];
            enocean_secmat_add(&m_app_secmat[m_enocean_dev_cnt]);

            m_enocean_dev_cnt++;

            hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot add new device. Max number of supported devices: %d\n", MAX_ENOCEAN_DEVICES_SUPPORTED);
        }
    }
}

static uint32_t server_index_get(const simple_on_off_client_t * p_client)
{
    uint32_t index = p_client - &m_clients[0];
    NRF_MESH_ASSERT(index < LIGHT_SWITCH_CLIENTS);
    return index;
}

static void client_status_cb(const simple_on_off_client_t * p_self, simple_on_off_status_t status, uint16_t src)
{
    uint32_t server_index = server_index_get(p_self);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Status received from server_idx: %d state: %d\n", server_index, status);

    switch (status)
    {
        case SIMPLE_ON_OFF_STATUS_ON:
            hal_led_pin_set(BSP_LED_0 + server_index, true);
            break;

        case SIMPLE_ON_OFF_STATUS_OFF:
            hal_led_pin_set(BSP_LED_0 + server_index, false);
            break;

        case SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY:
            hal_led_blink_ms(LEDS_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case SIMPLE_ON_OFF_STATUS_CANCELLED:
        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        node_reset();
    }
}

static void button_event_handler(uint32_t button_number)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        /* Press SW1 to store latest sequence number.
        USER_NOTE: End product should implement a mechanism to trigger this process upon
        external event, such as power fail interrupt, and ensure that the device finishes flash writes
        before running out of power. */
        case 1:
        {
            app_data_store_try();
            break;
        }

        /* Initiate node reset */
        case 3:
        {
            /* Clear all the states to reset the node. */
            mesh_stack_config_clear();
            app_flash_clear(&m_app_secmat_flash[0], sizeof(m_app_secmat_flash));
            node_reset();
            break;
        }

        default:
            break;
    }
}

static void app_mesh_core_event_cb(const nrf_mesh_evt_t * p_evt)
{
    /* USER_NOTE: User can insert mesh core event proceesing here */
    switch (p_evt->type)
    {
        /* Start user application specific functions only when flash is stable */
        case NRF_MESH_EVT_FLASH_STABLE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Mesh evt: FLASH_STABLE \n");
            {
                static bool s_app_started;
                if (!s_app_started)
                {
                    /* Flash operation initiated during initialization has been completed */
                    app_start();
                    s_app_started = true;
                }
            }
            break;

        default:
            break;
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    /* Model initialization */
    for (uint32_t i = 0; i < LIGHT_SWITCH_CLIENTS; ++i)
    {
        m_clients[i].status_cb = client_status_cb;
        ERROR_CHECK(simple_on_off_client_init(&m_clients[i], i + 1));
        ERROR_CHECK(access_model_subscription_list_alloc(m_clients[i].model_handle));
    }
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void app_rtt_input_handler(int key)
{
    if (key >= '0' && key <= '3')
    {
        uint32_t button_number = key - '0';
        button_event_handler(button_number);
    }
}

static void mesh_init(void)
{
    uint8_t dev_uuid[NRF_MESH_UUID_SIZE] = CLIENT_NODE_UUID;

    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = dev_uuid,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };
    ERROR_CHECK(mesh_stack_init(&init_params, &m_device_provisioned));

    /* Register event handler to receive NRF_MESH_EVT_FLASH_STABLE. Application functionality will
    be started after this event */
    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS , LOG_LEVEL_DBG3, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Enocean Switch Translator Demo -----\n");

    hal_leds_init();
#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif
    nrf_clock_lf_cfg_t lfc_cfg = DEV_BOARD_LF_CLK_CFG;
    ERROR_CHECK(mesh_softdevice_init(lfc_cfg));
    mesh_init();

    app_flash_init();
    enocean_translator_init(app_enocean_cb);
}

static void app_start(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Starting application \n");

    /* Load app specific data */
    if (app_flash_data_load(APP_DATA_ENTRY_HANDLE, &m_app_secmat_flash[0], sizeof(m_app_secmat_flash)) == NRF_SUCCESS)
    {
        for (uint8_t i = 0; i < MAX_ENOCEAN_DEVICES_SUPPORTED; i++)
        {
            m_app_secmat[i].p_ble_gap_addr = &m_app_secmat_flash[i].ble_gap_addr[0];
            m_app_secmat[i].p_key = &m_app_secmat_flash[i].key[0];
            m_enocean_dev_cnt++;
            enocean_secmat_add(&m_app_secmat[i]);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Restored: Enocean security materials\n");
        }
    }
    else
    {
        m_enocean_dev_cnt = 0;
    }

    /* Install rx callback to intercept incoming ADV packets so that they can be passed to the
    EnOcean packet processor */
    nrf_mesh_rx_cb_set(rx_callback);
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    ERROR_CHECK(mesh_stack_start());

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }

    const uint8_t *p_uuid = nrf_mesh_configure_device_uuid_get();
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Device UUID ", p_uuid, NRF_MESH_UUID_SIZE);

    hal_led_mask_set(LEDS_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(LEDS_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);

#if !PERSISTENT_STORAGE
    app_start();
#endif
}

int main(void)
{
    initialize();
    execution_start(start);

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
