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

#include <stdio.h>

/* HAL */
#include "boards.h"
#include "nrf_delay.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "flash_manager.h"
#include "mesh_stack.h"
#include "access_config.h"
#include "proxy.h"
#include "mesh_opt.h"

/* Provisioning and configuration */
#include "nrf_mesh_configure.h"
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_client.h"

/* Logging and RTT */
#include "rtt_input.h"
#include "log.h"

/* Example specific includes */
#include "app_config.h"
#include "enocean_switch_example.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh_configure.h"
#include "enocean.h"
#include "example_common.h"
#include "ble_softdevice_support.h"
#include "ad_type_filter.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
/* Configure switch debounce timeout for EnOcean switch here */
#define SWITCH_DEBOUNCE_INTERVAL_US (MS_TO_US(500))
/* Two EnOcean switches can be used in parallel */
#define MAX_ENOCEAN_DEVICES_SUPPORTED (2)

#define APP_STATE_OFF                 (0)
#define APP_STATE_ON                  (1)

#define APP_UNACK_MSG_REPEAT_COUNT    (2)

#define LIGHT_SWITCH_CLIENTS          (PTM215B_NUMBER_OF_SWITCHES/2)

/* Range for the enocean record entry IDs */
#define ENOCEAN_SWITCH_RECORD_START (0x0001)
#define ENOCEAN_SWITCH_RECORD_END (ENOCEAN_SWITCH_RECORD_START + MAX_ENOCEAN_DEVICES_SUPPORTED - 1)
/* File IDs for the enocean example that store its parameters in the persistence memory.
 * The IDs must be unique. */
#define MESH_APP_FILE_ID (0x0011)
/* Enocean switch mesh config data entry. */
#define ENOCEAN_SWITCH_ENTRY_ID MESH_CONFIG_ENTRY_ID(MESH_APP_FILE_ID, ENOCEAN_SWITCH_RECORD_START)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION       (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                 (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Delay value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_DELAY_MS           (50)
/* Transition time value used by the OnOff client for sending OnOff Set messages. */
#define APP_ONOFF_TRANSITION_TIME_MS (100)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static uint32_t enocean_switch_setter(mesh_config_entry_id_t id, const void * p_entry);
static void enocean_switch_getter(mesh_config_entry_id_t id, void * p_entry);
static void enocean_switch_deleter(mesh_config_entry_id_t id);
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in);
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);
static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);
static void app_start(void);


/*****************************************************************************
 * Static variables
 *****************************************************************************/
typedef struct
{
    uint32_t a0_ts;
    uint32_t a1_ts;
    uint32_t b0_ts;
    uint32_t b1_ts;
} app_switch_debounce_state_t;

/* Single advertiser instance. May periodically transmit one packet at a time. */
static generic_onoff_client_t m_clients[LIGHT_SWITCH_CLIENTS];
static bool m_device_provisioned;

static app_secmat_flash_t m_app_secmat_flash[MAX_ENOCEAN_DEVICES_SUPPORTED];
static enocean_commissioning_secmat_t m_app_secmat[MAX_ENOCEAN_DEVICES_SUPPORTED];
static uint8_t  m_enocean_dev_cnt;
static app_switch_debounce_state_t m_switch_state[MAX_ENOCEAN_DEVICES_SUPPORTED];

static nrf_mesh_evt_handler_t m_mesh_core_event_handler = { .evt_cb = app_mesh_core_event_cb };

const generic_onoff_client_callbacks_t client_cbs =
{
    .onoff_status_cb = app_generic_onoff_client_status_cb,
    .ack_transaction_status_cb = app_gen_onoff_client_transaction_status_cb,
    .periodic_publish_cb = app_gen_onoff_client_publish_interval_cb
};

/* Declare a mesh config file with a unique file ID */
NRF_MESH_STATIC_ASSERT(MESH_OPT_FIRST_FREE_ID <= MESH_APP_FILE_ID);
MESH_CONFIG_FILE(m_enocean_switch_file, MESH_APP_FILE_ID, MESH_CONFIG_STRATEGY_CONTINUOUS);

MESH_CONFIG_ENTRY(light_switch_provisioner,
                  ENOCEAN_SWITCH_ENTRY_ID,
                  MAX_ENOCEAN_DEVICES_SUPPORTED,
                  sizeof(app_secmat_flash_t),
                  enocean_switch_setter,
                  enocean_switch_getter,
                  enocean_switch_deleter,
                  false);

static uint32_t enocean_switch_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    if (!IS_IN_RANGE(id.record, ENOCEAN_SWITCH_RECORD_START, ENOCEAN_SWITCH_RECORD_END))
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t idx = id.record - ENOCEAN_SWITCH_RECORD_START;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Enocean switch [%d] setter ...\n", idx);

    const app_secmat_flash_t * p_app_secmat_flash = (const app_secmat_flash_t *) p_entry;

    m_app_secmat_flash[idx].seq = p_app_secmat_flash->seq;
    memcpy(m_app_secmat_flash[idx].ble_gap_addr, p_app_secmat_flash->ble_gap_addr, BLE_GAP_ADDR_LEN);
    memcpy(m_app_secmat_flash[idx].key, p_app_secmat_flash->key, PTM215B_COMM_PACKET_KEY_SIZE);

    return NRF_SUCCESS;
}

static void enocean_switch_getter(mesh_config_entry_id_t id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, ENOCEAN_SWITCH_RECORD_START, ENOCEAN_SWITCH_RECORD_END));

    uint16_t idx = id.record - ENOCEAN_SWITCH_RECORD_START;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Enocean switch [%d] getter ...\n", idx);

    app_secmat_flash_t * p_app_secmat_flash = (app_secmat_flash_t *) p_entry;

    p_app_secmat_flash->seq = m_app_secmat_flash[idx].seq;
    memcpy(p_app_secmat_flash->ble_gap_addr, m_app_secmat_flash[idx].ble_gap_addr, BLE_GAP_ADDR_LEN);
    memcpy(p_app_secmat_flash->key, m_app_secmat_flash[idx].key, PTM215B_COMM_PACKET_KEY_SIZE);
}

static void enocean_switch_deleter(mesh_config_entry_id_t id)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(id.record, ENOCEAN_SWITCH_RECORD_START, ENOCEAN_SWITCH_RECORD_END));

    uint16_t idx = id.record - ENOCEAN_SWITCH_RECORD_START;
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Enocean switch [%d] deleter ...\n", idx);

    memset(&m_app_secmat_flash[idx], 0x00, sizeof(app_secmat_flash_t));
}

static void enocean_switch_invalidate(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Enocean switch data invalidated ...\n");

    while (m_enocean_dev_cnt > 0)
    {
        mesh_config_entry_id_t entry_id = ENOCEAN_SWITCH_ENTRY_ID;
        entry_id.record += m_enocean_dev_cnt - 1;
        NRF_MESH_ERROR_CHECK(mesh_config_entry_delete(entry_id));
        m_enocean_dev_cnt--;
    }
}

static uint8_t enocean_device_index_get(enocean_evt_t * p_evt)
{
    uint8_t i;
    for (i = 0; i < m_enocean_dev_cnt; i++)
    {
        if (memcmp(p_evt->p_ble_gap_addr, m_app_secmat[i].p_ble_gap_addr, BLE_GAP_ADDR_LEN) == 0)
        {
            break;
        }
    }

    /* This should never assert. */
    if (i == m_enocean_dev_cnt)
    {
        APP_ERROR_CHECK(false);
    }

    return i;
}

/* Forward the ADV packets to the Enocean module */
static void rx_callback(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    enocean_packet_process(p_rx_data);
}

static void app_switch_debounce(enocean_switch_status_t * p_status, uint8_t index)
{
    uint32_t timestamp = timer_now();
    uint32_t status = NRF_ERROR_INTERNAL;
    generic_onoff_set_params_t set_params;
    model_transition_t transition_params;
    static uint8_t tid = 0;

    set_params.tid = tid++;
    transition_params.delay_ms = APP_ONOFF_DELAY_MS;
    transition_params.transition_time_ms = APP_ONOFF_TRANSITION_TIME_MS;

    if (p_status->action == PRESS_ACTION)
    {
        /* Change state on the unicast server address using 1st on/off client */
        if (p_status->a0 && timer_diff(timestamp, m_switch_state[index].a0_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
        {
            m_switch_state[index].a0_ts = timestamp;
            set_params.on_off = APP_STATE_ON;
            hal_led_pin_set(BSP_LED_0, set_params.on_off);
            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
        }
        else if (p_status->a1 && timer_diff(timestamp, m_switch_state[index].a1_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
        {
            m_switch_state[index].a1_ts = timestamp;
            set_params.on_off = APP_STATE_OFF;
            hal_led_pin_set(BSP_LED_0, set_params.on_off);
            status = generic_onoff_client_set(&m_clients[0], &set_params, &transition_params);
        }

        if (status == NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: Client[0]: ONOFF SET %d\n", set_params.on_off);
        }
        status = NRF_ERROR_INTERNAL;

        /* Change state on the nodes subscribed to the Odd group address using 2nd on/off client */
        if (p_status->b0 && timer_diff(timestamp, m_switch_state[index].b0_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
        {
            m_switch_state[index].b0_ts = timestamp;
            set_params.on_off = APP_STATE_ON;
            hal_led_pin_set(BSP_LED_1, set_params.on_off);
            status = generic_onoff_client_set_unack(&m_clients[1], &set_params, &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
        }
        else if (p_status->b1 && timer_diff(timestamp, m_switch_state[index].b1_ts) > SWITCH_DEBOUNCE_INTERVAL_US)
        {
            m_switch_state[index].b1_ts = timestamp;
            set_params.on_off = APP_STATE_OFF;
            hal_led_pin_set(BSP_LED_1, set_params.on_off);
            status = generic_onoff_client_set_unack(&m_clients[1], &set_params, &transition_params, APP_UNACK_MSG_REPEAT_COUNT);
        }

        if (status == NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: Client[1]: ONOFF SET %d\n", set_params.on_off);
        }
    }
}

/* This example translates the messages from the PTM215B switches to on/off client model messages.
The mapping of the switches and corresponding client model messages is as follows:

Pressing Switch 1 will turn ON LED 1 on the servers with ODD addresses.
Pressing Switch 2 will turn OFF LED 1 on the servers with ODD addresses.
Pressing Switch 3 will turn ON LED 1 on the servers with EVEN addresses.
Pressing Switch 4 will turn OFF LED 1 on the servers with EVEN addresses.
*/
static void app_enocean_cb(enocean_evt_t * p_evt)
{
    if  (p_evt->type == ENOCEAN_EVT_DATA_RECEIVED)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sw data A0: %d A1: %d B0: %d B1: %d Action: %d\n",
              p_evt->params.data.status.a0,p_evt->params.data.status.a1,p_evt->params.data.status.b0,
              p_evt->params.data.status.b1,p_evt->params.data.status.action);

        app_switch_debounce(&p_evt->params.data.status, enocean_device_index_get(p_evt));
    }
    else if (p_evt->type == ENOCEAN_EVT_SECURITY_MATERIAL_RECEIVED)
    {
        if (m_enocean_dev_cnt < MAX_ENOCEAN_DEVICES_SUPPORTED)
        {
            app_secmat_flash_t app_secmat_flash;

            app_secmat_flash.seq = p_evt->params.secmat.seq;
            memcpy(app_secmat_flash.ble_gap_addr, p_evt->p_ble_gap_addr, BLE_GAP_ADDR_LEN);
            memcpy(app_secmat_flash.key, p_evt->params.secmat.p_key, PTM215B_COMM_PACKET_KEY_SIZE);

            mesh_config_entry_id_t entry_id = ENOCEAN_SWITCH_ENTRY_ID;
            entry_id.record += m_enocean_dev_cnt;

            uint32_t status = mesh_config_entry_set(entry_id, &app_secmat_flash);
            if (status == NRF_SUCCESS)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Storing: Enocean security material\n");

                m_app_secmat[m_enocean_dev_cnt].p_seq =  &m_app_secmat_flash[m_enocean_dev_cnt].seq;
                m_app_secmat[m_enocean_dev_cnt].p_ble_gap_addr = &m_app_secmat_flash[m_enocean_dev_cnt].ble_gap_addr[0];
                m_app_secmat[m_enocean_dev_cnt].p_key = &m_app_secmat_flash[m_enocean_dev_cnt].key[0];
                enocean_secmat_add(&m_app_secmat[m_enocean_dev_cnt]);

                m_enocean_dev_cnt++;
            }

            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "Cannot add new device. Max number of supported devices: %d\n", MAX_ENOCEAN_DEVICES_SUPPORTED);
        }
    }
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_gen_onoff_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_gen_onoff_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status)
{
    switch(status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer success.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer timeout.\n");
            break;

        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Acknowledged transfer cancelled.\n");
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }
}

/* Generic OnOff client model interface: Process the received status message in this callback */
static void app_generic_onoff_client_status_cb(const generic_onoff_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const generic_onoff_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d, Target OnOff: %d, Remaining Time: %d ms\n",
              p_meta->src.value, p_in->present_on_off, p_in->target_on_off, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "OnOff server: 0x%04x, Present OnOff: %d\n",
              p_meta->src.value, p_in->present_on_off);
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        /* Trigger clearing of application data and schedule node reset. */
        enocean_switch_invalidate();
        node_reset();
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t------------------------------------------------------\n"
    "\t\t Button/RTT 4) Clear all the states to reset the node.\n"
    "\t\t------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        /* Initiate node reset */
        case 4:
        {
            if (!mesh_stack_is_device_provisioned())
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
                return;
            }

            /* Clear all the states to reset the node. */
#if MESH_FEATURE_GATT_PROXY_ENABLED
            (void) proxy_stop();
#endif
            enocean_switch_invalidate();
            mesh_stack_config_clear();
            node_reset();
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }
}

static void app_mesh_core_event_cb(const nrf_mesh_evt_t * p_evt)
{
    /* USER_NOTE: User can insert mesh core event processing here */
    switch (p_evt->type)
    {
        /* Start user application specific functions only when stack is enabled */
        case NRF_MESH_EVT_ENABLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Mesh evt: NRF_MESH_EVT_ENABLED \n");
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
        m_clients[i].settings.p_callbacks = &client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;

        ERROR_CHECK(generic_onoff_client_init(&m_clients[i], i + 1));
    }
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

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning
     * service to the Proxy  */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

static void app_rtt_input_handler(int key)
{
    if (key >= '1' && key <= '4')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void mesh_init(void)
{
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
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

    /* Register event handler to receive NRF_MESH_EVT_FLASH_STABLE. Application functionality will
    be started after this event */
    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS | LOG_SRC_BEARER, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Enocean Translator Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();

#if MESH_FEATURE_GATT_ENABLED
    gap_params_init();
    conn_params_init();
#endif

    mesh_init();

    enocean_translator_init(app_enocean_cb);
}

static void app_start(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Starting application \n");

    /* Load app specific data */
    m_enocean_dev_cnt = 0;
    mesh_config_entry_id_t idx = ENOCEAN_SWITCH_ENTRY_ID;

    for (uint8_t i = 0; i < MAX_ENOCEAN_DEVICES_SUPPORTED; i++)
    {
        app_secmat_flash_t app_secmat_flash;

        uint32_t status = mesh_config_entry_get(idx, &app_secmat_flash);
        if (status == NRF_SUCCESS)
        {
            m_app_secmat[i].p_ble_gap_addr = &m_app_secmat_flash[i].ble_gap_addr[0];
            m_app_secmat[i].p_key = &m_app_secmat_flash[i].key[0];
            m_app_secmat[i].p_seq = &m_app_secmat_flash[i].seq;
            m_enocean_dev_cnt++;
            enocean_secmat_add(&m_app_secmat[i]);
        }
        idx.record++;
    }

    if (m_enocean_dev_cnt > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Restored: Enocean security materials\n");
    }

    /* Enable reception of EnOcean specific AD types from the scanner and install rx callback to
     * intercept incoming ADV packets so that they can be passed to the EnOcean packet processor. */
    bearer_adtype_add(BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA);
    nrf_mesh_rx_cb_set(rx_callback);
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_sd_ble_opt_set_cb = NULL,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_ENOCEAN
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);

#if !PERSISTENT_STORAGE
    app_start();
#endif

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
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
