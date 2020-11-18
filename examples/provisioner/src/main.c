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
#include "boards.h"
#include "nrf_delay.h"
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "access_config.h"
#include "device_state_manager.h"
#include "mesh_stack.h"
#include "net_state.h"
#include "mesh_opt_provisioner.h"
#include "mesh_config_entry.h"
#include "mesh_opt.h"

/* Provisioning and configuration */
#include "provisioner_helper.h"
#include "node_setup.h"
#include "mesh_app_utils.h"

/* Models */
#include "config_client.h"
#include "config_server.h"
#include "health_client.h"

/* Logging and RTT */
#include "rtt_input.h"
#include "log.h"

/* Example specific includes */
#include "example_network_config.h"
#include "nrf_mesh_config_examples.h"
#include "ble_softdevice_support.h"
#include "example_common.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define APP_PROVISIONING_LED            BSP_LED_0
#define APP_CONFIGURATION_LED           BSP_LED_1


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static uint32_t provisioner_setter(mesh_config_entry_id_t id, const void * p_entry);
static void provisioner_getter(mesh_config_entry_id_t id, void * p_entry);
static void provisioner_deleter(mesh_config_entry_id_t id);
static void app_health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event);
static void app_config_successful_cb(void);
static void app_config_failed_cb(void);
static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt);

static void app_start(void);


/*****************************************************************************
 * Static variables
 *****************************************************************************/
/* Required for the provisioner helper module */
static network_dsm_handles_data_volatile_t m_dev_handles;

static network_stats_data_stored_t m_nw_state;
static bool m_node_prov_setup_started;
static nrf_mesh_evt_handler_t m_mesh_core_event_handler = { .evt_cb = app_mesh_core_event_cb };

NRF_MESH_STATIC_ASSERT(MESH_OPT_FIRST_FREE_ID <= MESH_APP_FILE_ID);
MESH_CONFIG_FILE(m_provisioner_file, MESH_APP_FILE_ID, MESH_CONFIG_STRATEGY_CONTINUOUS);

MESH_CONFIG_ENTRY(provisioner,
                  PROVISIONER_ENTRY_ID,
                  1,
                  sizeof(network_stats_data_stored_t),
                  provisioner_setter,
                  provisioner_getter,
                  provisioner_deleter,
                  false);

static uint32_t provisioner_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Provisioner setter ...\n");
    NRF_MESH_ASSERT_DEBUG(PROVISIONER_RECORD == id.record);

    network_stats_data_stored_t * p_nsds = (network_stats_data_stored_t *) p_entry;
    memcpy(&m_nw_state, p_nsds, sizeof(network_stats_data_stored_t));

    return NRF_SUCCESS;
}

static void provisioner_getter(mesh_config_entry_id_t id, void * p_entry)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Provisioner getter ...\n");
    NRF_MESH_ASSERT_DEBUG(PROVISIONER_RECORD == id.record);

    network_stats_data_stored_t * p_nw_state = (network_stats_data_stored_t *) p_entry;
    memcpy(p_nw_state, &m_nw_state, sizeof(m_nw_state));
}

static void provisioner_deleter(mesh_config_entry_id_t id)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Provisioner deleter ...\n");
    NRF_MESH_ASSERT_DEBUG(PROVISIONER_RECORD == id.record);

    /* Clear and set default values. */
    memset(&m_nw_state, 0x00, sizeof(m_nw_state));
}

static void provisioner_store(void)
{
    mesh_config_entry_id_t id = PROVISIONER_ENTRY_ID;

    NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &m_nw_state));
}

static void provisioner_invalidate(void)
{
    /* Stop scanner. */
    prov_helper_scan_stop();
    /* Delete all old values and remove from mesh config. */
    (void)mesh_config_entry_delete(PROVISIONER_ENTRY_ID);
}

/*****************************************************************************/

static void app_data_store_cb(void)
{
    provisioner_store();
}

/*****************************************************************************/
/**** Configuration process related callbacks ****/

static void app_config_successful_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u successful\n", m_nw_state.configured_devices);

    hal_led_pin_set(APP_CONFIGURATION_LED, 0);
    hal_led_pin_set(APP_PROVISIONING_LED, 1);

    m_nw_state.configured_devices++;
    provisioner_store();
    prov_helper_provision_next_device();
    prov_helper_scan_start();
}

static void app_config_failed_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Configuration of device %u failed.\n", m_nw_state.configured_devices);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Press Button/RTT 1 to retry configuration.\n");
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Press Button/RTT 2 to start provisioning new nodes.\n");
    m_node_prov_setup_started = false;
    hal_led_pin_set(APP_CONFIGURATION_LED, 0);
}

static void app_prov_success_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning successful\n");

    hal_led_pin_set(APP_PROVISIONING_LED, 0);
    hal_led_pin_set(APP_CONFIGURATION_LED, 1);

    provisioner_store();
}

static void app_prov_failed_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning failed. Press Button 1 to retry.\n");

    m_node_prov_setup_started = false;

    hal_led_pin_set(APP_PROVISIONING_LED, 0);
}


/*****************************************************************************/
/**** Model related callbacks ****/
static void app_health_event_cb(const health_client_t * p_client, const health_client_evt_t * p_event)
{
    switch (p_event->type)
    {
        case HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED:
            __LOG(LOG_SRC_APP,
                  LOG_LEVEL_INFO,
                  "Node 0x%04x alive with %u active fault(s), RSSI: %d\n",
                  p_event->p_meta_data->src.value,
                  p_event->data.fault_status.fault_array_length,
                  ((p_event->p_meta_data->p_core_metadata->source == NRF_MESH_RX_SOURCE_SCANNER)
                       ? p_event->p_meta_data->p_core_metadata->params.scanner.rssi
                       : 0));
            break;
        default:
            break;
    }
}

static void app_config_server_event_cb(const config_server_evt_t * p_evt)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "config_server Event %d.\n", p_evt->type);

    if (p_evt->type == CONFIG_SERVER_EVT_NODE_RESET)
    {
        /* This should never return */
        hal_device_reset(0);
    }
}

static void app_config_client_event_cb(config_client_event_type_t event_type, const config_client_event_t * p_event, uint16_t length)
{
    /* USER_NOTE: Do additional processing of config client events here if required */
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Config client event\n");

    /* Pass events to the node setup helper module for further processing */
    node_setup_config_client_event_process(event_type, p_event, length);
}


static void provisioning_start(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Start provisioning procedure ...\n");

    prov_helper_provision_next_device();
    prov_helper_scan_start();

    hal_led_pin_set(APP_PROVISIONING_LED, 1);
}

static void provisioning_resume(void)
{
    if (!m_node_prov_setup_started)
    {
        /* If previously provisioned device is not configured. */
        if (m_nw_state.configured_devices < m_nw_state.provisioned_devices)
        {
            m_nw_state.provisioned_devices--;
            provisioning_start();
            m_node_prov_setup_started = true;
        }
    }
}

/** Check if all devices have been provisioned. If not, provision remaining devices.
 *  Check if all devices have been configured. If not, start configuring them.
 */
static void check_network_state(void)
{
    if (!m_node_prov_setup_started)
    {
        /* If previously provisioned device is not configured, start node setup procedure. */
        if (m_nw_state.configured_devices < m_nw_state.provisioned_devices)
        {
            /* Execute configuration */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Waiting for provisioned node to be configured ...\n");

            node_setup_start(m_nw_state.last_device_address, PROVISIONER_RETRY_COUNT,
                            m_nw_state.appkey, APPKEY_INDEX, NETKEY_INDEX, m_nw_state.current_uri);

            hal_led_pin_set(APP_CONFIGURATION_LED, 1);
        }
        else
        {
            provisioning_start();
        }

        m_node_prov_setup_started = true;
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Waiting for previous procedure to finish ...\n");
    }
}

static void app_mesh_core_event_cb (const nrf_mesh_evt_t * p_evt)
{
    /* USER_NOTE: User can insert mesh core event processing here */
    switch(p_evt->type)
    {
        /* Start user application specific functions only when stack is enabled */
        case NRF_MESH_EVT_ENABLED:
            __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Mesh evt: NRF_MESH_EVT_ENABLED\n");
#if (PERSISTENT_STORAGE)
            /* Mesh stack initialization has been completed */
            app_start();
#endif
            break;

        case NRF_MESH_EVT_CONFIG_LOAD_FAILURE:
            if (p_evt->params.config_load_failure.id.file == MESH_APP_FILE_ID)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Persistent provisioner data was corrupted. Set default values. \n");
                /* Clear and set default values. */
                memset(&m_nw_state, 0x00, sizeof(m_nw_state));
            }
            break;

        default:
            break;
    }
}

/* Binds the local models correctly with the desired keys */
void app_default_models_bind_setup(void)
{
    /* Bind health client to App key, and configure publication key */
    ERROR_CHECK(access_model_application_bind(m_dev_handles.m_health_client_instance.model_handle, m_dev_handles.m_appkey_handle));
    ERROR_CHECK(access_model_publish_application_set(m_dev_handles.m_health_client_instance.model_handle, m_dev_handles.m_appkey_handle));

    /* Bind self-config server to the self device key */
    ERROR_CHECK(config_server_bind(m_dev_handles.m_self_devkey_handle));
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");

    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
    /* This function may return if there are ongoing flash operations. */
    mesh_stack_device_reset();
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t--------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) Initiate provisioning and configuration of unprovisioned devices.\n"
    "\t\t Button/RTT 4) Clear all the states to reset the node.\n"
    "\t\t--------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        case 1:
        {
            /* Check if all devices have been provisioned or not */
            check_network_state();
            break;
        }

        case 2:
        {
            provisioning_resume();
            break;
        }

        /* Initiate node reset */
        case 4:
        {
            if (mesh_stack_is_device_provisioned())
            {
                /* Clear all the states to reset the node. */
                provisioner_invalidate();
                mesh_stack_config_clear();
            }
            node_reset();
            break;
        }

        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }
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

void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    m_dev_handles.m_netkey_handle = DSM_HANDLE_INVALID;
    m_dev_handles.m_appkey_handle = DSM_HANDLE_INVALID;
    m_dev_handles.m_self_devkey_handle = DSM_HANDLE_INVALID;

    /* This app requires following models :
     * config client : To be able to configure other devices
     * health client : To be able to interact with other health servers */
    ERROR_CHECK(config_client_init(app_config_client_event_cb));
    ERROR_CHECK(health_client_init(&m_dev_handles.m_health_client_instance, 0, app_health_event_cb));
}

static void mesh_init(void)
{
    bool device_provisioned;
    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = app_config_server_event_cb
    };

    nrf_mesh_evt_handler_add(&m_mesh_core_event_handler);
    uint32_t status = mesh_stack_init(&init_params, &device_provisioned);
    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Data in the persistent memory was corrupted.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reboot device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            ERROR_CHECK(status);
    }

    if (status == NRF_SUCCESS)
    {
        /* Initialize the provisioner */
        mesh_provisioner_init_params_t m_prov_helper_init_info =
        {
            .p_dev_data = &m_dev_handles,
            .p_nw_data = &m_nw_state,
            .netkey_idx = NETKEY_INDEX,
            .attention_duration_s = ATTENTION_DURATION_S,
            .p_data_store_cb  = app_data_store_cb,
            .p_prov_success_cb = app_prov_success_cb,
            .p_prov_failed_cb = app_prov_failed_cb
        };
        prov_helper_init(&m_prov_helper_init_info);

        if (!device_provisioned)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setup defaults: Adding keys, addresses, and bindings \n");

            prov_helper_provision_self();
            app_default_models_bind_setup();
            app_data_store_cb();
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Restored: Handles \n");
            prov_helper_device_handles_load();
        }

        node_setup_cb_set(app_config_successful_cb, app_config_failed_cb);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Static Provisioner Demo -----\n");

    ERROR_CHECK(app_timer_init());
    hal_leds_init();

#if BUTTON_BOARD
    ERROR_CHECK(hal_buttons_init(button_event_handler));
#endif

    ble_stack_init();
    mesh_init();
    node_setup_uri_check();
}

static void app_start(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Starting application ...\n");
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioned Nodes: %d, Configured Nodes: %d\n",
          m_nw_state.provisioned_devices, m_nw_state.configured_devices);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Dev key ", m_nw_state.self_devkey, NRF_MESH_KEY_SIZE);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "Net key ", m_nw_state.netkey, NRF_MESH_KEY_SIZE);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "App key ", m_nw_state.appkey, NRF_MESH_KEY_SIZE);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Press Button/RTT 1 to start provisioning and configuration process. \n");
}

static void start(void)
{
    rtt_input_enable(app_rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "<start> \n");

    ERROR_CHECK(mesh_stack_start());

#if (!PERSISTENT_STORAGE)
    app_start();
#endif

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

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
