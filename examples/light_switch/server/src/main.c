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
#include "simple_hal.h"
#include "app_timer.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "generic_onoff_server.h"
#include "scene_setup_server.h"
#include "model_config_file.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "light_switch_example_common.h"
#include "app_onoff.h"
#include "ble_softdevice_support.h"
#include "app_dtt.h"
#include "app_scene.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define ONOFF_SERVER_0_LED          (BSP_LED_0)
#define APP_ONOFF_ELEMENT_INDEX     (0)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION      (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                (NRF_MESH_TRANSMIC_SIZE_SMALL)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void mesh_events_handle(const nrf_mesh_evt_t * p_evt);
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff);
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff);
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff);
#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_onoff_scene_transition_cb(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene);
#endif
/*****************************************************************************
 * Static variables
 *****************************************************************************/
static bool m_device_provisioned;
static nrf_mesh_evt_handler_t m_event_handler =
{
    .evt_cb = mesh_events_handle,
};

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
/* Defaut Transition Time server structure definition and initialization */
APP_DTT_SERVER_DEF(m_dtt_server_0,
                   APP_FORCE_SEGMENTATION,
                   APP_MIC_SIZE,
                   NULL)

/* Scene Setup server structure definition and initialization */
APP_SCENE_SETUP_SERVER_DEF(m_scene_server_0,
                           APP_FORCE_SEGMENTATION,
                           APP_MIC_SIZE,
                           app_onoff_scene_transition_cb,
                           &m_dtt_server_0.server)
#endif

/* Generic OnOff server structure definition and initialization */
APP_ONOFF_SERVER_DEF(m_onoff_server_0,
                     APP_FORCE_SEGMENTATION,
                     APP_MIC_SIZE,
                     app_onoff_server_set_cb,
                     app_onoff_server_get_cb,
                     app_onoff_server_transition_cb)

/* Callback for updating the hardware state */
static void app_onoff_server_set_cb(const app_onoff_server_t * p_server, bool onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting GPIO value: %d\n", onoff)

    hal_led_pin_set(ONOFF_SERVER_0_LED, onoff);
}

/* Callback for reading the hardware state */
static void app_onoff_server_get_cb(const app_onoff_server_t * p_server, bool * p_present_onoff)
{
    /* Resolve the server instance here if required, this example uses only 1 instance. */

    *p_present_onoff = hal_led_pin_get(ONOFF_SERVER_0_LED);
}

/* Callback for updating the hardware state */
static void app_onoff_server_transition_cb(const app_onoff_server_t * p_server,
                                                uint32_t transition_time_ms, bool target_onoff)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target OnOff: %d\n",
                                       transition_time_ms, target_onoff);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void app_onoff_scene_transition_cb(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target Scene: %d\n",
                                       transition_time_ms, target_scene);
}
#endif

static void app_model_init(void)
{
    /* Instantiate onoff server on element index APP_ONOFF_ELEMENT_INDEX */
    ERROR_CHECK(app_onoff_init(&m_onoff_server_0, APP_ONOFF_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App OnOff Model Handle: %d\n", m_onoff_server_0.server.model_handle);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    /* Instantiate Generic Default Transition Time server as needed by Scene models */
    ERROR_CHECK(app_dtt_init(&m_dtt_server_0, APP_ONOFF_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App DTT Model Handle: %d\n", m_dtt_server_0.server.model_handle);

    /* Instantiate scene server and register onoff server to have scene support */
    ERROR_CHECK(app_scene_model_init(&m_scene_server_0, APP_ONOFF_ELEMENT_INDEX));
    ERROR_CHECK(app_scene_model_add(&m_scene_server_0, &m_onoff_server_0.scene_if));
    ERROR_CHECK(app_onoff_scene_context_set(&m_onoff_server_0, &m_scene_server_0));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Scene Model Handle: %d\n", m_scene_server_0.scene_setup_server.model_handle);
#endif
}

/*************************************************************************************************/

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_ENABLED)
    {
        APP_ERROR_CHECK(app_onoff_value_restore(&m_onoff_server_0));
    }
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    model_config_file_clear();
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_RESET);
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

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t-------------------------------------------------------------------------------\n"
    "\t\t Button/RTT 1) LED state will toggle and inform clients about the state change.\n"
    "\t\t Button/RTT 4) Clear all the states to reset the node.\n"
    "\t\t-------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Pressing SW1 on the Development Kit will result in LED state to toggle and trigger
        the STATUS message to inform client about the state change. This is a demonstration of
        state change publication due to local event. */
        case 1:
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "User action \n");
            hal_led_pin_set(ONOFF_SERVER_0_LED, !hal_led_pin_get(ONOFF_SERVER_0_LED));
            app_onoff_status_publish(&m_onoff_server_0);
            break;
        }

        /* Initiate node reset */
        case 4:
        {
            /* Clear all the states to reset the node. */
            if (mesh_stack_is_device_provisioned())
            {
#if MESH_FEATURE_GATT_PROXY_ENABLED
                (void) proxy_stop();
#endif
                mesh_stack_config_clear();
                node_reset();
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "The device is unprovisioned. Resetting has no effect.\n");
            }
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

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");
    app_model_init();
}

static void mesh_init(void)
{
    /* Initialize the application storage for models */
    model_config_file_init();

    mesh_stack_init_params_t init_params =
    {
        .core.irq_priority       = NRF_MESH_IRQ_PRIORITY_LOWEST,
        .core.lfclksrc           = DEV_BOARD_LF_CLK_CFG,
        .core.p_uuid             = NULL,
        .models.models_init_cb   = models_init_cb,
        .models.config_server_cb = config_server_evt_cb
    };

    uint32_t status = mesh_stack_init(&init_params, &m_device_provisioned);

    if (status == NRF_SUCCESS)
    {
        /* Check if application stored data is valid, if not clear all data and use default values. */
        status = model_config_file_config_apply();
    }

    switch (status)
    {
        case NRF_ERROR_INVALID_DATA:
            /* Clear model config file as loading failed */
            model_config_file_clear();
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
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_FRIEND, LOG_LEVEL_DBG1, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light Switch Server Demo -----\n");

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
            .p_device_uri = EX_URI_LS_SERVER
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    /* NRF_MESH_EVT_ENABLED is triggered in the mesh IRQ context after the stack is fully enabled.
     * This event is used to call Model APIs for establishing bindings and publish a model state information. */
    nrf_mesh_evt_handler_add(&m_event_handler);
    ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);

    hal_led_mask_set(HAL_LED_MASK_UPPER_HALF, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK_UPPER_HALF, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_START);
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
