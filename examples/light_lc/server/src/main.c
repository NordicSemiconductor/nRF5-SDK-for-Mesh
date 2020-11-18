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

/* HAL */
#include "boards.h"

/* Core */
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_gatt.h"
#include "nrf_mesh_configure.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_config_examples.h"
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "mesh_config.h"
#include "device_state_manager.h"
#include "access_config.h"
#include "proxy.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific include */
#include "app_light_lc.h"
#include "app_config.h"
#include "example_common.h"
#include "pwm_utils.h"
#include "light_lightness_utils.h"
#include "light_lc_state_utils.h"
#include "light_lc_server_property_constants.h"
#include "model_config_file.h"
#include "ble_softdevice_support.h"
#include "app_timer.h"
#include "app_scene.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION  (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE            (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Gives the light lightness element index. */
#define APP_LL_ELEMENT_INDEX    (0)
/* Gives the light LC element index. */
#define APP_LC_ELEMENT_INDEX    (1)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void mesh_events_handle(const nrf_mesh_evt_t * p_evt);
static void lightness_set_cb(const app_light_lightness_setup_server_t * p_app, uint16_t lightness);
static void lightness_get_cb(const app_light_lightness_setup_server_t * p_app, uint16_t * p_present_lightness);
static void lightness_transition_cb(const app_light_lightness_setup_server_t * p_server,
                                         uint32_t transition_time_ms, uint16_t target_lightness);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void scene_transition_lightness_cb(const app_scene_setup_server_t * p_app,
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

/* LC Setup Server and associated model structures' definition and initialization */
APP_LIGHT_LIGHTNESS_SETUP_SERVER_DEF(m_lc_server_0_ll,
                                     APP_FORCE_SEGMENTATION,
                                     APP_MIC_SIZE,
                                     lightness_set_cb,
                                     lightness_get_cb,
                                     lightness_transition_cb)
APP_LIGHT_LC_SETUP_SERVER_DEF(m_lc_server_0,
                              APP_FORCE_SEGMENTATION,
                              APP_MIC_SIZE)

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
/* Scene Setup server structure definition and initialization */
APP_SCENE_SETUP_SERVER_DEF(m_scene_server_0,
                           APP_FORCE_SEGMENTATION,
                           APP_MIC_SIZE,
                           scene_transition_lightness_cb,
                           &m_lc_server_0_ll.light_lightness_setup_server.generic_ponoff_setup_srv.generic_dtt_srv)
#endif

/* Application variable for holding instantaneous lightness value */
static uint16_t m_pwm0_present_actual_lightness;

/* PWM hardware instance and associated variables */
/* Note: PWM cycle period determines the the max value that can be used to represent 100%
 * duty cycles, therefore value scaling is required to get pwm tick value
 * between 0 and max.
 */
static APP_PWM_INSTANCE(PWM0, 1);
static app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_1CH(200, BSP_LED_0);
static pwm_utils_contex_t m_pwm = {
                                    .p_pwm = &PWM0,
                                    .p_pwm_config = &m_pwm0_config,
                                    .channel = 0
                                  };

/* Private function definitions */

/* Callback for updating the hardware state */
static void lightness_set_cb(const app_light_lightness_setup_server_t * p_app, uint16_t lightness)
{
    m_pwm0_present_actual_lightness = lightness;
    (void)pwm_utils_level_set_unsigned(&m_pwm, m_pwm0_present_actual_lightness);
}

/* Callback for reading the hardware state */
static void lightness_get_cb(const app_light_lightness_setup_server_t * p_app, uint16_t * p_present_lightness)
{
    *p_present_lightness = (m_pwm0_present_actual_lightness);
}

/* Callback for receiveing transition time. */
static void lightness_transition_cb(const app_light_lightness_setup_server_t * p_server,
                                         uint32_t transition_time_ms, uint16_t target_lightness)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target lightness: %d\n",
                                       transition_time_ms, target_lightness);
}

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
static void scene_transition_lightness_cb(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target Scene: %d\n",
                                       transition_time_ms, target_scene);
}
#endif

static void models_init_cb(void)
{
    /* Initialize the LC Setup Server and associated models */
    /* Initialize the Light Lightness Setup Server */
    APP_ERROR_CHECK(app_light_lightness_model_init(&m_lc_server_0_ll, APP_LL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Light Lightness Model handle: %d, Element index: %d\n",
          m_lc_server_0_ll.light_lightness_setup_server.model_handle,
          m_lc_server_0_ll.light_lightness_setup_server.settings.element_index);

    /* Initialize the LC server */
    APP_ERROR_CHECK(app_light_lc_model_init(&m_lc_server_0, APP_LC_ELEMENT_INDEX, &m_lc_server_0_ll));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App LC (%d, %d) = (Model Handle, element index)\n",
          m_lc_server_0.light_lc_setup_srv.model_handle,
          m_lc_server_0.light_lc_setup_srv.settings.element_index);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    /* Instantiate scene server and register light LC server to have scene support. The Light
    Lightness Setup Server is not added to the scene as it is handled throught the Light LC Setup
    Server. */
    APP_ERROR_CHECK(app_scene_model_init(&m_scene_server_0, APP_LL_ELEMENT_INDEX));
    APP_ERROR_CHECK(app_scene_model_add(&m_scene_server_0, &m_lc_server_0.scene_if));
    APP_ERROR_CHECK(app_light_lc_scene_context_set(&m_lc_server_0, &m_scene_server_0));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Scene Model handle: %d, Element index: %d\n",
          m_scene_server_0.scene_setup_server.model_handle,
          m_scene_server_0.scene_setup_server.settings.element_index);
#endif
}

/*************************************************************************************************/

static void mesh_events_handle(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_ENABLED)
    {
        bool lc_control;

        /* The onpowerup/last/actual binding is required at boot time to restore the correct state
         * of the light control model. */
        APP_ERROR_CHECK(app_light_lc_ponoff_binding(&m_lc_server_0, &lc_control));
        if (!lc_control)
        {
            /* The powerup settings are set such that the LC server isn't controlling the lightness.
             * Tell the Light Lightness server to control it */
            APP_ERROR_CHECK(app_light_lightness_binding_setup(m_lc_server_0.p_app_ll));
        }
    }
#if NRF_MESH_LOG_ENABLE
    else if (p_evt->type == NRF_MESH_EVT_CONFIG_LOAD_FAILURE)
    {
        const nrf_mesh_evt_config_load_failure_t * p_details = &p_evt->params.config_load_failure;
        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1, "Corrupted entry: file:%d record:%d reason:%d\n",
              p_details->id.file, p_details->id.record, p_details->reason);
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_DBG1, "Raw data:", (const uint8_t *)p_details->p_data, p_details->data_len);
    }
#endif
}

static void node_reset(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- Node reset  -----\n");
    model_config_file_clear();
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

static void fade_value_toggle(light_lc_setup_server_t * p_s_server, uint32_t value, uint16_t property_id,
                              uint32_t def_val)
{
    uint32_t time_fade_value;

    time_fade_value = light_lc_state_utils_property_get(p_s_server, property_id);
    time_fade_value = time_fade_value != def_val ? def_val  : value;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting property value to %d\n", time_fade_value);
    light_lc_state_utils_property_set(p_s_server, time_fade_value, property_id);
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t-----------------------------------------------------------------------------------------------\n"
    "\t\t RTT 1) Toggle setting the property values to requested values or to the default values.\n"
    "\t\t RTT 4) Clear all the states to reset the node.\n"
    "\t\t-----------------------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);
    switch (button_number)
    {
        /* Toggle setting the property values to requested value or to the app_config.h default
         * values (for testing) */
        case 1:
        {
            fade_value_toggle(&m_lc_server_0.light_lc_setup_srv, 0, LIGHT_LC_SERVER_TIME_FADE_PID,
                              LIGHT_LC_DEFAULT_PR_TIME_FADE_MS);
            fade_value_toggle(&m_lc_server_0.light_lc_setup_srv, 0, LIGHT_LC_SERVER_TIME_FADE_ON_PID,
                              LIGHT_LC_DEFAULT_PR_TIME_FADE_ON_MS);
            fade_value_toggle(&m_lc_server_0.light_lc_setup_srv, 0, LIGHT_LC_SERVER_TIME_FADE_STANDBY_AUTO_PID,
                              LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_AUTO_MS);
            fade_value_toggle(&m_lc_server_0.light_lc_setup_srv, 0, LIGHT_LC_SERVER_TIME_FADE_STANDBY_MANUAL_PID,
                              LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_MANUAL_MS);
            /* Set the Run time long for the test  - 120 seconds */
            fade_value_toggle(&m_lc_server_0.light_lc_setup_srv, 120000, LIGHT_LC_SERVER_TIME_RUN_ON_PID,
                              LIGHT_LC_DEFAULT_PR_TIME_RUN_ON_MS);
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
    if (key >= '1' && key <= '6')
    {
        uint32_t button_number = key - '1';
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
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
    /* Restores the application parameters after switching from the Provisioning service to the
     * Proxy */
    gap_params_init();
    conn_params_init();
#endif

    unicast_address_print();
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
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Data in the persistent memory was corrupted. Device starts as unprovisioned.\n");
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Reboot device before starting of the provisioning process.\n");
            break;
        case NRF_SUCCESS:
            break;
        default:
            APP_ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh LC Server Demo -----\n");

    pwm_utils_enable(&m_pwm);
    APP_ERROR_CHECK(app_timer_init());
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
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = NULL,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = NULL,
            .p_device_uri = EX_URI_LC_SERVER
        };
        APP_ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    /* NRF_MESH_EVT_ENABLED is triggered in the mesh IRQ context after the stack is fully enabled.
     * This event is used to call Model APIs for establishing bindings and publish a model state information. */
    nrf_mesh_evt_handler_add(&m_event_handler);
    APP_ERROR_CHECK(mesh_stack_start());

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
}

/* Entry-point */

int main(void)
{
    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}
