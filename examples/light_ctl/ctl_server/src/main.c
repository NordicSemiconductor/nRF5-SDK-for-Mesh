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

/* Example specific includes */
#include "app_light_ctl.h"
#include "app_light_lightness.h"
#include "app_config.h"
#include "example_common.h"
#include "pwm_utils.h"
#include "light_lightness_utils.h"
#include "light_ctl_utils.h"
#include "ble_softdevice_support.h"
#include "app_timer.h"
#include "model_config_file.h"
#include "mesh_config_listener.h"
#include "light_ctl_mc.h"
#include "app_scene.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
/* Client lightness parameter step size */
#define APP_CTL_LIGHTNESS_STEP_SIZE     (16384)
/* Client temperature parameter step size */
#define APP_CTL_TEMPERATURE_STEP_SIZE   (8000)
/* Client delta uv parameter step size */
#define APP_CTL_DELTA_UV_STEP_SIZE      (16384)

#define APP_TEMPERATURE_MIN             (LIGHT_CTL_TEMPERATURE_MIN_LIMIT)
#define APP_TEMPERATURE_MAX             (LIGHT_CTL_TEMPERATURE_MAX_LIMIT)

/* Delta UV is a value from -1 to 1 - converted from the int16_t value by dividing by 32768
 * (@tagMeshMdlSp section 6.1.3.4). To allow this to be seen with integer division, only dividing by
 * 327 (to show values from -100 to 100) */
#define DELTA_UV_DIVISOR_TIMES_100 (327)

#define SCALE_FACTOR (1024)

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION          (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                    (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Gives the light CTL element index. */
#define APP_CTL_ELEMENT_INDEX           (0)

/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void mesh_events_handle(const nrf_mesh_evt_t * p_evt);
static void light_ctl_set_cb(const app_light_ctl_setup_server_t * p_app, app_light_ctl_temperature_duv_hw_state_t * present_ctl_state);
static void light_ctl_get_cb(const app_light_ctl_setup_server_t * p_app, app_light_ctl_temperature_duv_hw_state_t * present_ctl_state);
static void ligth_ctl_transition_cb(const app_light_ctl_setup_server_t * p_server,
                                         uint32_t transition_time_ms,
                                         app_light_ctl_temperature_duv_hw_state_t target_ctl_state);
static void light_ctl_lightness_set_cb(const app_light_lightness_setup_server_t * p_app, uint16_t lightness);
static void light_ctl_lightness_get_cb(const app_light_lightness_setup_server_t * p_app, uint16_t * p_lightness);
static void temperature32_range_listener_cb(mesh_config_change_reason_t reason,
                                            mesh_config_entry_id_t id,
                                            const void * p_entry);
static void ligth_ctl_lightness_transition_cb(const app_light_lightness_setup_server_t * p_server,
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
static uint16_t m_range_min = APP_TEMPERATURE_MIN;
static uint16_t m_range_max = APP_TEMPERATURE_MAX;

/* CTL Setup Server and associated model structures' definition and initialization */
APP_LIGHT_LIGHTNESS_SETUP_SERVER_DEF(m_light_ctl_server_0_ll,
                                     APP_FORCE_SEGMENTATION,
                                     APP_MIC_SIZE,
                                     light_ctl_lightness_set_cb,
                                     light_ctl_lightness_get_cb,
                                     ligth_ctl_lightness_transition_cb)
APP_LIGHT_CTL_SETUP_SERVER_DEF(m_light_ctl_server_0,
                               APP_FORCE_SEGMENTATION,
                               APP_MIC_SIZE,
                               light_ctl_set_cb,
                               light_ctl_get_cb,
                               ligth_ctl_transition_cb)

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
/* Scene Setup server structure definition and initialization */
APP_SCENE_SETUP_SERVER_DEF(m_scene_server_0,
                           APP_FORCE_SEGMENTATION,
                           APP_MIC_SIZE,
                           scene_transition_lightness_cb,
                           &m_light_ctl_server_0_ll.light_lightness_setup_server.generic_ponoff_setup_srv.generic_dtt_srv)
#endif

/* Application variables for holding instantaneous CTL state values */
static app_light_ctl_temperature_duv_hw_state_t m_pwm0_present_ctl_value;
static uint16_t m_pwm0_present_actual_lightness;

/* PWM hardware instance and associated variables */
/* Note: PWM cycle period determines the the max value that can be used to represent 100%
 * duty cycles, therefore value scaling is required to get pwm tick value
 * between 0 and max.
 */
static APP_PWM_INSTANCE(PWM0, 1);
static app_pwm_config_t m_pwm0_config = APP_PWM_DEFAULT_CONFIG_2CH(200, BSP_LED_0, BSP_LED_1);
static pwm_utils_contex_t m_pwm = {
                                    .p_pwm = &PWM0,
                                    .p_pwm_config = &m_pwm0_config,
                                    .channel = 0
                                  };

MESH_CONFIG_LISTENER(m_temperature32_range_entry, LIGHT_CTL_TEMPERATURE_RANGE_EID, temperature32_range_listener_cb);

static void temperature32_range_listener_cb(mesh_config_change_reason_t reason,
                                            mesh_config_entry_id_t id,
                                            const void * p_entry)
{
    const light_ctl_temperature_range_set_params_t * p_range = p_entry;

    NRF_MESH_ASSERT(id.record == LIGHT_CTL_TEMPERATURE_RANGE_EID_START);

    m_range_min =  light_ctl_utils_temperature32_to_temperature(p_range->temperature32_range_min);
    m_range_max =  light_ctl_utils_temperature32_to_temperature(p_range->temperature32_range_max);
}

/* Private function definitions */

static void pwm_lightness_ctemp_duv_set(uint16_t lightness, uint16_t temperature, int16_t delta_uv)
{
    /* Tunable white light demonstration is shown using two LEDs on the DK board. Assume that the
     * LED1 represents a warm white light element, and the LED2 represents a cool white light
     * element. These two LEDs are turned on with different PWM duty cycles corresponding to the
     * given color temperature. Also, the overall light output is scaled by the given lightness
     * level to achieve dimming.
     *
     * The photometrically accurate implementation of the tunable white light is out of scope of
     * this example.
     */

    uint32_t warm_factor;
    uint16_t warm_level;
    uint16_t cool_level;

    if (m_range_max >= m_range_min && m_range_max >= temperature)
    {
        warm_factor = (m_range_max - temperature) * SCALE_FACTOR / (APP_TEMPERATURE_MAX - APP_TEMPERATURE_MIN);

        warm_level = (uint16_t)((uint32_t)lightness * warm_factor / SCALE_FACTOR);
        cool_level = (uint16_t)((uint32_t)lightness * (SCALE_FACTOR - warm_factor) / SCALE_FACTOR);

        __LOG(LOG_SRC_APP, LOG_LEVEL_DBG3, "warm: %05d cool: %05d duv: %05d (unused)\n",
              warm_level, cool_level, delta_uv);

        m_pwm.channel = 0;
        (void)pwm_utils_level_set_unsigned(&m_pwm, warm_level);
        m_pwm.channel = 1;
        (void)pwm_utils_level_set_unsigned(&m_pwm, cool_level);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "Invalid inputs: range_max: %d range_min: %d temperature: %d\n",
              m_range_max, m_range_min, temperature);
    }
}

/* Callback for updating the hardware state */
static void light_ctl_set_cb(const app_light_ctl_setup_server_t * p_app,
                             app_light_ctl_temperature_duv_hw_state_t * p_present_ctl_state)
{
    m_pwm0_present_ctl_value = *p_present_ctl_state;
    pwm_lightness_ctemp_duv_set(m_pwm0_present_actual_lightness,
            light_ctl_utils_temperature32_to_temperature(m_pwm0_present_ctl_value.temperature32),
            m_pwm0_present_ctl_value.delta_uv);
}

/* Callback for reading the hardware state */
static void light_ctl_get_cb(const app_light_ctl_setup_server_t * p_app,
                             app_light_ctl_temperature_duv_hw_state_t * p_present_ctl_state)
{
    p_present_ctl_state->temperature32 = m_pwm0_present_ctl_value.temperature32;
    p_present_ctl_state->delta_uv = m_pwm0_present_ctl_value.delta_uv;
}

/* Callback for implementing custom HW interface using given transition time */
static void ligth_ctl_transition_cb(const app_light_ctl_setup_server_t * p_server,
                                         uint32_t transition_time_ms,
                                         app_light_ctl_temperature_duv_hw_state_t target_ctl_state)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Transition time: %d, Target temperature: %d, Target delta_uv: %d\n",
          transition_time_ms, light_ctl_utils_temperature32_to_temperature(target_ctl_state.temperature32),
          target_ctl_state.delta_uv);
}

/* Callback for updating the hardware state for lightness */
static void light_ctl_lightness_set_cb(const app_light_lightness_setup_server_t * p_app, uint16_t lightness)
{
    m_pwm0_present_actual_lightness = lightness;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "actual lightness = %d \n", m_pwm0_present_actual_lightness);

    pwm_lightness_ctemp_duv_set(m_pwm0_present_actual_lightness,
            light_ctl_utils_temperature32_to_temperature(m_pwm0_present_ctl_value.temperature32),
            m_pwm0_present_ctl_value.delta_uv);
}

/* Callback for reading the hardware state */
static void light_ctl_lightness_get_cb(const app_light_lightness_setup_server_t * p_app, uint16_t * p_lightness)
{
    *p_lightness = m_pwm0_present_actual_lightness;
}

/* Callback for receiveing transition time. */
static void ligth_ctl_lightness_transition_cb(const app_light_lightness_setup_server_t * p_server,
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
    /* Initialize the CTL Setup Server and associated models */
    /* Initialize the Light Lightness Setup Server - the CTL main element is shared with light lightness */
    APP_ERROR_CHECK(app_light_lightness_model_init(&m_light_ctl_server_0_ll, APP_CTL_ELEMENT_INDEX));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App Light Lightness Model handle: %d, Element index: %d\n",
          m_light_ctl_server_0_ll.light_lightness_setup_server.model_handle,
          m_light_ctl_server_0_ll.light_lightness_setup_server.settings.element_index);
    /* Initialize the CTL Setup Server */
    APP_ERROR_CHECK(app_light_ctl_model_init(&m_light_ctl_server_0, APP_CTL_ELEMENT_INDEX, &m_light_ctl_server_0_ll));
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "App CTL Model Handle: %d, Element index: %d\n",
          m_light_ctl_server_0.light_ctl_setup_srv.model_handle,
          m_light_ctl_server_0.light_ctl_setup_srv.settings.element_index);

#if SCENE_SETUP_SERVER_INSTANCES_MAX > 0
    /* Instantiate scene server and register light CTL server to have scene support */
    ERROR_CHECK(app_scene_model_init(&m_scene_server_0, APP_CTL_ELEMENT_INDEX));
    ERROR_CHECK(app_scene_model_add(&m_scene_server_0, &m_light_ctl_server_0_ll.scene_if));
    ERROR_CHECK(app_light_lightness_scene_context_set(&m_light_ctl_server_0_ll, &m_scene_server_0));
    ERROR_CHECK(app_scene_model_add(&m_scene_server_0, &m_light_ctl_server_0.scene_if));
    ERROR_CHECK(app_light_ctl_scene_context_set(&m_light_ctl_server_0, &m_scene_server_0));
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
        light_ctl_temperature_range_set_params_t range_values;
        APP_ERROR_CHECK(light_ctl_mc_temperature32_range_state_get(m_light_ctl_server_0.light_ctl_setup_srv.state.handle,
                                                                   &range_values));
        m_range_min =  light_ctl_utils_temperature32_to_temperature(range_values.temperature32_range_min);
        m_range_max =  light_ctl_utils_temperature32_to_temperature(range_values.temperature32_range_max);

        /* The onpowerup/last/actual binding is required at boot time to restore the correct state
         * of the lightness model. */
        APP_ERROR_CHECK(app_light_lightness_binding_setup(&m_light_ctl_server_0_ll));

        /* Note that the mid app is expecting the lightness binding setup first - when the publish
         * occurs for the ctl binding, lightness will have already been set. */
        APP_ERROR_CHECK(app_light_ctl_binding_setup(&m_light_ctl_server_0));
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
    /* This function may return if there are ongoing flash operations. */
    model_config_file_clear();
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
    "\t\t------------------------------------------------------------------------------------------\n"
    "\t\t RTT 1) Decrease LED state and Color temperature value, until min value is reached.\n"
    "\t\t RTT 2) Increase LED state and Color temperature value, until max value is reached.\n"
    "\t\t RTT 3) Increase the Delta UV value, will wrap around when max is reached.\n"
    "\t\t RTT 4) Clear all the states to reset the node.\n"
    "\t\t------------------------------------------------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    uint16_t temperature;

    /* Increase button number because the buttons on the board is marked with 1 to 4 */
    button_number++;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    temperature = light_ctl_utils_temperature32_to_temperature(m_pwm0_present_ctl_value.temperature32);
    switch (button_number)
    {
        /* Sending value `1` or `2` via RTT will result in LED state to change and the Color
           temperature value to change and trigger the STATUS message to inform client about the
           state change. This is a demonstration of state change publication due to local event.
        */
        case 1:
        {
            m_pwm0_present_actual_lightness =
                MAX(0, (m_pwm0_present_actual_lightness - APP_CTL_LIGHTNESS_STEP_SIZE));
            temperature =
                MAX(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN, (int32_t) temperature - APP_CTL_TEMPERATURE_STEP_SIZE);
            m_pwm0_present_ctl_value.temperature32 = light_ctl_utils_temperature_to_temperature32(temperature);
            break;
        }

        case 2:
        {
            m_pwm0_present_actual_lightness =
                MIN(UINT16_MAX, ((uint32_t) m_pwm0_present_actual_lightness + APP_CTL_LIGHTNESS_STEP_SIZE));
            temperature =
                MIN(LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX, (int32_t) temperature + APP_CTL_TEMPERATURE_STEP_SIZE);
            m_pwm0_present_ctl_value.temperature32 = light_ctl_utils_temperature_to_temperature32(temperature);
            break;
        }

        case 3:
        {
            /* No checks for min or max, just let the 16 bit value wrap around */
            m_pwm0_present_ctl_value.delta_uv += APP_CTL_DELTA_UV_STEP_SIZE;
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

    if (button_number == 1 || button_number == 2 || button_number == 3)
    {
        pwm_lightness_ctemp_duv_set(m_pwm0_present_actual_lightness,
            light_ctl_utils_temperature32_to_temperature(m_pwm0_present_ctl_value.temperature32),
            m_pwm0_present_ctl_value.delta_uv);

        uint32_t status = app_light_ctl_current_value_publish(&m_light_ctl_server_0);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "lightness: %d temperature: %d delta uv: %d\n",
              m_pwm0_present_actual_lightness,
              light_ctl_utils_temperature32_to_temperature(m_pwm0_present_ctl_value.temperature32),
              m_pwm0_present_ctl_value.delta_uv);

        if ( status != NRF_SUCCESS)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Unable to publish status message, status: %d\n", status);
        }
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
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light CTL Setup Server Demo -----\n");

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
            .p_device_uri = EX_URI_CTL_SERVER
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
