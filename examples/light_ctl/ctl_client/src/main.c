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
#include "nrf_mesh.h"
#include "mesh_stack.h"
#include "mesh_config_entry.h"
#include "device_state_manager.h"
#include "access_config.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Models */
#include "light_ctl_client.h"
#include "light_ctl_utils.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "example_common.h"
#include "ble_softdevice_support.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
#define APP_STATE_OFF                (0)
#define APP_STATE_ON                 (1)

#define APP_UNACK_MSG_REPEAT_COUNT   (2)

/* Timeout, in seconds, to demonstrate cumulative Delta Set messages with same TID value */
#define APP_TIMEOUT_FOR_TID_CHANGE   (3)

/* Client parameter step sizes */
#define APP_LIGHTNESS_STEP_SIZE          (10000L)
#define APP_TEMPERATURE32_STEP_SIZE      ((int64_t)light_ctl_utils_temperature_to_temperature32(5000))
#define APP_DELTA_UV_STEP_SIZE           (10000L)
#define APP_TEMPERATURE32_MIN            (light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT))
#define APP_TEMPERATURE32_MAX            (light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT))

/* Controls if the model instance should force all mesh messages to be segmented messages. */
#define APP_FORCE_SEGMENTATION       (false)
/* Controls the MIC size used by the model instance for sending the mesh messages. */
#define APP_MIC_SIZE                 (NRF_MESH_TRANSMIC_SIZE_SMALL)
/* Delay value used by the client for sending messages. */
#define APP_DELAY_MS                 (0)
/* Transition time value used by the client for sending messages. */
#define APP_TRANSITION_TIME_MS       (1000)


/*****************************************************************************
 * Forward declaration of static functions
 *****************************************************************************/
static void app_light_ctl_client_publish_interval_cb(access_model_handle_t handle, void * p_self);
static void app_light_ctl_client_transaction_status_cb(access_model_handle_t model_handle,
                                                       void * p_args,
                                                       access_reliable_status_t status);
static void app_light_ctl_client_status_cb(const light_ctl_client_t * p_self,
                                           const access_message_rx_meta_t * p_meta,
                                           const light_ctl_status_params_t * p_in);
static void app_light_ctl_client_temperature_status_cb(const light_ctl_client_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const light_ctl_temperature_status_params_t * p_in);
static void app_light_ctl_client_temperature_range_status_cb(const light_ctl_client_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const light_ctl_temperature_range_status_params_t * p_in);
static void app_light_ctl_client_default_status_cb(const light_ctl_client_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const light_ctl_default_status_params_t * p_in);


/*****************************************************************************
 * Static variables
 *****************************************************************************/
static light_ctl_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static bool m_device_provisioned;
static light_ctl_set_params_t m_set_params;
static light_ctl_temperature_set_params_t m_temperature_set_params;
static light_ctl_temperature_range_set_params_t m_range_set_params;
static light_ctl_default_set_params_t m_default_set_params;

static const light_ctl_client_callbacks_t m_client_cbs =
{
    .light_ctl_status_cb = app_light_ctl_client_status_cb,
    .light_ctl_temperature_status_cb = app_light_ctl_client_temperature_status_cb,
    .light_ctl_temperature_range_status_cb = app_light_ctl_client_temperature_range_status_cb,
    .light_ctl_default_status_cb = app_light_ctl_client_default_status_cb,
    .ack_transaction_status_cb = app_light_ctl_client_transaction_status_cb,
    .periodic_publish_cb = app_light_ctl_client_publish_interval_cb
};

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

/* This callback is called periodically if model is configured for periodic publishing */
static void app_light_ctl_client_publish_interval_cb(access_model_handle_t handle, void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "Publish desired message here.\n");
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
* determine suitable course of action (e.g. re-initiate previous transaction) by using this
* callback.
*/
static void app_light_ctl_client_transaction_status_cb(access_model_handle_t model_handle,
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

/* Light CTL client model interface: Process the received status message in this callback */
static void app_light_ctl_client_status_cb(const light_ctl_client_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 const light_ctl_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "CTL client: 0x%04x, Present: lightness %d, temperature %d "
              "Target: lightness %d, temperature %d remaining time: %d ms\n",
              p_meta->src.value, p_in->present_lightness,
              light_ctl_utils_temperature32_to_temperature(p_in->present_temperature32),
              p_in->target_lightness,
              light_ctl_utils_temperature32_to_temperature(p_in->target_temperature32),
              p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "CTL client: 0x%04x, Present: lightness %d, temperature %d\n",
              p_meta->src.value, p_in->present_lightness,
              light_ctl_utils_temperature32_to_temperature(p_in->present_temperature32));
    }
}

/* Light CTL client model interface: Process the received range status message in this callback */
static void app_light_ctl_client_temperature_status_cb(const light_ctl_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const light_ctl_temperature_status_params_t * p_in)
{
    if (p_in->remaining_time_ms > 0)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "CTL client: 0x%04x, Present: temperature %d delta_uv %d"
              "Target: temperature %d delta_uv %d remaining time %d\n",
              p_meta->src.value,
              light_ctl_utils_temperature32_to_temperature(p_in->present_temperature32),
              p_in->present_delta_uv,
              light_ctl_utils_temperature32_to_temperature(p_in->target_temperature32),
              p_in->target_delta_uv, p_in->remaining_time_ms);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "CTL client: 0x%04x, Present: temperature %d delta_uv %d\n",
              p_meta->src.value,
              light_ctl_utils_temperature32_to_temperature(p_in->present_temperature32),
              p_in->present_delta_uv);
    }

}

/* Light CTL client model interface: Process the received range status message in this callback */
static void app_light_ctl_client_temperature_range_status_cb(const light_ctl_client_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const light_ctl_temperature_range_status_params_t * p_in)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "CTL client: 0x%04x, Range Temperature32 status: %s, min: %d, max: %d\n",
          p_meta->src.value, (p_in->status_code == 0) ? "success":"can't set range",
          light_ctl_utils_temperature32_to_temperature(p_in->temperature32_range_min),
          light_ctl_utils_temperature32_to_temperature(p_in->temperature32_range_max));
}

/* Light CTL client model interface: Process the received default status message in this callback */
static void app_light_ctl_client_default_status_cb(const light_ctl_client_t * p_self,
                                                   const access_message_rx_meta_t * p_meta,
                                                   const light_ctl_default_status_params_t * p_in)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "CTL client: 0x%04x, Default: lightness %d temperature %d delta_uv %d\n",
          p_meta->src.value, p_in->lightness,
          light_ctl_utils_temperature32_to_temperature(p_in->temperature32), p_in->delta_uv);
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
        node_reset();
    }
}

#if NRF_MESH_LOG_ENABLE
static const char m_usage_string[] =
    "\n"
    "\t\t---------------------------------------------------\n"
    "\t\t Button/RTT 1) Set CTL lightness increase (unack).\n"
    "\t\t Button/RTT 2) Set CTL lightness decrease (unack).\n"
    "\t\t Button/RTT 3) Set CTL temperature increase (unack).\n"
    "\t\t Button/RTT 4) Set CTL temperature decrease (unack).\n"
    "\t\t        RTT 5) Set CTL delta UV increase (unack).\n"
    "\t\t        RTT 6) Set CTL delta UV decrease (unack).\n\n"
    "\t\t        RTT 7) Send get CTL message.\n"
    "\t\t        RTT 8) Send get CTL temperature message.\n"
    "\t\t        RTT 9) Send get CTL temperature range message.\n"
    "\t\t        RTT a) Send get CTL default message.\n\n"
    "\t\t        RTT b) Send set default CTL lightness increase (unack).\n"
    "\t\t        RTT c) Send set default CTL lightness decrease (unack).\n"
    "\t\t        RTT d) Send set default CTL temperature increase (unack).\n"
    "\t\t        RTT e) Send set default CTL temperature decrease (unack).\n"
    "\t\t        RTT f) Send set default CTL delta uv increase (unack).\n"
    "\t\t        RTT g) Send set default CTL delta uv decrease (unack).\n\n"
    "\t\t        RTT h) Send set temperature range min increase (unack).\n"
    "\t\t        RTT i) Send set temperature range min decrease (unack).\n"
    "\t\t        RTT j) Send set temperature range max increase (unack).\n"
    "\t\t        RTT k) Send set temperature range max decrease (unack).\n\n"
    "\t\t        RTT l) Switch between the clients.\n"
    "\t\t---------------------------------------------------\n";
#endif

static void button_event_handler(uint32_t button_number)
{
    if (button_number < 0xa)
    {
        /* Increase button number because the buttons on the board is marked with 1 to 4 */
        button_number++;
    }
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    uint32_t status = NRF_SUCCESS;
    static uint8_t client = 0;
    model_transition_t transition_params;
    static uint32_t timestamp = 0;
    timestamp_t current_time;

    switch(button_number)
    {
        case 1:
            m_set_params.lightness =
                (m_set_params.lightness + APP_LIGHTNESS_STEP_SIZE) >= UINT16_MAX ?
                UINT16_MAX : m_set_params.lightness + APP_LIGHTNESS_STEP_SIZE;
            m_set_params.temperature32 = m_temperature_set_params.temperature32;
            m_set_params.delta_uv = m_temperature_set_params.delta_uv;
            m_set_params.tid++;
            break;
        case 2:
            m_set_params.lightness =
                (m_set_params.lightness - APP_LIGHTNESS_STEP_SIZE) <= 0 ?
                0 : m_set_params.lightness - APP_LIGHTNESS_STEP_SIZE;
            m_set_params.temperature32 = m_temperature_set_params.temperature32;
            m_set_params.delta_uv = m_temperature_set_params.delta_uv;
            m_set_params.tid++;
            break;
        case 3:
            m_temperature_set_params.temperature32 =
                (m_temperature_set_params.temperature32 + APP_TEMPERATURE32_STEP_SIZE) >= APP_TEMPERATURE32_MAX ?
                APP_TEMPERATURE32_MAX : m_temperature_set_params.temperature32 + APP_TEMPERATURE32_STEP_SIZE;
            m_temperature_set_params.tid++;
            break;
        case 4:
            m_temperature_set_params.temperature32 =
                (m_temperature_set_params.temperature32 - APP_TEMPERATURE32_STEP_SIZE) <= APP_TEMPERATURE32_MIN ?
                APP_TEMPERATURE32_MIN : m_temperature_set_params.temperature32 - APP_TEMPERATURE32_STEP_SIZE;
            m_temperature_set_params.tid++;
            break;
        case 5:
            m_temperature_set_params.delta_uv =
                (m_temperature_set_params.delta_uv + APP_DELTA_UV_STEP_SIZE) >= UINT16_MAX ?
                UINT16_MAX : m_temperature_set_params.delta_uv + APP_DELTA_UV_STEP_SIZE;
            m_temperature_set_params.tid++;
            break;
        case 6:
            m_temperature_set_params.delta_uv =
                (m_temperature_set_params.delta_uv - APP_DELTA_UV_STEP_SIZE) <= 0 ?
                0 : m_temperature_set_params.delta_uv - APP_DELTA_UV_STEP_SIZE;
            m_temperature_set_params.tid++;
            break;

        case 0x0b:
            m_default_set_params.lightness =
                (m_default_set_params.lightness + APP_LIGHTNESS_STEP_SIZE) >= UINT16_MAX ?
                UINT16_MAX : m_default_set_params.lightness + APP_LIGHTNESS_STEP_SIZE;
            // no TID for this cmd
            break;
        case 0x0c:
            m_default_set_params.lightness =
                (m_default_set_params.lightness - APP_LIGHTNESS_STEP_SIZE) <= 0 ?
                0 : m_default_set_params.lightness - APP_LIGHTNESS_STEP_SIZE;
            // no TID for this cmd
            break;

        case 0x0d:
            m_default_set_params.temperature32 =
                (m_default_set_params.temperature32 + APP_TEMPERATURE32_STEP_SIZE) >= APP_TEMPERATURE32_MAX ?
                APP_TEMPERATURE32_MAX : m_default_set_params.temperature32 + APP_TEMPERATURE32_STEP_SIZE;
            // no TID for this cmd
            break;
        case 0x0e:
            m_default_set_params.temperature32 =
                (m_default_set_params.temperature32 - APP_TEMPERATURE32_STEP_SIZE) <= APP_TEMPERATURE32_MIN ?
                APP_TEMPERATURE32_MIN : m_default_set_params.temperature32 - APP_TEMPERATURE32_STEP_SIZE;
            // no TID for this cmd
            break;

        case 0x0f:
            m_default_set_params.delta_uv =
                (m_default_set_params.delta_uv + APP_DELTA_UV_STEP_SIZE) >= UINT16_MAX ?
                UINT16_MAX : m_default_set_params.delta_uv + APP_DELTA_UV_STEP_SIZE;
            // no TID for this cmd
            break;
        case 0x10:
            m_default_set_params.delta_uv =
                (m_default_set_params.delta_uv - APP_DELTA_UV_STEP_SIZE) <= 0 ?
                0 : m_default_set_params.delta_uv - APP_DELTA_UV_STEP_SIZE;
            // no TID for this cmd
            break;

        case 0x11:
            m_range_set_params.temperature32_range_min =
                (m_range_set_params.temperature32_range_min + APP_TEMPERATURE32_STEP_SIZE) >= APP_TEMPERATURE32_MAX ?
                APP_TEMPERATURE32_MAX : m_range_set_params.temperature32_range_min + APP_TEMPERATURE32_STEP_SIZE;
            // no TID for this cmd
            break;
        case 0x12:
            m_range_set_params.temperature32_range_min =
                (m_range_set_params.temperature32_range_min - APP_TEMPERATURE32_STEP_SIZE) <= APP_TEMPERATURE32_MIN ?
                APP_TEMPERATURE32_MIN : m_range_set_params.temperature32_range_min - APP_TEMPERATURE32_STEP_SIZE;
            // no TID for this cmd
            break;
        case 0x13:
            m_range_set_params.temperature32_range_max =
                (m_range_set_params.temperature32_range_max + APP_TEMPERATURE32_STEP_SIZE) >= APP_TEMPERATURE32_MAX ?
                APP_TEMPERATURE32_MAX : m_range_set_params.temperature32_range_max + APP_TEMPERATURE32_STEP_SIZE;
            // no TID for this cmd
            break;
        case 0x14:
            m_range_set_params.temperature32_range_max =
                (m_range_set_params.temperature32_range_max - APP_TEMPERATURE32_STEP_SIZE) <=  APP_TEMPERATURE32_MIN ?
                APP_TEMPERATURE32_MIN : m_range_set_params.temperature32_range_max - APP_TEMPERATURE32_STEP_SIZE;
            // no TID for this cmd
            break;
        default:
            break;
    }

    current_time = timer_now();
    if (timestamp + SEC_TO_US(APP_TIMEOUT_FOR_TID_CHANGE) < current_time)
    {
        m_temperature_set_params.tid++;
        m_set_params.tid++;
    }
    timestamp = current_time;

    transition_params.delay_ms = APP_DELAY_MS;
    transition_params.transition_time_ms = APP_TRANSITION_TIME_MS;
    switch (button_number)
    {
        case 1:
        case 2:
            /* Demonstrate un-acknowledged transaction, using 1st client model instance, set CTL */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Sending msg: UnAck CTL Set: lightness %d, temperature %d delta_uv %d "
                  "tid: %d trans time: %d ms delay: %d ms\n\n",
                   m_set_params.lightness,
                   light_ctl_utils_temperature32_to_temperature(m_set_params.temperature32),
                   m_set_params.delta_uv,
                   m_set_params.tid, transition_params.transition_time_ms,
                   transition_params.delay_ms);
            status = light_ctl_client_set_unack(&m_clients[client],
                                                &m_set_params, &transition_params, 0);
            break;
        case 3:
        case 4:
        case 5:
        case 6:
            /* Demonstrate un-acknowledged transaction, using 1st client model instance, set CTL
             * temperature */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Sending msg: UnAck CTL Temperature Set: temperature %d delta_uv %d "
                  "tid: %d trans time: %d ms delay: %d ms\n\n",
                   light_ctl_utils_temperature32_to_temperature(m_temperature_set_params.temperature32),
                   m_temperature_set_params.delta_uv,
                   m_temperature_set_params.tid, transition_params.transition_time_ms,
                   transition_params.delay_ms);
            status = light_ctl_client_temperature_set_unack(&m_clients[client],
                                                             &m_temperature_set_params,
                                                             &transition_params, 0);
            break;

            /* Following cases are accessible only via RTT input */
        case 7:
            /* get CTL */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: CTL Get\n");
            status = light_ctl_client_get(&m_clients[client]);
            break;
        case 8:
            /* get CTL temperature */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: CTL Temperature Get\n");
            status = light_ctl_client_temperature_get(&m_clients[client]);
            break;
        case 9:
            /* get CTL temperature range */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: CTL Temperature Range Get\n");
            status = light_ctl_client_temperature_range_get(&m_clients[client]);
            break;
        case 0xa:
            /* lightness default get */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: CTL Default Get\n");
            status = light_ctl_client_default_get(&m_clients[client]);
            break;

        case 0xb:
        case 0xc:
        case 0xd:
        case 0xe:
        case 0xf:
        case 0x10:
            /* set the CTL default */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "Sending msg: UnAck CTL Default Set: lightness %d, temperature %d, delta uv %d\n",
                  m_default_set_params.lightness,
                  light_ctl_utils_temperature32_to_temperature(m_default_set_params.temperature32),
                  m_default_set_params.delta_uv);
            status = light_ctl_client_default_set_unack(&m_clients[client],
                                                        &m_default_set_params, 0);
            break;

        case 0x11:
        case 0x12:
        case 0x13:
        case 0x14:
            /* set the CTL temperature range */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sending msg: UnAck CTL Temperature Range Set: range: %d %d \n",
                  light_ctl_utils_temperature32_to_temperature(m_range_set_params.temperature32_range_min),
                  light_ctl_utils_temperature32_to_temperature(m_range_set_params.temperature32_range_max));
            status = light_ctl_client_temperature_range_set_unack(&m_clients[client],
                                                                  &m_range_set_params, 0);
            break;
        /* Switch between each client instance */
        case 0x15:
            client++;
            client = (client < CLIENT_MODEL_INSTANCE_COUNT) ? client : 0;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Switching to client instance: %d\n", client);
            break;
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send: status: %d\n", client, status);
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client. One (or more) of the following is wrong:
             * - An application key is missing, or there is no application key bound to the model
             * - The client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets
             * the model's publication state.
             */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Publication not configured for client %u\n", client);
            break;

        default:
            ERROR_CHECK(status);
            break;
    }
}

static void rtt_input_handler(int key)
{
    uint32_t button_number = UINT32_MAX;

    if (key >= '1' && key <= '9')
    {
        button_number = key - '1';
        button_event_handler(button_number);
    }
    else if (key >= 'a' && key <= 'l')
    {
        button_number = key - 'a' + 0xa;
        button_event_handler(button_number);
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &m_client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_MIC_SIZE;

        ERROR_CHECK(light_ctl_client_init(&m_clients[i], i + 1));
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
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Light CTL Client Demo -----\n");

    m_temperature_set_params.temperature32 = APP_TEMPERATURE32_MIN;
    m_temperature_set_params.delta_uv = UINT16_MAX/2;
    m_range_set_params.temperature32_range_min = APP_TEMPERATURE32_MIN;
    m_range_set_params.temperature32_range_max = APP_TEMPERATURE32_MAX;
    m_default_set_params.temperature32 = APP_TEMPERATURE32_MIN;
    m_default_set_params.delta_uv = UINT16_MAX/2;

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
    rtt_input_enable(rtt_input_handler, RTT_INPUT_POLL_PERIOD_MS);

    if (!m_device_provisioned)
    {
        static const uint8_t static_auth_data[NRF_MESH_KEY_SIZE] = STATIC_AUTH_DATA;
        mesh_provisionee_start_params_t prov_start_params =
        {
            .p_static_data    = static_auth_data,
            .prov_complete_cb = provisioning_complete_cb,
            .prov_device_identification_start_cb = device_identification_start_cb,
            .prov_device_identification_stop_cb = NULL,
            .prov_abort_cb = provisioning_aborted_cb,
            .p_device_uri = EX_URI_CTL_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
    }
    else
    {
        unicast_address_print();
    }

    mesh_app_uuid_print(nrf_mesh_configure_device_uuid_get());

    ERROR_CHECK(mesh_stack_start());

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
