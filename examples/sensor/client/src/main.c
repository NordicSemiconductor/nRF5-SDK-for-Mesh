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
#include "device_state_manager.h"
#include "access_config.h"
#include "mesh_mem.h"

/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Models */
#include "sensor_client.h"
#include "sensor_messages.h"

#include "sensor_utils.h"
#include "app_sensor_utils.h"

/* Example specific includes */
#include "app_config.h"
#include "nrf_mesh_config_examples.h"
#include "example_common.h"
#include "ble_softdevice_support.h"

#define MAXIMUM_PROPERTY_DATA_BYTES 8
#define MAXIMUM_CADENCE_INSTANCE_BYTES (sizeof(sensor_cadence_t) + MAXIMUM_PROPERTY_DATA_BYTES)
#define REPEAT_NUMBER_FOR_UNACK_MESSAGES  3

static sensor_client_t m_clients[CLIENT_MODEL_INSTANCE_COUNT];
static uint8_t m_message_buffer[CLIENT_MODEL_INSTANCE_COUNT * APP_CONFIG_MAX_MESSAGE_BYTES];
static bool            m_device_provisioned;

/* Forward declaration */
static void app_sensor_client_publish_interval_cb(access_model_handle_t handle, void * p_self);

static void app_sensor_client_status_cb(const sensor_client_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        const sensor_status_msg_pkt_t * p_in,
                                        uint16_t length);

static void app_sensor_client_descriptor_status_cb(const sensor_client_t * p_self,
                                                   const access_message_rx_meta_t * p_meta,
                                                   const sensor_descriptor_t * p_in,
                                                   uint16_t num_descriptors);

static void app_sensor_client_cadence_status_cb(const sensor_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const sensor_cadence_status_msg_pkt_t * p_in,
                                                uint16_t length);

static void app_sensor_client_column_status_cb(const sensor_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const sensor_column_status_msg_pkt_t * p_in,
                                               uint16_t length);

static void app_sensor_client_series_status_cb(const sensor_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const sensor_series_status_msg_pkt_t * p_in,
                                               uint16_t length);

static void app_sensor_client_settings_status_cb(const sensor_client_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 const sensor_settings_status_msg_pkt_t * p_in,
                                                 uint16_t length);

static void app_sensor_client_setting_status_cb(const sensor_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const sensor_setting_status_msg_pkt_t * p_in,
                                                uint16_t length);

static void app_sensor_client_transaction_status_cb(access_model_handle_t model_handle,
                                                    void * p_args,
                                                    access_reliable_status_t status);

static const sensor_client_callbacks_t m_client_cbs =
{
    .sensor_status_cb = app_sensor_client_status_cb,
    .sensor_descriptor_status_cb = app_sensor_client_descriptor_status_cb,
    .sensor_cadence_status_cb = app_sensor_client_cadence_status_cb,
    .sensor_column_status_cb = app_sensor_client_column_status_cb,
    .sensor_series_status_cb = app_sensor_client_series_status_cb,
    .sensor_settings_status_cb = app_sensor_client_settings_status_cb,
    .sensor_setting_status_cb = app_sensor_client_setting_status_cb,
    .ack_transaction_status_cb = app_sensor_client_transaction_status_cb,
    .periodic_publish_cb = app_sensor_client_publish_interval_cb
};

static void device_identification_start_cb(uint8_t attention_duration_s)
{
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK_HALF,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_s));
}

static void provisioning_aborted_cb(void)
{
    hal_led_blink_stop();
}

static void provisioning_complete_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Successfully provisioned\n");

#if MESH_FEATURE_GATT_ENABLED
    /* Restores the application parameters after switching from the Provisioning service to the
       Proxy.
     */
    gap_params_init();
    conn_params_init();
#endif

    dsm_local_unicast_address_t node_address;
    dsm_local_unicast_addresses_get(&node_address);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Node Address: 0x%04x \n", node_address.address_start);

    hal_led_blink_stop();
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_INTERVAL_MS, LED_BLINK_CNT_PROV);
}

/* This callback is called periodically if model is configured for periodic publishing */
static void app_sensor_client_publish_interval_cb(access_model_handle_t handle,
                                                 void * p_self)
{
     __LOG(LOG_SRC_APP, LOG_LEVEL_WARN, "entered (%d) = (model handle)\n", handle);
}

/* Acknowledged transaction status callback, if acknowledged transfer fails, application can
 * determine suitable course of action (e.g. re-initiate previous transaction) by using this
 * callback.
 */
static void app_sensor_client_transaction_status_cb(access_model_handle_t model_handle,
                                                    void * p_args,
                                                    access_reliable_status_t status)
{
    switch (status)
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

static bool parse_sensor_status(unpacked_sensor_data_t * p_sensor_data,
                                    const uint8_t * p_msg_data,
                                    uint16_t total_length,
                                    uint16_t data_entry_count,
                                    bool * p_all_data_read)
{
    bool entry_found = false;
    uint16_t entry_num = 0;

    *p_all_data_read = false;

    uint8_t * p_msg_rover = (uint8_t *) p_msg_data;
    while (!entry_found)
    {
        p_msg_rover = sensor_marshalled_entry_parse(p_msg_rover,
                                                    &(p_sensor_data->mpid.format),
                                                    &(p_sensor_data->mpid.length),
                                                    &(p_sensor_data->mpid.property_id),
                                                    &(p_sensor_data->raw_value));
        if (data_entry_count == entry_num)
        {
            entry_found = true;
        }

        if (p_msg_rover >= p_msg_data + total_length)
        {
            /* We are at the end of the buffer. */
            *p_all_data_read = true;
            break;
        }

        entry_num++;
    }
    return entry_found;
}

/* Sensor client model interface: Process the received status message in this callback */
static void app_sensor_client_status_cb(const sensor_client_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        const sensor_status_msg_pkt_t * p_in,
                                        uint16_t length)
{
    bool all_data_read = false;
    uint16_t data_entry_count = 0;

    /* parse each sensor data entry in the received message */
    while (!all_data_read)
    {
        unpacked_sensor_data_t sensor_data;
        bool entry_found = parse_sensor_status(&sensor_data,
                                                   p_in,
                                                   length,
                                                   data_entry_count,
                                                   &all_data_read);
        if (!entry_found)
        {
            /* can't continue if we can't find each entry */
            all_data_read = true;
        }
        else
        {
            data_entry_count++;
            if (SENSOR_MOTION_SENSED_PROPERTY_ID == sensor_data.mpid.property_id)
            {
                uint8_t b_exp;
                uint8_t value = sensor_percentage8_parse(*(sensor_data.raw_value), &b_exp);
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                      "Sensor client: 0x%04x, type=%d, raw value: %d, value: %d.%d\n",
                      p_meta->src.value, p_meta->src.type,
                      *(sensor_data.raw_value),
                      value, b_exp);
            }
            else if (SENSOR_PRESENCE_DETECT_PROPERTY_ID == sensor_data.mpid.property_id)
            {
                uint8_t value = *(sensor_data.raw_value);
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                      "Sensor client: 0x%04x, type=%d, raw value: %d, value: %s\n",
                      p_meta->src.value,
                      p_meta->src.type, value,
                      value?"presence detected":"no presence detected");
            }
            else
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "unsupported property id 0x%X\n", sensor_data.mpid.property_id);
            }
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                  "format=%d, length=%d, property_id: 0x%X\n",
                  sensor_data.mpid.format,
                  sensor_data.mpid.length,
                  sensor_data.mpid.property_id);

            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "raw value =", sensor_data.raw_value, sensor_data.mpid.length);
        }
    }
}

/* Sensor client model interface:
 *  Process the received descriptor status message in this callback.
 */
static void app_sensor_client_descriptor_status_cb(const sensor_client_t * p_self,
                                                   const access_message_rx_meta_t * p_meta,
                                                   const sensor_descriptor_t * p_in,
                                                   uint16_t num_descriptors)
{
    for (uint16_t i = 0; i < num_descriptors; i++)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "Sensor client: 0x%04x, property_id: 0x%X\n",
              p_meta->src.value,
              p_in->property_id);
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "tol+: %d, tol-: %d, sample: %d, meas: %d, upd:%d\n",
              p_in[i].positive_tolerance,
              p_in[i].negative_tolerance,
              p_in[i].sampling_function,
              p_in[i].measurement_period,
              p_in[i].update_interval);
    }
}

/* Sensor client model interface:
 * Process the received cadence status message in this callback.
 */
static void app_sensor_client_cadence_status_cb(const sensor_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const sensor_cadence_status_msg_pkt_t * p_in,
                                                uint16_t bytes)
{
    uint8_t buffer[MAXIMUM_CADENCE_INSTANCE_BYTES];
    uint16_t buffer_bytes = MAXIMUM_CADENCE_INSTANCE_BYTES;
    uint16_t property_id = *(uint16_t *)p_in;
    sensor_cadence_t * p_cadence;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Sensor client: 0x%04x, cadence status received, length=%d, property_id=0x%X\n",
          p_meta->src.value,
          bytes,
          property_id);

    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "*p_in =", &p_in[0], bytes);

    p_cadence = sensor_cadence_to_buffer_deserialize((uint8_t *) p_in,
                                                     bytes,
                                                     buffer,
                                                     &buffer_bytes);

    if (p_cadence != NULL)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "cadence (%d, %d, %d) = (fast_period, trigger_type, min_interval)\n",
              p_cadence->fast_period_exponent,
              p_cadence->trigger_type,
              p_cadence->min_interval_exponent);

        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO,
                 "trigger_delta_down = ",
                 p_cadence->p_trigger_delta_down,
                 p_cadence->range_value_bytes_allocated);
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO,
                 "trigger_delta_up = ",
                 p_cadence->p_trigger_delta_up,
                 p_cadence->range_value_bytes_allocated);
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO,
                 "fast_cadence_low = ",
                 p_cadence->p_fast_cadence_low,
                 p_cadence->range_value_bytes_allocated);
        __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO,
                 "fast_cadence_high = ",
                 p_cadence->p_fast_cadence_high,
                 p_cadence->range_value_bytes_allocated);
        return;
    }

    if (buffer_bytes > sizeof(buffer))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "insufficient buffer bytes, (%d, %d) = (available, required)\n",
              sizeof(buffer),
              buffer_bytes);

    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Unsupported (%d) = (property id)\n", property_id);
    }
}

/* Sensor client model interface:
 * Process the received column status message in this callback.
 */
static void app_sensor_client_column_status_cb(const sensor_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const sensor_column_status_msg_pkt_t * p_in,
                                               uint16_t length)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Sensor client: 0x%04x, column received. Property ID: %d\n",
          p_meta->src.value, p_in->property_id);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "raw_value_xwy  = ", p_in->raw_value_xwy, length);
}

/* Sensor client model interface:
 * Process the received series status message in this callback.
 */
static void app_sensor_client_series_status_cb(const sensor_client_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const sensor_series_status_msg_pkt_t * p_in,
                                               uint16_t length)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "Sensor client: 0x%04x, series received. Property ID: %d\n",
          p_meta->src.value, p_in->property_id);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "raw_value_xwy  = ", p_in->raw_value_xwy, length);
}

/* Sensor client model interface:
 * Process the received settings status message in this callback.
 */
static void app_sensor_client_settings_status_cb(const sensor_client_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 const sensor_settings_status_msg_pkt_t * p_in,
                                                 uint16_t length)
{
    uint16_t property_id = p_in->property_id;
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor Client: 0x%04x, ->settingS<- received\n", p_meta->src.value);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "PID: 0x%X, length %d\n", property_id, length);

    if (length > sizeof(property_id))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Setting property IDs:\n");
        uint16_t number = (length - sizeof(p_in->property_id)) / sizeof(uint16_t);
        for (uint16_t i = 0; i < number; i++)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "%d(0x%04x)\n",
                  p_in->setting_property_ids[i], p_in->setting_property_ids[i]);
        }
    }
    else
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "No settings\n");
    }
}

/* Sensor client model interface:
   Process the received *setting* status message in this callback.
*/
static void app_sensor_client_setting_status_cb(const sensor_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const sensor_setting_status_msg_pkt_t * p_in,
                                                uint16_t length)
{
    uint16_t property_id = p_in->property_id;
    uint16_t setting_property_id = p_in->setting_property_id;
    uint8_t  setting_access = p_in->setting_access;

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Sensor client: 0x%04x, setting received\n", p_meta->src.value);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "PID: 0x%X, S_PID: 0x%X, access: %d\n",
          property_id,
          setting_property_id,
          setting_access);
    __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO,
             "Raw value = ",
             p_in->setting_raw,
             (length - sizeof(sensor_setting_status_msg_pkt_t)));
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

static const char m_usage_string[] =
    "\n"
    "\t\t---------------------------------------------------\n"
    "\t\t Button/RTT 1) Send a descriptor get message for all properties.\n"
    "\t\t Button/RTT 2) Send a descriptor get message for the Motion Sensed property.\n"
    "\t\t Button/RTT 3) Send a cadence get message for the Motion Sensed property.\n"
    "\t\t Button/RTT 4) Send an acknowledged cadence set message for the Motion Sensed property.\n"
    "\t\t        RTT 5) Send an unacknowledged cadence set message for the Motion Sensed property.\n"
    "\t\t        RTT 6) Send a settings get message for the Motion Sensed property.\n"
    "\t\t        RTT 7) Send a status get message for all properties.\n"
    "\t\t        RTT 8) Send a status get message for the Motion Sensed property.\n"
    "\t\t        RTT 9) Switch between the clients.\n"
    "\t\t---------------------------------------------------\n";

static void button_event_handler(uint32_t button_number)
{
    static uint8_t buffer[MAXIMUM_CADENCE_INSTANCE_BYTES];
    uint16_t buffer_bytes = sizeof(buffer);
    static uint8_t client = 0;
    sensor_cadence_t * p_cadence = NULL;
    uint16_t bytes;
    uint8_t message_buffer[APP_CONFIG_MAX_MESSAGE_BYTES];
    uint32_t status = NRF_SUCCESS;

    button_number++; /* Increase to match number printed on DK */
    /* change a setting or cadence with the buttons */
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        case 1:
            /* get descriptor */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get descriptor\n");
            status = sensor_client_descriptor_get(&m_clients[client], 0);
            break;

        case 2:
            /* get descriptor, single property */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get descriptor\n");
            status = sensor_client_descriptor_get(&m_clients[client], SENSOR_MOTION_SENSED_PROPERTY_ID);
            break;

        case 3:
             /* get cadence */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get cadence MOTION_SENSE\n");
            status = sensor_client_cadence_get(&m_clients[client],
                                                SENSOR_MOTION_SENSED_PROPERTY_ID);
            break;

        case 4:
            /* Set cadence for SENSOR_MOTION_SENSED_PROPERTY_ID, acknowledged
             */
            p_cadence = sensor_cadence_create(SENSOR_MOTION_SENSED_PROPERTY_ID, buffer, &buffer_bytes);
            if (!p_cadence)
            {
                if (buffer_bytes > sizeof(buffer))
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                          "insufficient buffer bytes, (%d, %d) = (available, required)\n",
                          sizeof(buffer),
                          buffer_bytes);
                }
                else
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                          "unsupported, (0x%04x (%d)) = property id\n",
                          SENSOR_MOTION_SENSED_PROPERTY_ID,
                          SENSOR_MOTION_SENSED_PROPERTY_ID);
                }
                break;
            }

            p_cadence->fast_period_exponent = 7; /* fast period divisor is 2^7 = 64 */
            p_cadence->trigger_type = 0;
            p_cadence->p_trigger_delta_down[0] = 1;
            p_cadence->p_trigger_delta_up[0] = 1;
            p_cadence->min_interval_exponent = 1;
            p_cadence->p_fast_cadence_low[0] = 0;
            p_cadence->p_fast_cadence_high[0] = 49;

            bytes = sizeof(message_buffer);
            sensor_cadence_to_buffer_serialize(p_cadence, message_buffer, &bytes);
            if (!bytes)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "could not serialize cadence\n");
                break;
            }

            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "sending cadence msg =", message_buffer, bytes);

            status = sensor_client_cadence_set(&m_clients[client],
                                               (sensor_cadence_set_msg_pkt_t *)message_buffer, bytes);
            break;

        case 5:
            /* Set cadence for SENSOR_MOTION_SENSED_PROPERTY_ID, unack
             */
            p_cadence = sensor_cadence_create(SENSOR_MOTION_SENSED_PROPERTY_ID, buffer, &buffer_bytes);
            if (!p_cadence)
            {
                if (buffer_bytes > sizeof(buffer))
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                          "insufficient buffer bytes, (%d, %d) = (available, required)\n",
                          sizeof(buffer),
                          buffer_bytes);
                }
                else
                {
                    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
                          "unsupported, (0x%04x (%d)) = property id\n",
                          SENSOR_MOTION_SENSED_PROPERTY_ID,
                          SENSOR_MOTION_SENSED_PROPERTY_ID);
                }
                break;
            }

            /* set the cadence - ask for any presence changes */
            p_cadence->fast_period_exponent = 0; /* fast period divisor is 2^0 = 1*/
            p_cadence->trigger_type = 0; /* keep property id meaning */
            /* request a message on every sensor change */
            p_cadence->p_trigger_delta_down[0] = 1;
            p_cadence->p_trigger_delta_up[0] = 1;
            p_cadence->min_interval_exponent = 1;
            p_cadence->p_fast_cadence_low[0] = 1;
            p_cadence->p_fast_cadence_high[0] = 0;

            bytes = sizeof(message_buffer);
            sensor_cadence_to_buffer_serialize(p_cadence, message_buffer, &bytes);
            if (!bytes)
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "could not serialize cadence\n");
                break;
            }

            __LOG_XB(LOG_SRC_APP, LOG_LEVEL_INFO, "sending cadence msg =", message_buffer, bytes);

            status = sensor_client_cadence_set_unack(&m_clients[client],
                                               (sensor_cadence_set_msg_pkt_t *)message_buffer,
                                               bytes,
                                               REPEAT_NUMBER_FOR_UNACK_MESSAGES);
            break;

        case 6:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get settings MOTION_SENSED\n");
            status = sensor_client_settings_get(&m_clients[client],
                                                SENSOR_MOTION_SENSED_PROPERTY_ID);
            break;

        case 7:
            /* get status message */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get sensor data\n");
            status = sensor_client_get(&m_clients[client], 0);
            break;

        case 8:
            /* get status message, single prop */
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "get sensor data\n");
            status = sensor_client_get(&m_clients[client], SENSOR_MOTION_SENSED_PROPERTY_ID);
            break;

        case 9:
            client++;
            client = (client < CLIENT_MODEL_INSTANCE_COUNT) ? client : 0;
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Switching to client instance: %d\n", client);
            break;

        default:
            ERROR_CHECK(NRF_ERROR_INTERNAL);
            break;
    }

    switch (status)
    {
        case NRF_SUCCESS:
            break;

        case NRF_ERROR_NO_MEM:
        case NRF_ERROR_BUSY:
        case NRF_ERROR_INVALID_STATE:
            __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Client %u cannot send\n", client);
            hal_led_blink_ms(HAL_LED_MASK, LED_BLINK_SHORT_INTERVAL_MS, LED_BLINK_CNT_NO_REPLY);
            break;

        case NRF_ERROR_INVALID_PARAM:
            /* Publication not enabled for this client.
             * One (or more) of the following is wrong:
             *
             * - An application key is missing,
             *
             * - or there is no application key bound to the model
             *
             * - or the client does not have its publication state set
             *
             * It is the provisioner that adds an application key, binds it to the model and sets the
             * model's publication state.
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
    if (button_number == UINT32_MAX)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, m_usage_string);
    }
}

static void models_init_cb(void)
{
    for (uint32_t i = 0; i < CLIENT_MODEL_INSTANCE_COUNT; ++i)
    {
        m_clients[i].settings.p_callbacks = &m_client_cbs;
        m_clients[i].settings.timeout = 0;
        m_clients[i].settings.force_segmented = APP_CONFIG_FORCE_SEGMENTATION;
        m_clients[i].settings.transmic_size = APP_CONFIG_MIC_SIZE;
        m_clients[i].p_message_buffer = &m_message_buffer[i * APP_CONFIG_MAX_MESSAGE_BYTES];
        m_clients[i].message_buffer_bytes = APP_CONFIG_MAX_MESSAGE_BYTES;

        ERROR_CHECK(sensor_client_init(&m_clients[i], i + 1));
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding model %d\n", i);
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
            APP_ERROR_CHECK(status);
    }
}

static void initialize(void)
{
    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "----- BLE Mesh Sensor Client Demo -----\n");

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
            .p_device_uri = EX_URI_SENSOR_CLIENT
        };
        ERROR_CHECK(mesh_provisionee_prov_start(&prov_start_params));
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
