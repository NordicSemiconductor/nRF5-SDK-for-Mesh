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
#include "proxy.h"
#include "nrf_power.h"
#include "mesh_config_entry.h"
#include "mesh_config.h"


/* Provisioning and configuration */
#include "mesh_provisionee.h"
#include "mesh_app_utils.h"

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

/* Example specific includes */
#include "app_config.h"
#include "example_common.h"
#include "nrf_mesh_config_examples.h"
#include "ble_softdevice_support.h"

#include "sensor_utils.h"
#include "app_sensor.h"
#include "app_sensor_utils.h"

#define UNSPECIFIED 0

#define APP_SENSOR_ELEMENT_INDEX     (0)

static bool m_device_provisioned;

/* Callback function forward declarations */

static void app_sensor_get_cb(const app_sensor_server_t * p_server,
                              uint16_t property_id,
                              uint8_t * p_sensor_data,
                              uint16_t * p_out_bytes);

static void app_sensor_settings_get_cb(const app_sensor_server_t * p_server,
                                       uint16_t property_id,
                                       sensor_settings_status_msg_pkt_t * p_out,
                                       uint16_t * p_out_bytes);

static void app_sensor_setting_set_cb(const app_sensor_server_t * p_server,
                                      uint16_t property_id,
                                      uint16_t setting_property_id,
                                      const sensor_setting_set_msg_pkt_t * p_in,
                                      uint16_t in_len,
                                      sensor_setting_status_msg_pkt_t * p_out,
                                      uint16_t * p_out_bytes);

static void app_sensor_setting_get_cb(const app_sensor_server_t * p_server,
                                      uint16_t property_id,
                                      uint16_t setting_property_id,
                                      sensor_setting_status_msg_pkt_t * p_out,
                                      uint16_t * p_out_bytes);

static void app_sensor_column_get_cb(const app_sensor_server_t * p_server,
                                     const sensor_column_get_msg_pkt_t * p_in,
                                     uint16_t in_len,
                                     sensor_column_status_msg_pkt_t * p_out,
                                     uint16_t * p_out_len);

static void app_sensor_series_get_cb(const app_sensor_server_t * p_server,
                                     const sensor_series_get_msg_pkt_t * p_in,
                                     uint16_t in_len,
                                     sensor_series_status_msg_pkt_t * p_out,
                                     uint16_t * p_out_len);

/* Descriptor for this particular sensor (static) */
#define NUM_DESCRIPTORS 1

/* Server structure definitions */

static const sensor_descriptor_t m_pir_descriptor[NUM_DESCRIPTORS] =
{
    {
        .property_id = SENSOR_MOTION_SENSED_PROPERTY_ID,
        .positive_tolerance = UNSPECIFIED,
        .negative_tolerance = UNSPECIFIED,
        .sampling_function = UNSPECIFIED,
        .measurement_period = UNSPECIFIED,
        .update_interval = UNSPECIFIED
    }
};

/* The first item in the array gives the number of listed/supported property ids.
 */
static uint16_t property_array[] = {1, SENSOR_MOTION_SENSED_PROPERTY_ID};

/* Define a cadence timer and a min interval timer for each of the properties that the
 * server supports.
 */
APP_TIMER_DEF(m_sensor_server_0_cadence_timer_0);
APP_TIMER_DEF(m_sensor_server_0_min_interval_timer_0);

static app_timer_id_t cadence_timer_ids[1] =
{
    &m_sensor_server_0_cadence_timer_0_data
};

static app_timer_id_t min_interval_timer_ids[1] =
{
    &m_sensor_server_0_min_interval_timer_0_data
};

static uint8_t m_message_buffer[APP_CONFIG_MAX_MESSAGE_BYTES];

APP_SENSOR_SERVER_DEF(m_sensor_server_0,
                      APP_CONFIG_FORCE_SEGMENTATION,
                      APP_CONFIG_MIC_SIZE,
                      app_sensor_get_cb,
                      app_sensor_settings_get_cb,
                      app_sensor_setting_set_cb,
                      app_sensor_setting_get_cb,
                      app_sensor_column_get_cb,
                      app_sensor_series_get_cb,
                      property_array,
                      cadence_timer_ids,
                      min_interval_timer_ids,
                      m_pir_descriptor,
                      NUM_DESCRIPTORS,
                      m_message_buffer,
                      sizeof(m_message_buffer))



static uint8_t m_pir_motion_sensed_in_period;

static void app_sensor_get_cb(const app_sensor_server_t * p_server,
                              uint16_t property_id,
                              uint8_t * p_out,
                              uint16_t * p_out_bytes)
{
    uint16_t required_out_bytes;

    /* The required buffer size can vary with property ID.
     */
    switch (property_id) {
    case SENSOR_NO_PROPERTY_ID:
        /* Return data from all sensors.
         * This demo supports 1 sensor that provides 1 byte measurements.
         */
        required_out_bytes = 1;
        break;

    case SENSOR_MOTION_SENSED_PROPERTY_ID:
        required_out_bytes = 1;
        break;

    default:
        /* Error - this demo only supports 1 property ID. */
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "unsupported property id = 0x%04x (%d)\n",
              property_id,
              property_id);

        required_out_bytes = *p_out_bytes + 1;
        break;
    }

    if (*p_out_bytes < required_out_bytes)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "inadequate buffer (0x%04x (%d), %d, %d) = (property id, required, actual)\n",
              property_id,
              property_id,
              required_out_bytes,
              *p_out_bytes);

        *p_out_bytes = 0;
        return;
    }

    /* Convert the 0-100 pir value to a percentage 8 - re: @tagMeshDevPr, section 3.8.1
     * Motion Sensed property.
     *
     * Percentage 8 has a binary exponent of -1, so if we could have fractional percentages here,
     * that would be added in (but this simple PIR sensor does not, so we always pass in a 0 here.
     */
    p_out[0] = sensor_percentage8_create(m_pir_motion_sensed_in_period, 0);

    *p_out_bytes = 1;
}


static void app_sensor_settings_get_cb(const app_sensor_server_t * p_server,
                                       uint16_t property_id,
                                       sensor_settings_status_msg_pkt_t * p_out,
                                       uint16_t * p_out_bytes)
{
    /* return all the settings for the sensor identified by the property id */
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "no settings (0x%04x (%d)) = (property id)\n",
          property_id,
          property_id);

    /* For this DK demo, there are no sensor settings. Return the status with the property id.
     */
    *p_out_bytes = sizeof(property_id);
    p_out->property_id = property_id;
}

static void app_sensor_setting_set_cb(const app_sensor_server_t * p_server,
                                      uint16_t property_id,
                                      uint16_t setting_property_id,
                                      const sensor_setting_set_msg_pkt_t * p_in,
                                      uint16_t in_len,
                                      sensor_setting_status_msg_pkt_t * p_out,
                                      uint16_t * p_out_bytes)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "no implementation (0x%04x (%d), 0x%04x (%d)) = (property id, setting property id)\n",
        property_id,
        property_id,
        setting_property_id,
        setting_property_id);

    /* For this DK demo, there are no sensor settings. Return the status with the property id and
     * setting property id.
     */
    *p_out_bytes = sizeof(property_id) + sizeof(setting_property_id);
    p_out->property_id = property_id;
    p_out->setting_property_id = setting_property_id;
}

static void app_sensor_setting_get_cb(const app_sensor_server_t * p_server,
                                      uint16_t property_id,
                                      uint16_t setting_property_id,
                                      sensor_setting_status_msg_pkt_t * p_out,
                                      uint16_t * p_out_bytes)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "no implementation (0x%04x (%d), 0x%04x (%d)) = (property id, setting property id)\n",
        property_id,
        property_id,
        setting_property_id,
        setting_property_id);

    /* For this DK demo, there are no sensor settings. Return the status with the property id
     * and setting property id.
     */
    *p_out_bytes = sizeof(property_id) + sizeof(setting_property_id);
    p_out->property_id = property_id;
    p_out->setting_property_id = setting_property_id;
}

static void app_sensor_column_get_cb(const app_sensor_server_t * p_server,
                                     const sensor_column_get_msg_pkt_t * p_in,
                                     uint16_t in_len,
                                     sensor_column_status_msg_pkt_t * p_out,
                                     uint16_t * p_out_bytes)
{
    /* If there is no property ID, return data from all sensors.
     */
    if (SENSOR_NO_PROPERTY_ID == p_in->property_id)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "no implementation: no property id\n");
    }
    else
    {
        /* There is a property ID, return sensor data from the requested sensor.
         */
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "no implementation for property id 0x%04x\n", p_in->property_id);
    }

    /* For this DK demo, there is no column data available. Respond with the requested property id.
    */
    *p_out_bytes = sizeof(p_in->property_id);
    p_out->property_id = p_in->property_id;}

static void app_sensor_series_get_cb(const app_sensor_server_t * p_server,
                                     const sensor_series_get_msg_pkt_t * p_in,
                                     uint16_t in_len,
                                     sensor_series_status_msg_pkt_t * p_out,
                                     uint16_t * p_out_bytes)
{
    /* If there is no property ID, return data from all sensors.
     */
    if (SENSOR_NO_PROPERTY_ID == p_in->property_id)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "no implementation:, no property id\n");
    }
    else
    {
        /* There is a property ID, return sensor data from the requested sensor */
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
              "no implementation for property id 0x%04x\n", p_in->property_id);
    }

    /*  @tagMeshMdlSp: "4.3.1.2.8 Sending a Sensor Series Status message: If the requested Property ID
     *  is not recognized by the Sensor Server or if there is no Sensor Series Column state for
     *  requested Property ID, then the Raw Value X field, the Sensor Column Width field, and the Raw
     *  Value Y field shall be omitted."
     *
     *  For this DK demo, there are no series data available.
     */
    *p_out_bytes = sizeof(p_in->property_id);
    p_out->property_id = p_in->property_id;
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

static void provisioning_device_identification_start_cb(uint8_t attention_duration_sec)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device identification started\n");
    hal_led_mask_set(HAL_LED_MASK, LED_MASK_STATE_OFF);
    hal_led_blink_ms(HAL_LED_MASK_HALF,
                     LED_BLINK_ATTENTION_INTERVAL_MS,
                     LED_BLINK_ATTENTION_COUNT(attention_duration_sec));
}

static void provisioning_device_identification_stop_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Device identification stopped\n");
    hal_led_blink_stop();
}

static const char m_usage_string[] =
    "\n"
    "\t\t---------------------------------------------------\n"
    "\t\t Button/RTT 1) Decrease mocked sensor value by 1.\n"
    "\t\t Button/RTT 2) Decrease mocked sensor value by 10.\n"
    "\t\t Button/RTT 3) Increase mocked sensor value by 1.\n"
    "\t\t Button/RTT 4) Increase mocked sensor value by 10.\n"
    "\t\t        RTT 5) Clear all the states to reset the node.\n"
    "\t\t---------------------------------------------------\n";

static void button_event_handler(uint32_t button_number)
{
    button_number++; /* Increase to match number printed on DK */
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Button %u pressed\n", button_number);

    switch (button_number)
    {
        case 1:
        {
            m_pir_motion_sensed_in_period = (m_pir_motion_sensed_in_period > 1)
                                          ? m_pir_motion_sensed_in_period - 1
                                          : 0;
            break;
        }

        case 2:
        {
            m_pir_motion_sensed_in_period = (m_pir_motion_sensed_in_period > 10)
                                          ? m_pir_motion_sensed_in_period - 10
                                          : 0;
            break;
        }

        case 3:
        {
            m_pir_motion_sensed_in_period = (m_pir_motion_sensed_in_period < 99)
                                          ? m_pir_motion_sensed_in_period + 1
                                          : 100;
            break;
        }

        case 4:
        {
            m_pir_motion_sensed_in_period = (m_pir_motion_sensed_in_period < 90)
                                          ? m_pir_motion_sensed_in_period + 10
                                          : 100;
            break;
        }

        case 5:
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

    if (button_number >= 1 && button_number <= 4)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "mocked motion sensed =  %d%%\n", m_pir_motion_sensed_in_period);
        (void)sensor_status_publish(&m_sensor_server_0, SENSOR_MOTION_SENSED_PROPERTY_ID);
    }
}

static void rtt_input_handler(int key)
{
    uint32_t button_number = UINT32_MAX;

    if (key >= '1' && key <= '5')
    {
        button_number = key - '1';
        button_event_handler(button_number);
    }
    if (button_number == UINT32_MAX)
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

static void provisioning_abort_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Provisioning aborted\n");
    hal_led_blink_stop();
}

static void models_init_cb(void)
{
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "Initializing and adding models\n");

    uint32_t ret = app_sensor_init(&m_sensor_server_0, APP_SENSOR_ELEMENT_INDEX);

    APP_ERROR_CHECK(ret);

    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO,
          "App Sensor Setup Server Model Handle - no errs: %d\n",
          m_sensor_server_0.server.model_handle);
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

/*lint -e438 */
/*lint -e529 */
/* Turn off lint warnings for unused m_timer, this is used to avoid compiler
 * warning for unused static m_sensor_server_0_cadence_timer_0 */

static void initialize(void)
{
    /* Use m_sensor_server_0_cadence_timer_0.
     */
    const app_timer_id_t m_timer __attribute__((unused)) = m_sensor_server_0_cadence_timer_0;
    const app_timer_id_t m_min_interval_timer __attribute__((unused))
        = m_sensor_server_0_min_interval_timer_0;

    __LOG_INIT(LOG_SRC_APP | LOG_SRC_ACCESS, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "----- BLE Mesh Sensor Server Demo -----\n");

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
            .prov_device_identification_start_cb = provisioning_device_identification_start_cb,
            .prov_device_identification_stop_cb = provisioning_device_identification_stop_cb,
            .prov_abort_cb = provisioning_abort_cb,
            .p_device_uri = EX_URI_SENSOR_SERVER
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
    nrf_power_dcdcen_set(1);

    initialize();
    start();

    for (;;)
    {
        (void)sd_app_evt_wait();
    }
}

/*lint +e438 */
/*lint +e529 */
