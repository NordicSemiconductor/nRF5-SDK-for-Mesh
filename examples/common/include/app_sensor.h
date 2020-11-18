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

#ifndef APP_SENSOR_H__
#define APP_SENSOR_H__

#include <stdint.h>

#include "list.h"

#include "sensor_messages.h"
#include "sensor_setup_server.h"
#include "app_timer.h"

/**
 * @defgroup APP_SENSOR Sensor Server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application level Sensor server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Sensor server model.
 *
 * The application should use the set callback provided by this module to set the hardware state. The
 * hardware state could be changed by reflecting the value provided by the set callback on the GPIO
 * or by sending this value to the connected lighting peripheral using some other interface (e.g.
 * serial interface). Similarly, the application should use the get callback provided by this module
 * to read the hardware state.
 *
 * This module triggers the set callback only when it determines that it is time to inform the user
 * application. It is possible that the client can send multiple overlapping set commands. In such
 * case any transition in progress will be abandoned and fresh transition will be started if
 * required.
 * <br>
 * @warning To comply with the @tagMeshMdlSp test cases, the application must adhere to
 * the requirements defined in the following sections:
 * - @tagMeshMdlSp section 4 (Sensors).
 * - @tagMeshSp section 3.7.6.1 (Publish).
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */


typedef uint8_t pir_data_size_t;

/* Forward declaration */
typedef struct __app_sensor_server_t app_sensor_server_t;

/* internal data for mid-app */
typedef uint8_t descriptor_status_t;

/**
 * Macro to create application level app_sensor_server_t context.
 *
 * Timer for cadence.  This is separate from when we send status when we get an interrupt from the
 * PIR sensor
 *
 * @param[in] _name                     Name of the app_sensor_server_t instance
 * @param[in] _force_segmented          If the Sensor server shall use force segmentation of messages
 * @param[in] _mic_size                 MIC size to be used by Sensor server
 * @param[in] _get_cb                   Callback for reading the state from the application.
 * @param[in] _settings_get_cb          Callback for reading settings data from the application.
 * @param[in] _setting_set_cb           Callback for setting a setting value from the application.
 * @param[in] _setting_get_cb           Callback for reading setting data from the application.
 * @param[in] _column_get_cb            Callback for reading column data from the application.
 * @param[in] _series_get_cb            Callback for reading series from the application.
 * @param[in] _property_array           The array of supported properties.
 * @param[in] _cadence_timer_ids        An array of timer Ids; one for each supported property.
 * @param[in] _min_interval_timer_ids   An array of timer Ids; one for each supported property.
 * @param[in] _descriptor_struct_array  The array of descriptors.
 * @param[in] _num_descs                The number of descriptors.
 * @param[in] _p_message_buffer         Buffer used for storing messages used by the sensor model.
 * @param[in] _message_buffer_bytes     The number of bytes available in _p_message_buffer.
*/
#define APP_SENSOR_SERVER_DEF(_name, _force_segmented, _mic_size, _get_cb,     \
                              _settings_get_cb, _setting_set_cb,               \
                              _setting_get_cb, _column_get_cb, _series_get_cb, \
                              _property_array,                                 \
                              _cadence_timer_ids,                              \
                              _min_interval_timer_ids,                         \
                              _descriptor_struct_array, _num_descs,            \
                              _p_message_buffer, _message_buffer_bytes);       \
    static uint8_t m_descriptor_buf[_num_descs * SENSOR_DESCRIPTOR_MSG_SIZE];  \
    static app_sensor_server_t _name =                                         \
    {                                                                          \
        .server.settings.force_segmented = _force_segmented,                   \
        .server.settings.transmic_size = _mic_size,                            \
        .sensor_get_cb = _get_cb,                                              \
        .sensor_settings_get_cb = _settings_get_cb,                            \
        .sensor_setting_set_cb = _setting_set_cb,                              \
        .sensor_setting_get_cb = _setting_get_cb,                              \
        .sensor_column_get_cb = _column_get_cb,                                \
        .sensor_series_get_cb = _series_get_cb,                                \
        .p_sensor_property_array = _property_array,                            \
        .p_cadence_timer_ids = _cadence_timer_ids,                             \
        .p_min_interval_timer_ids = _min_interval_timer_ids,                   \
        .p_sensor_descriptor = _descriptor_struct_array,                       \
        .p_descriptor_message = m_descriptor_buf,                              \
        .sensor_num_desc = _num_descs,                                         \
        .p_message_buffer = _p_message_buffer,                                 \
        .message_buffer_bytes = _message_buffer_bytes                          \
    };


/** Internal structure to hold state.  Since settings and data are read directly from the
 * hardware, those aren't stored here.
 */
typedef struct
{
    list_node_t * p_cadence_list;
    uint16_t marshalled_list_bytes;
} app_sensor_state_t;

/** Application state read callback prototype.
 * This callback is called by this module whenever application sensor state is required
 * to be read.
 *
 * @param[in]  p_server          Pointer to @ref __app_sensor_server_t [app_sensor_server_t] context
 * @param[in]  property_id       Property id of the sensor to be read
 * @param[in]  p_out             The address of the response message.
 * @param[out] p_out_bytes       The number of message bytes at p_out.
 *
 */
typedef void (*app_sensor_get_cb_t)(const app_sensor_server_t * p_server,
                                    uint16_t property_id,
                                    uint8_t * p_out,
                                    uint16_t * p_out_bytes);


/** Application state read settings callback prototype.
 * This callback is called by this module whenever application settings state is
 * required to be read.
 *
 * @param[in]  p_server          Pointer to @ref __app_sensor_server_t [app_sensor_server_t]
 *                               context
 * @param[in]  property_id       Property id of the sensor to query
 * @param[out] p_out             Pointer to the response message.
 * @param[out] p_out_bytes       The number of message bytes at *pp_out.
 */
typedef void (*app_sensor_settings_get_cb_t)(const app_sensor_server_t * p_server,
                                             uint16_t property_id,
                                             sensor_settings_status_msg_pkt_t * p_out,
                                             uint16_t * p_out_bytes);

/** Application state set setting callback prototype.
 *
 * This callback is called by the this module whenever application is required to be informed to
 * reflect the desired setting state value, as a result of the received SET setting message.
 *
 * @param[in]  p_server            Pointer to @ref __app_sensor_server_t [app_sensor_server_t] context
 * @param[in]  property_id         Property id of the sensor to be set
 * @param[in]  setting_property_id Setting property id of the sensor setting to be set
 * @param[in]  p_in                The setting set command.
 * @param[in]  in_bytes            The number of bytes in p_in.
 * @param[out] p_out               Pointer to the response message.
 * @param[out] p_out_bytes         The number of message bytes at *pp_out.
 */
typedef void (*app_sensor_setting_set_cb_t)(const app_sensor_server_t * p_server,
                                            uint16_t property_id,
                                            uint16_t setting_property_id,
                                            const sensor_setting_set_msg_pkt_t * p_in,
                                            uint16_t in_bytes,
                                            sensor_setting_status_msg_pkt_t * p_out,
                                            uint16_t * p_out_bytes);

/** Application state read single setting callback prototype.
 * This callback is called by this module whenever a setting state is
 * required to be read.
 *
 * @param[in]  p_server            Pointer to @ref __app_sensor_server_t [app_sensor_server_t] context
 * @param[in]  property_id         Property id of the sensor to query
 * @param[in]  setting_property_id Setting property id of the sensor setting to be set
 * @param[out] p_out               Pointer to the response message.
 * @param[out] p_out_bytes         The number of message bytes at *pp_out.
 */
typedef void (*app_sensor_setting_get_cb_t)(const app_sensor_server_t * p_server,
                                            uint16_t property_id,
                                            uint16_t setting_property_id,
                                            sensor_setting_status_msg_pkt_t * p_out,
                                            uint16_t * p_out_bytes);

/** Application state read column callback prototype.
 *
 * This callback is called by the app_sensor.c whenever application sensor state is required
 * to be read. The User application provides the pointer to formatted data in `pp_out`.
 *
 * @note Unlike other model APIs, the buffer is not provided by the `app_sensor.c` module.
 * Instead the user application must allocate some buffer of desired size and provide
 * a pointer to it. This buffer must contain a formatted Sensor Column Status message data.
 *
 * @param[in]  p_server          Pointer to @ref __app_sensor_server_t [app_sensor_server_t]
 *                               context.
 * @param[in]  p_in              Pointer to the Sensor Column Get message parameters. This contains
 *                               requested property id.
 * @param[in]  in_bytes          Pointer to the number of bytes at p_in.
 * @param[out] p_out             Pointer to the response message buffer. This buffer must be
 *                               provided by the application.
 * @param[out] p_out_bytes       The number of message bytes at *pp_out.
 */
typedef void (*app_sensor_column_get_cb_t)(const app_sensor_server_t * p_server,
                                           const sensor_column_get_msg_pkt_t * p_in,
                                           uint16_t in_bytes,
                                           sensor_column_status_msg_pkt_t * p_out,
                                           uint16_t * p_out_bytes);

/** Application state read series callback prototype.
 * This callback is called by the app_sensor.c whenever application sensor state is required
 * to be read. The User application provides the pointer to formatted data in `pp_out`.
 *
 * @note Unlike other model APIs, the buffer is not provided by the `app_sensor.c` module.
 * Instead the user application must allocate some buffer of desired size and provide
 * a pointer to it. This buffer must contain a formatted Sensor Series Status message data.
 *
 * @param[in]  p_server          Pointer to @ref __app_sensor_server_t [app_sensor_server_t]
 *                               context
 * @param[in]  p_in              Pointer to the Sensor Series Get message parameters. This contains
 *                               requested property id.
 * @param[in]  in_bytes          Pointer to the number of bytes at p_in.
 * @param[out] p_out             Pointer to the response message. This buffer must be
 *                               provided by the application.
 * @param[out] p_out_bytes       The number of message bytes at *pp_out.
 */
typedef void (*app_sensor_series_get_cb_t)(const app_sensor_server_t * p_server,
                                           const sensor_series_get_msg_pkt_t * p_in,
                                           uint16_t in_bytes,
                                           sensor_series_status_msg_pkt_t * p_out,
                                           uint16_t * p_out_bytes);


/** Application level structure holding the Sensor server model
 * context and sensor state representation */
struct __app_sensor_server_t
{
    sensor_setup_server_t server;

    /** Callback to be called for requesting current sensor data from
     * the user application
     */
    app_sensor_get_cb_t  sensor_get_cb;
    /** Callback to be called for requesting settings from the user application
     */
    app_sensor_settings_get_cb_t sensor_settings_get_cb;
    /** Callback to be called for informing the user application to update the settings
     */
    app_sensor_setting_set_cb_t sensor_setting_set_cb;

    /** Callback to be called for requesting a setting from the user application
     */
    app_sensor_setting_get_cb_t sensor_setting_get_cb;
    /** Callback to be called for requesting a column of formatted sensor data from the user
        application
    */
    app_sensor_column_get_cb_t sensor_column_get_cb;
    /** Callback to be called for requesting a series of formatted sensor data from the user
        application
    */
    app_sensor_series_get_cb_t sensor_series_get_cb;

    /** Array of the supported property IDs
     */
    uint16_t * p_sensor_property_array;

    /** array of timer IDs for supporting fast cadence.
     */
    app_timer_id_t const * p_cadence_timer_ids;

    /** array of timer IDs for enforcing status min interval.
     */
    app_timer_id_t const * p_min_interval_timer_ids;

    /** Main's descriptor definition - passed in by main (main's static const) - mid app uses this to
     *  create its internal static variable in message-appropriate format.  This descriptor is not
     *  packed (not appropriate to send in a message).
     */
    /* the number of descriptors
     */
    uint16_t sensor_num_desc;
    /* the actual descriptor(s)
     */
    const sensor_descriptor_t * p_sensor_descriptor;
    /** Points to the message buffer.
     */
    uint8_t * p_message_buffer;
    /** The number of bytes of buffer space at p_message_buffer.
     */
    uint16_t message_buffer_bytes;

    /** Internal variable. Representation of the Sensor state-related data
     */
    app_sensor_state_t state;
    /** mid-app's packed version of the descriptor
     */
    descriptor_status_t * p_descriptor_message;
};

/** Initializes the behavioral module for the Sensor model
 *
 * @param[in] p_server              Pointer to the application server server structure array.
 * @param[in] element_index         Element index on which this server will be instantiated.
 *
 * @retval NRF_SUCCESS              The model is initialized successfully.
 * @retval NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                  member variable pointers.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated
 *                                  or no more subscription lists available in memory pool
 *                                  (see @ref ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index.
*/
uint32_t app_sensor_init(app_sensor_server_t * p_server, uint16_t element_index);

/** @} end of APP_SENSOR */

#endif /* APP_SENSOR_H__ */
