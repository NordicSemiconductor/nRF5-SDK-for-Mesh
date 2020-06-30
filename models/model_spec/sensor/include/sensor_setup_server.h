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

#ifndef SENSOR_SERVER_H__
#define SENSOR_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "sensor_common.h"
#include "model_common.h"
#include "sensor_messages.h"

/**
 * @defgroup SENSOR_SERVER Sensor Setup server model interface
 * @ingroup SENSOR_MODEL
 *
 * @{
 */

/** Server model ID */
#define SENSOR_SERVER_MODEL_ID 0x1100

/** Setup server model ID */
#define SENSOR_SETUP_SERVER_MODEL_ID 0x1101

/* Forward declaration */
typedef struct __sensor_server_t sensor_server_t;

/* Forward declaration */
typedef struct __sensor_setup_server_t sensor_setup_server_t;


/**
 * Callback type for Sensor descriptor get message.
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[out]    pp_out        Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_descriptor_get_cb_t)(const sensor_setup_server_t * p_self,
                                           const access_message_rx_meta_t * p_meta,
                                           uint16_t property_id,
                                           sensor_descriptor_status_msg_pkt_t ** pp_out,
                                           uint16_t * p_out_bytes);

/**
 * Callback type for Sensor Get message.
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[out]    pp_out        Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_state_get_cb_t)(const sensor_setup_server_t * p_self,
                                      const access_message_rx_meta_t * p_meta,
                                      uint16_t property_id,
                                      sensor_status_msg_pkt_t ** pp_out,
                                      uint16_t * p_out_bytes);

/**
 * Callback type for Sensor Column Get message. (setting is singular, not settings)
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[in]     p_in          The input message
 * @param[in]     in_bytes      The number of data bytes at p_in
 * @param[out]    pp_out        Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_column_get_cb_t)(const sensor_setup_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const sensor_column_get_msg_pkt_t * p_in,
                                       uint16_t in_bytes,
                                       sensor_column_status_msg_pkt_t ** pp_out,
                                       uint16_t * p_out_bytes);

/**
 * Callback type for Sensor Series Get message. (setting is singular, not settings)
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[in]     p_in          The input message
 * @param[in]     in_bytes      The number of data bytes at p_in
 * @param[out]    pp_out        Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes at *pp_out.
 */
typedef void (*sensor_series_get_cb_t)(const sensor_setup_server_t * p_self,
                                       const access_message_rx_meta_t * p_meta,
                                       const sensor_series_get_msg_pkt_t * p_in,
                                       uint16_t in_bytes,
                                       sensor_series_status_msg_pkt_t ** pp_out,
                                       uint16_t * p_out_bytes);


/**
 * Callback type for Sensor Cadence Get message.
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[out]    pp_out        Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_cadence_get_cb_t)(const sensor_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        uint16_t property_id,
                                        sensor_cadence_status_msg_pkt_t ** pp_out,
                                        uint16_t * p_out_bytes);

/**
 * Callback type for Sensor Cadence Set message.
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[in]     p_in          The input message
 * @param[in]     in_bytes      The number of data bytes at p_in
 * @param[out]    p_out         Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_cadence_set_cb_t)(const sensor_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        uint16_t property_id,
                                        const sensor_cadence_set_msg_pkt_t * p_in,
                                        uint16_t in_bytes,
                                        sensor_cadence_status_msg_pkt_t ** pp_out,
                                        uint16_t * p_out_bytes);
/**
 * Callback type for Sensor Settings Get message.
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[out]    p_out         Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_settings_get_cb_t)(const sensor_setup_server_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         uint16_t property_id,
                                         sensor_settings_status_msg_pkt_t ** pp_out,
                                         uint16_t * p_out_bytes);

/**
 * Callback type for Sensor Setting Get message. (setting is singular, not settings)
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[in]     setting_property_id The requested setting property id.
 * @param[out]    p_out        Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_setting_get_cb_t)(const sensor_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        uint16_t property_id,
                                        uint16_t setting_property_id,
                                        sensor_setting_status_msg_pkt_t ** pp_out,
                                        uint16_t * p_out_bytes);

/**
 * Callback type for Sensor Setting Set message.
 *
 * @param[in]     p_self        Pointer to the model structure.
 * @param[in]     p_meta        Access metadata for the received message.
 * @param[in]     property_id   The requested property id (or 0 if no PID specified)
 * @param[in]     settings_property_id The requested settings property id.
 * @param[in]     p_in          Pointer to the input message
 * @param[in]     in_bytes      The number of data bytes at p_in
 * @param[out]    p_out         Pointer to the response message.
 * @param[out]    p_out_bytes   The number of response message bytes.
 */
typedef void (*sensor_setting_set_cb_t)(const sensor_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        uint16_t property_id,
                                        uint16_t settings_property_id,
                                        const sensor_setting_set_msg_pkt_t * p_in,
                                        uint16_t in_bytes,
                                        sensor_setting_status_msg_pkt_t ** pp_out,
                                        uint16_t * p_out_bytes);

/**
 * Callback type for scheduling sensor publications.
 *
 * @param[in]     p_self        Pointer to the model structure.
 */
typedef void (*sensor_publication_schedule_cb_t)(const sensor_setup_server_t * p_self);

/**
 * User provided settings and callbacks for the model instance.
 */
typedef struct
{
    /** If server should force outgoing messages as segmented messages.
     *  See @ref mesh_model_force_segmented. */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages. See @ref nrf_mesh_transmic_size_t
     *  and @ref mesh_model_large_mic. */
    nrf_mesh_transmic_size_t transmic_size;

    /* There are no callbacks for the state for this model, these callbacks are defined for the setup
     * server.
     */
} sensor_server_settings_t;

/**  */
struct __sensor_server_t
{
    /** Model handle assigned to this instance.
     */
    access_model_handle_t model_handle;

    /** Model settings and callbacks for this instance.
     */
    sensor_server_settings_t settings;
};

/**
 * Transaction callbacks for the Sensor state.
 */
typedef struct
{
    sensor_descriptor_get_cb_t       descriptor_get_cb;
    sensor_state_get_cb_t            get_cb;
    sensor_column_get_cb_t           column_get_cb;
    sensor_series_get_cb_t           series_get_cb;
    sensor_cadence_get_cb_t          cadence_get_cb;
    sensor_cadence_set_cb_t          cadence_set_cb;
    sensor_settings_get_cb_t         settings_get_cb;
    sensor_setting_get_cb_t          setting_get_cb;
    sensor_setting_set_cb_t          setting_set_cb;
    sensor_publication_schedule_cb_t publication_schedule_cb;
} sensor_setup_server_cbs_t;

/**
 * Sensor server callback list.
 */
typedef struct
{
    /** Callbacks for the level state.
     */
    sensor_setup_server_cbs_t sensor_cbs;
} sensor_setup_server_callbacks_t;

/**
 * User provided settings and callbacks for the model instance.
 */
typedef struct
{
    uint16_t element_index;
    /** If server should force outgoing messages as segmented messages.
     *  See @ref mesh_model_force_segmented. */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages. See @ref nrf_mesh_transmic_size_t
     *  and @ref mesh_model_large_mic. */
    nrf_mesh_transmic_size_t transmic_size;

    /** The list of property ids supported by this device.  The first entry is the number of property
     *  ids in the array
     */
    uint16_t * property_array;

    /** Callback list. */
    const sensor_setup_server_callbacks_t * p_callbacks;
} sensor_setup_server_settings_t;

/**  */
struct __sensor_setup_server_t
{
    /** Model handle assigned to this instance.
     */
    access_model_handle_t model_handle;

    /** Parent model context for - Sensor server.
     */
    sensor_server_t sensor_srv;

    /** Model settings and callbacks for this instance.
     */
    sensor_setup_server_settings_t settings;
};

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of
 * local action.
 *
 * @param[in]     p_server                 Status server context pointer.
 * @param[in]     p_data                   Message buffer
 * @param[in]     data_length              Length of message buffer in bytes
 * @param[in]     status_opcode            Identifies the status type
 *
 * @retval NRF_SUCCESS              The message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to this
 *                                  destination. Wait for the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t sensor_server_status_publish(const sensor_server_t * p_server,
                                      const sensor_status_msg_pkt_t * p_data,
                                      uint16_t data_length,
                                      sensor_opcode_t status_opcode);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of
 * local action.
 *
 * @param[in]     p_s_server               Status server context pointer.
 * @param[in]     p_data                   Message buffer
 * @param[in]     data_length              Length of message buffer in bytes
 * @param[in]     status_opcode            Identifies the status type
 *
 * @retval NRF_SUCCESS              The message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to this
 *                                  destination. Wait for the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t sensor_server_setup_status_publish(const sensor_setup_server_t * p_s_server,
                                            const sensor_status_msg_pkt_t * p_data,
                                            uint16_t data_length,
                                            sensor_opcode_t status_opcode);

/**
 * Initializes Sensor Setup server.
 *
 * @note The server handles the model allocation and adding.
 *
 * @param[in]     p_server                 Sensor server context pointer.
 * @param[in]     element_index            Element index to add the model to.
 *
 * @retval NRF_SUCCESS              The model is initialized successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated
 *                                  or no more subscription lists available in memory pool
 *                                  (see @ref ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index.
 */
uint32_t sensor_setup_server_init(sensor_setup_server_t * p_server, uint16_t element_index);

/**@} end of SENSOR_SERVER */
#endif /* SENSOR_SERVER_H__ */
