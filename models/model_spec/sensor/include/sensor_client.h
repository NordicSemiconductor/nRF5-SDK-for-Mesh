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

#ifndef SENSOR_CLIENT_H__
#define SENSOR_CLIENT_H__


#include <stdint.h>
#include "access.h"
#include "access_reliable.h"
#include "sensor_common.h"
#include "sensor_messages.h"
/**
 * @defgroup SENSOR_CLIENT Sensor client model interface
 * @ingroup SENSOR_MODEL
 * @{
 */

/** Client model ID */
#define SENSOR_CLIENT_MODEL_ID 0x1102

typedef struct
{
    uint8_t format;  /* only 1 bit */
    uint8_t length;  /* 4 or 7 bits */
    uint16_t property_id; /* 11 or 16 bits */
} unpacked_marshalled_sensor_data_t;



typedef struct
{
    unpacked_marshalled_sensor_data_t mpid;
    uint8_t * raw_value;
} unpacked_sensor_data_t;


/* Forward declaration */
typedef struct __sensor_client_t sensor_client_t;

/**
 * Callback type for incoming sensor data
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input event parameters for the user application
 * @param[in]     bytes                    The number of bytes of data at p_in
 */
typedef void (*sensor_state_status_cb_t)(const sensor_client_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         const sensor_status_msg_pkt_t * p_in,
                                         uint16_t bytes);

/**
 * Callback type for sensor descriptor state related transactions
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input event parameters for the user application
 * @param[in]     num_descriptors          The number of descriptors described at p_in
 */
typedef void (*sensor_descriptor_state_status_cb_t)(const sensor_client_t * p_self,
                                                    const access_message_rx_meta_t * p_meta,
                                                    const sensor_descriptor_t * p_in,
                                                    uint16_t num_descriptors);

/**
 * Callback type for cadence related transactions
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input event parameters for the user application
 * @param[in]     length                   Length of the message (p_in) in bytes
 */
typedef void (*sensor_cadence_state_status_cb_t)(const sensor_client_t *p_self,
                                                 const access_message_rx_meta_t *p_meta,
                                                 const sensor_cadence_status_msg_pkt_t *p_in,
                                                 uint16_t length);

/**
 * Callback type for column state related transactions
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input event parameters for the user application
 * @param[in]     length                   Incoming data length
 */
typedef void (*sensor_column_state_status_cb_t)(const sensor_client_t *p_self,
                                                const access_message_rx_meta_t *p_meta,
                                                const sensor_column_status_msg_pkt_t *p_in,
                                                uint16_t length);

/**
 * Callback type for series state related transactions
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input event parameters for the user application
 * @param[in]     length                   Incoming data length
 */
typedef void (*sensor_series_state_status_cb_t)(const sensor_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const sensor_series_status_msg_pkt_t * p_in,
                                                uint16_t length);

/**
 * Callback type for settings state related transactions
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input event parameters for the user application
 * @param[in]     length                   Length of the message (p_in) in bytes
 */
typedef void (*sensor_settings_state_status_cb_t)(const sensor_client_t * p_self,
                                                  const access_message_rx_meta_t * p_meta,
                                                  const sensor_settings_status_msg_pkt_t * p_in,
                                                  uint16_t length);

/**
 * Callback type for setting state related transactions
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input event parameters for the user application
 * @param[in]     length                   Length of the message (p_in) in bytes
 */
typedef void (*sensor_setting_state_status_cb_t)(const sensor_client_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 const sensor_setting_status_msg_pkt_t * p_in,
                                                 uint16_t length);

typedef struct
{
    /** Client model response message callback. */
    sensor_state_status_cb_t sensor_status_cb;
    sensor_descriptor_state_status_cb_t sensor_descriptor_status_cb;
    sensor_cadence_state_status_cb_t sensor_cadence_status_cb;
    sensor_column_state_status_cb_t sensor_column_status_cb;
    sensor_series_state_status_cb_t sensor_series_status_cb;
    sensor_settings_state_status_cb_t sensor_settings_status_cb;
    sensor_setting_state_status_cb_t sensor_setting_status_cb;

    /** Callback to call after the acknowledged transaction has ended. */
    access_reliable_cb_t ack_transaction_status_cb;
    /** Callback called at the end of the each period for the publishing */
    access_publish_timeout_cb_t periodic_publish_cb;
} sensor_client_callbacks_t;

/**
 * User provided settings and callbacks for the model instance
 */
typedef struct
{
    /** Reliable message timeout in microseconds. If this value is set to zero, during model
     * initialization this value will be updated to the value specified by
     * by @ref MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT. */
    uint32_t timeout;
    /** If server should force outgoing messages as segmented messages
     *  See @ref mesh_model_force_segmented. */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages. See @ref nrf_mesh_transmic_size_t
     *  and @ref mesh_model_large_mic. */
    nrf_mesh_transmic_size_t transmic_size;

    /** Callback list */
    const sensor_client_callbacks_t *p_callbacks;
} sensor_client_settings_t;

/**  */
struct __sensor_client_t
{
    /** Model handle assigned to this instance */
    access_model_handle_t model_handle;

    /** Acknowledged message context variable */
    access_reliable_t access_message;

    /** Model settings and callbacks for this instance */
    sensor_client_settings_t settings;

    /** Pointer to memory to store reliable messages.
     * Provided by application and shall be global or static. */
    uint8_t * p_message_buffer;

    /** Size of memory buffer in bytes to store reliable messages.
     * Provided by application. */
    uint16_t message_buffer_bytes;
};

/**
 * Initializes Sensor Client
 *
 * @note This function should only be called _once_.
 * @note The client handles the model allocation and adding.
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     element_index            Element index to add the model
 *
 * @retval NRF_SUCCESS              The model is initialized successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated or no
 *                                  more subscription lists available in memory pool (see @ref
 *                                  ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index.
 */
uint32_t sensor_client_init(sensor_client_t *p_client, uint16_t element_index);

/**
 * Sends a Set cadence message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     p_params                 Message parameters.
 * @param[in]     length                   Length in bytes of data at p_params.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_cadence_set(sensor_client_t * p_client,
                                   const sensor_cadence_set_msg_pkt_t * p_params,
                                   uint16_t length);

/**
 * Sends a Set Cadence Unacknowledged message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     p_params                 Message parameters.
 * @param[in]     length                   Length in bytes of data at p_params.
 * @param[in]     repeats                  Number of repetitions to use while sending unacknowledged message.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to this
 *                                  destination. Wait for the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t sensor_client_cadence_set_unack(sensor_client_t * p_client,
                                         const sensor_cadence_set_msg_pkt_t * p_params,
                                         uint16_t length,
                                         uint8_t repeats);

/**
 * Sends a Set setting message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     p_params                 Message parameters.
 * @param[in]     length                   Length in bytes of data at p_params.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_setting_set(sensor_client_t * p_client,
                                   const sensor_setting_set_msg_pkt_t * p_params,
                                   uint16_t length);
/**
 * Sends a Set Setting Unacknowledged message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     p_params                 Message parameters.
 * @param[in]     length                   Length in bytes of data at p_params.
 * @param[in]     repeats                  Number of repetitions to use while sending unacknowledged message.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_ADDR   The element index is greater than the number of local unicast
 *                                  addresses stored by the @ref DEVICE_STATE_MANAGER.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to this
 *                                  destination. Wait for the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t sensor_client_setting_set_unack(sensor_client_t * p_client,
                                         const sensor_setting_set_msg_pkt_t * p_params,
                                         uint16_t length,
                                         uint8_t repeats);
/**
 * Sends a Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     property_id              Property ID specifying sensor status to return
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_get(sensor_client_t * p_client, uint16_t property_id);

/**
 * Sends a Descriptor Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     property_id              Property ID specifying descriptor to return
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_descriptor_get(sensor_client_t * p_client, uint16_t property_id);

/**
 * Sends a Cadence Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     property_id              Property ID specifying cadence to return
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_cadence_get(sensor_client_t * p_client, uint16_t property_id);

/**
 * Sends a Settings Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     property_id              Property ID specifying settings to return
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_settings_get(sensor_client_t * p_client, uint16_t property_id);

/**
 * Sends a Setting Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     property_id              Property ID specifying settings to return
 * @param[in]     setting_property_id      Setting property ID specifying setting to return
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_setting_get(sensor_client_t * p_client,
                                   uint16_t property_id,
                                   uint16_t setting_property_id);

/**
 * Sends a Column Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     p_rawbuf                 Pointer to column message data.
 * @param[in]     length                   The number of bytes of data at p_rawbuf.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_column_get(sensor_client_t * p_client,
                                  sensor_column_get_msg_pkt_t * p_rawbuf,
                                  uint16_t length);

/**
 * Sends a Series Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in]     p_client                 Client model context pointer.
 * @param[in]     p_raw_colbuf             Pointer to series message data.
 * @param[in]     length                   The number of bytes of data at p_raw_colbuf.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to application key, publish address not set or
 *                                  wrong opcode format.
 * @retval NRF_ERROR_INVALID_STATE  Message already scheduled for an acknowledged transfer.
 */
uint32_t sensor_client_series_get(sensor_client_t * p_client,
                                  sensor_series_get_msg_pkt_t * p_raw_colbuf,
                                  uint16_t length);


/**@} end of SENSOR_CLIENT */
#endif /* SENSOR_CLIENT_H__ */
