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

#ifndef LIGHT_LC_CLIENT_H__
#define LIGHT_LC_CLIENT_H__

#include <stdint.h>
#include "access.h"
#include "access_reliable.h"
#include "light_lc_common.h"
#include "light_lc_messages.h"

/**
 * @defgroup LIGHT_LC_CLIENT Light LC client model interface
 * @ingroup LIGHT_LC_MODELS
 *
 * @{
 */

/** Client model ID */
#define LIGHT_LC_CLIENT_MODEL_ID 0x1311

/* Forward declaration */
typedef struct __light_lc_client_t light_lc_client_t;

/**
 * Callback type for mode state related transactions
 *
 * @param[in] p_self    Pointer to the model structure
 * @param[in] p_meta    Access metadata for the received message
 * @param[in] p_in      Pointer to the input event parameters for the user application
 */
typedef void (*light_lc_mode_state_status_cb_t)(const light_lc_client_t * p_self,
                                                const access_message_rx_meta_t * p_meta,
                                                const light_lc_mode_status_params_t * p_in);

/**
 * Callback type for occupancy mode state related transactions
 *
 * @param[in] p_self    Pointer to the model structure
 * @param[in] p_meta    Access metadata for the received message
 * @param[in] p_in      Pointer to the input event parameters for the user application
 */
typedef void (*light_lc_occupancy_mode_state_status_cb_t)(const light_lc_client_t * p_self,
                                                          const access_message_rx_meta_t * p_meta,
                                                          const light_lc_occupancy_mode_status_params_t * p_in);

/**
 * Callback type for light onoff state related transactions
 *
 * @param[in] p_self    Pointer to the model structure
 * @param[in] p_meta    Access metadata for the received message
 * @param[in] p_in      Pointer to the input event parameters for the user application
 */
typedef void (*light_lc_light_onoff_state_status_cb_t)(const light_lc_client_t * p_self,
                                                       const access_message_rx_meta_t * p_meta,
                                                       const light_lc_light_onoff_status_params_t * p_in);

/**
 * Callback type for property state related transactions
 *
 * @param[in] p_self    Pointer to the model structure
 * @param[in] p_meta    Access metadata for the received message
 * @param[in] p_in      Pointer to the input event parameters for the user application
 */
typedef void (*light_lc_property_state_status_cb_t)(const light_lc_client_t * p_self,
                                                    const access_message_rx_meta_t * p_meta,
                                                    const light_lc_property_status_params_t * p_in);


typedef struct
{
    /** Client model response message callback. */
    light_lc_mode_state_status_cb_t lc_mode_status_cb;                      /**< Callback for the Light LC Mode Status message */
    light_lc_occupancy_mode_state_status_cb_t lc_occupancy_mode_status_cb;  /**< Callback for the Light LC Occupancy Mode Status message */
    light_lc_light_onoff_state_status_cb_t lc_light_onoff_status_cb;        /**< Callback for the Light LC Light OnOff Status message */
    light_lc_property_state_status_cb_t lc_property_status_cb;              /**< Callback for the Light LC Property Status message */

    /** Callback to call after the acknowledged transaction has ended. */
    access_reliable_cb_t ack_transaction_status_cb;
    /** callback called at the end of the each period for the publishing */
    access_publish_timeout_cb_t periodic_publish_cb;
} light_lc_client_callbacks_t;

/**
 * User provided settings and callbacks for the model instance
 */
typedef struct
{
    /** Reliable message timeout in microseconds. If this value is set to zero, during model
     * initialization this value will be updated to the value specified by
     * by @ref MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT. */
    uint32_t timeout;
    /** If server should force outgoing messages as segmented messages.
     *  See @ref mesh_model_force_segmented. */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages.
     * See @ref nrf_mesh_transmic_size_t and @ref mesh_model_large_mic. */
    nrf_mesh_transmic_size_t transmic_size;

    /** Callback list */
    const light_lc_client_callbacks_t * p_callbacks;
} light_lc_client_settings_t;

typedef union
{
    light_lc_mode_set_msg_pkt_t mode_set;                       /**< Storage for the Light LC Mode Set message */
    light_lc_occupancy_mode_set_msg_pkt_t occupancy_mode_set;   /**< Storage for the Light LC Occupancy Mode Set message */
    light_lc_light_onoff_set_msg_pkt_t light_onoff_set;         /**< Storage for the Light LC Light OnOff Set message */
    light_lc_property_set_msg_pkt_t property_set;               /**< Storage for the Light LC Property Set message */
    light_lc_property_get_msg_pkt_t property_get;               /**< Storage for the Light LC Property Get message */
} light_lc_client_msg_data_t;

struct __light_lc_client_t
{
    /** Model handle assigned to this instance */
    access_model_handle_t model_handle;
    /** Holds the raw message packet data for transactions */
    light_lc_client_msg_data_t msg_pkt;
    /* Acknowledged message context variable */
    access_reliable_t access_message;

    /** Model settings and callbacks for this instance */
    light_lc_client_settings_t settings;
};

/**
 * Initializes Light LC client.
 *
 * @note This function should only be called _once_.
 * @note The client handles the model allocation and adding.
 *
 * @param[in] p_client          Client model context pointer.
 * @param[in] element_index     Element index to add the model
 *
 * @retval NRF_SUCCESS          The model is initialized successfully.
 * @retval NRF_ERROR_NULL       NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM     @ref ACCESS_MODEL_COUNT number of models already allocated
 *                              or no more subscription lists available in memory pool
 *                              (see @ref ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN  Multiple model instances per element are not allowed
 *                              or changes to device composition are not allowed.
 *                              Adding a new model after device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND  Invalid access element index.
 */
uint32_t light_lc_client_init(light_lc_client_t * p_client, uint8_t element_index);

/**
 * Sends a Mode Set message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 */
uint32_t light_lc_client_mode_set(light_lc_client_t * p_client,
                                  const light_lc_mode_set_params_t * p_params);

/**
 * Sends a Mode Set Unacknowledged message to the server.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 * @param[in] repeats               Number of repetitions to use while sending unacknowledged message.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lc_client_mode_set_unack(light_lc_client_t * p_client,
                                        const light_lc_mode_set_params_t * p_params,
                                        uint8_t repeats);

/**
 * Sends a Occupancy Mode Set message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 */
uint32_t light_lc_client_occupancy_mode_set(light_lc_client_t * p_client,
                                            const light_lc_occupancy_mode_set_params_t * p_params);

/**
 * Sends a Occupancy Mode Set Unacknowledged message to the server.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 * @param[in] repeats               Number of repetitions to use while sending unacknowledged message.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lc_client_occupancy_mode_set_unack(light_lc_client_t * p_client,
                                                  const light_lc_occupancy_mode_set_params_t * p_params,
                                                  uint8_t repeats);

/**
 * Sends a Light OnOff Set message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 * @param[in] p_transition          Optional transition parameters
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 */
uint32_t light_lc_client_light_onoff_set(light_lc_client_t * p_client,
                                         const light_lc_light_onoff_set_params_t * p_params,
                                         const model_transition_t * p_transition);

/**
 * Sends a Light OnOff Set Unacknowledged message to the server.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 * @param[in] p_transition          Optional transition parameters
 * @param[in] repeats               Number of repetitions to use while sending unacknowledged message.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lc_client_light_onoff_set_unack(light_lc_client_t * p_client,
                                               const light_lc_light_onoff_set_params_t * p_params,
                                               const model_transition_t * p_transition, uint8_t repeats);

/**
 * Sends a Property Set message to the server.
 *
 * @note Expected response: Status, if the message is sent as acknowledged message.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_LENGTH Failed to create packet.
 */
uint32_t light_lc_client_property_set(light_lc_client_t * p_client,
                                      const light_lc_property_set_params_t * p_params);

/**
 * Sends a Property Set Unacknowledged message to the server.
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 * @param[in] repeats               Number of repetitions to use while sending unacknowledged message.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect transition parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 * @retval NRF_ERROR_INVALID_LENGTH Failed to create packet.
 */
uint32_t light_lc_client_property_set_unack(light_lc_client_t * p_client,
                                            const light_lc_property_set_params_t * p_params,
                                            uint8_t repeats);

/**
 * Sends a Mode Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in] p_client              Client model context pointer.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 */
uint32_t light_lc_client_mode_get(light_lc_client_t * p_client);

/**
 * Sends a Occupancy Mode Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in] p_client              Client model context pointer.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 */
uint32_t light_lc_client_occupancy_mode_get(light_lc_client_t * p_client);

/**
 * Sends a Light OnOff Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in] p_client              Client model context pointer.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 */
uint32_t light_lc_client_light_onoff_get(light_lc_client_t * p_client);

/**
 * Sends a Property Get message to the server.
 *
 * @note Expected response: Status
 *
 * @param[in] p_client              Client model context pointer.
 * @param[in] p_params              Message parameters.
 *
 * @retval NRF_SUCCESS              The message is handed over to the mesh stack for transmission.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_BUSY           The model is busy publishing another message.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 */
uint32_t light_lc_client_property_get(light_lc_client_t * p_client, const light_lc_property_get_params_t * p_params);

/**@} end of LIGHT_LC_CLIENT */
#endif /* LIGHT_LC_CLIENT_H__ */
