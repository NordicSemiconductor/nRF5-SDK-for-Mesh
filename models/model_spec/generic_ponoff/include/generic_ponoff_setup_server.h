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

#ifndef GENERIC_PONOFF_SETUP_SERVER_H__
#define GENERIC_PONOFF_SETUP_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "generic_ponoff_common.h"
#include "model_common.h"

#include "generic_onoff_server.h"
#include "generic_dtt_server.h"

/**
 * @defgroup GENERIC_PONOFF_SETUP_SERVER Generic Power OnOff Setup server model interface
 * @ingroup GENERIC_PONOFF_MODEL
 *
 * This model extends Generic PowerOnOff server, Generic OnOff server, and Generic Default
 * Transition Time server. Therefore, this model generates events for messages received by its
 * parent model.
 *
 * @{
 */

/** Server model ID */
#define GENERIC_PONOFF_SERVER_MODEL_ID 0x1006

/** Setup server model ID */
#define GENERIC_PONOFF_SETUP_SERVER_MODEL_ID 0x1007

/* Forward declaration */
typedef struct __generic_ponoff_server_t generic_ponoff_server_t;

/* Forward declaration */
typedef struct __generic_ponoff_setup_server_t generic_ponoff_setup_server_t;

/**
 * Callback type for Generic Power OnOff Set/Set Unacknowledged message.
 *
 * @param[in]     p_self                   Pointer to the model structure.
 * @param[in]     p_meta                   Access metadata for the received message.
 * @param[in]     p_in                     Pointer to the input parameters for the user application.
 * @param[out]    p_out                    Pointer to store the output parameters from the user application.
 *                                         If null, indicates that it is UNACKNOWLEDGED message and no
 *                                         output params are required.
 */
typedef void (*generic_ponoff_state_set_cb_t)(const generic_ponoff_setup_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_ponoff_set_params_t * p_in,
                                             generic_ponoff_status_params_t * p_out);

/**
 * Callback type for Generic Power OnOff Get message.
 *
 * @param[in]     p_self                   Pointer to the model structure.
 * @param[in]     p_meta                   Access metadata for the received message.
 * @param[out]    p_out                    Pointer to store the output parameters from the user application.
 */
typedef void (*generic_ponoff_state_get_cb_t)(const generic_ponoff_setup_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             generic_ponoff_status_params_t * p_out);

/**
 * Transaction callbacks for the Power OnOff state.
 */
typedef struct
{
    generic_ponoff_state_set_cb_t    set_cb;
    generic_ponoff_state_get_cb_t    get_cb;
} generic_ponoff_setup_server_state_cbs_t;

/**
 * User provided settings and callbacks for the model instance.
 */
typedef struct
{
    /** If server should force outgoing messages as segmented messages.
     *  See @ref mesh_model_force_segmented. */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages.
     * See @ref nrf_mesh_transmic_size_t and @ref mesh_model_large_mic. */
    nrf_mesh_transmic_size_t transmic_size;

    /* There are no callbacks for the state for this model, these callbacks are defined for the setup server. */
} generic_ponoff_server_settings_t;

/**  */
struct __generic_ponoff_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Parent model context - Generic OnOff server, user must provide a state callback. */
    generic_onoff_server_t generic_onoff_srv;

    /** Model settings and callbacks for this instance. */
    generic_ponoff_server_settings_t settings;
};

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of
 * local action.
 *
 * @param[in] p_server              Status server context pointer.
 * @param[in] p_params              Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer supplied to function.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect message parameters, the model not bound to application
 *                                  key, or publish address not set or wrong opcode format.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to this
 *                                  destination. Wait for the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t generic_ponoff_server_status_publish(generic_ponoff_server_t * p_server, const generic_ponoff_status_params_t * p_params);


/**
 * Default Transition Time server callback list.
 */
typedef struct
{
    /** Callback for transactions related to Power OnOff states. */
    generic_ponoff_setup_server_state_cbs_t ponoff_cbs;
} generic_ponoff_setup_server_callbacks_t;

/**
 * User provided settings and callbacks for the model instance
 */
typedef struct
{
    /** Element Index. */
    uint8_t element_index;
    /** If server should force outgoing messages as segmented messages.
     *  See @ref mesh_model_force_segmented. */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages.
     * See @ref nrf_mesh_transmic_size_t and @ref mesh_model_large_mic. */
    nrf_mesh_transmic_size_t transmic_size;

    /** Callback list */
    const generic_ponoff_setup_server_callbacks_t * p_callbacks;
} generic_ponoff_setup_server_settings_t;

/**  */
struct __generic_ponoff_setup_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Parent model context for - Generic Power OnOff server. */
    generic_ponoff_server_t generic_ponoff_srv;
    /** Parent model context for - Generic Default Transition Time server. */
    generic_dtt_server_t  generic_dtt_srv;

    /** Model settings and callbacks for this instance. */
    generic_ponoff_setup_server_settings_t settings;
};

/**
 * Initializes Generic Power OnOff Setup server.
 *
 * @note The server handles the model allocation and adding.
 *
 * @param[in]     p_server                 Generic Power OnOff server context pointer.
 * @param[in]     element_index            Element index to add the model to.
 *
 * @retval NRF_SUCCESS                  The model is initialized successfully.
 * @retval NRF_ERROR_NULL               NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM             @ref ACCESS_MODEL_COUNT number of models already allocated
 *                                      or no more subscription lists available in memory pool (see
 *                                      @ref ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN          Multiple model instances per element are not allowed or
 *                                      changes to device composition are not allowed. Adding a new
 *                                      model after device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND          Invalid access element index.
 */
uint32_t generic_ponoff_setup_server_init(generic_ponoff_setup_server_t * p_server, uint8_t element_index);

/**@} end of GENERIC_PONOFF_SETUP_SERVER */
#endif /* GENERIC_PONOFF_SETUP_SERVER_H__ */
