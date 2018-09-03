/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#ifndef GENERIC_LEVEL_SERVER_H__
#define GENERIC_LEVEL_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "generic_level_common.h"
#include "model_common.h"

/**
 * @defgroup GENERIC_LEVEL_SERVER Generic Level server model interface
 * @ingroup GENERIC_LEVEL_MODEL
 * @{
 */

/** Server model ID */
#define GENERIC_LEVEL_SERVER_MODEL_ID 0x1002

/* Forward declaration */
typedef struct __generic_level_server_t generic_level_server_t;

/**
 * Callback type for Generic Level Set/Set Unacknowledged message
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input parameters for the user application
 * @param[in]     p_in_transition          Pointer to transition parameters, if present in the incoming message,
 *                                         otherwise set to null.
 * @param[out]    p_out                    Pointer to store the output parameters from the user application.
 *                                         If null, indicates that it is UNACKNOWLEDGED message and no
 *                                         output params are required.
 */
typedef void (*generic_level_state_set_cb_t)(const generic_level_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             const generic_level_set_params_t * p_in,
                                             const model_transition_t * p_in_transition,
                                             generic_level_status_params_t * p_out);

/**
 * Callback type for Generic Level Get message
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[out]    p_out                    Pointer to store the output parameters from the user application
 */
typedef void (*generic_level_state_get_cb_t)(const generic_level_server_t * p_self,
                                             const access_message_rx_meta_t * p_meta,
                                             generic_level_status_params_t * p_out);

/**
 * Callback type for Generic Level Delta Set/Delta Set Unacknowledged message
 *
 * @note This message callback implementation should check if the transaction is new or
 * same as previous by calling @ref model_transaction_is_new() API, and take appropriate action as
 * required by section 3.3.2.2.3 of Mesh Model Specification v1.0.
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input parameters for the user application
 * @param[in]     p_in_transition          Pointer to transition parameters, if present in the incoming message,
 *                                         otherwise set to null.
 * @param[out]    p_out                    Pointer to store the output parameters from the user application.
 *                                         If null, indicates that it is UNACKNOWLEDGED message and no
 *                                         output params are required.
 */
typedef void (*generic_level_state_delta_set_cb_t)(const generic_level_server_t * p_self,
                                                   const access_message_rx_meta_t * p_meta,
                                                   const generic_level_delta_set_params_t * p_in,
                                                   const model_transition_t * p_in_transition,
                                                   generic_level_status_params_t * p_out);


/**
 * Callback type for Generic Level Move Set/Move Set Unacknowledged message
 *
 * @param[in]     p_self                   Pointer to the model structure
 * @param[in]     p_meta                   Access metadata for the received message
 * @param[in]     p_in                     Pointer to the input parameters for the user application
 * @param[in]     p_in_transition          Pointer to transition parameters, if present in the incoming message,
 *                                         otherwise set to null.
 * @param[out]    p_out                    Pointer to store the output parameters from the user application.
 *                                         If null, indicates that it is UNACKNOWLEDGED message and no
 *                                         output params are required.
 */
typedef void (*generic_level_state_move_set_cb_t)(const generic_level_server_t * p_self,
                                                  const access_message_rx_meta_t * p_meta,
                                                  const generic_level_move_set_params_t * p_in,
                                                  const model_transition_t * p_in_transition,
                                                  generic_level_status_params_t * p_out);


/**
 * Transaction callbacks for the Level state
 */
typedef struct
{
    generic_level_state_get_cb_t        get_cb;
    generic_level_state_set_cb_t        set_cb;
    generic_level_state_delta_set_cb_t  delta_set_cb;
    generic_level_state_move_set_cb_t   move_set_cb;
} generic_level_server_state_cbs_t;

/**
 * Level server callback list
 */
typedef struct
{
    /** Callbacks for the level state */
    generic_level_server_state_cbs_t level_cbs;
} generic_level_server_callbacks_t;

/**
 * User provided settings and callbacks for the model instance
 */
typedef struct
{
    /** If server should force outgoing messages as segmented messages */
    bool force_segmented;
    /** TransMIC size used by the outgoing server messages. See @ref nrf_mesh_transmic_size_t */
    nrf_mesh_transmic_size_t transmic_size;

    /** Callback list */
    const generic_level_server_callbacks_t * p_callbacks;
} generic_level_server_settings_t;

/**  */
struct __generic_level_server_t
{
    /** Model handle assigned to this instance */
    access_model_handle_t model_handle;
    /** Tid tracker structure */
    tid_tracker_t tid_tracker;

    /** Model settings and callbacks for this instance */
    generic_level_server_settings_t settings;
};

/**
 * Initializes Generic On Off server.
 *
 * @note This function should only be called _once_.
 * @note The server handles the model allocation and adding.
 *
 * @param[in]     p_server             Generic On Off server context pointer.
 * @param[in]     element_index            Element index to add the model
 *
 */
uint32_t generic_level_server_init(generic_level_server_t * p_server, uint8_t element_index);

/**
 * Publishes unsolicited Status message
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of local action.
 *
 * @param[in]     p_server                 Status server context pointer.
 * @param[in]     p_params                 Message parameters
 *
 */
uint32_t generic_level_server_status_publish(generic_level_server_t * p_server, const generic_level_status_params_t * p_params);

/**@} end of GENERIC_LEVEL_SERVER */
#endif /* GENERIC_LEVEL_SERVER_H__ */
