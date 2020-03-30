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

#ifndef LIGHT_LIGHTNESS_SERVER_H__
#define LIGHT_LIGHTNESS_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "light_lightness_common.h"
#include "model_common.h"

#include "generic_ponoff_setup_server.h"
#include "generic_level_server.h"

/**
 * @defgroup LIGHT_LIGHTNESS_SETUP_SERVER Light Lightness Setup Server model interface
 * @ingroup LIGHT_LIGHTNESS_MODELS
 *
 * This model extends Light Lightness server, Generic Level server,
 * Generic PowerOnOff Setup server, Generic PowerOnOff server, Generic
 * OnOff server, and Generic Default Transition Time
 * server. Therefore, this model generates events for messages
 * received by its parent model.
 *
 * @{
 */

/** Server model ID */
#define LIGHT_LIGHTNESS_SERVER_MODEL_ID 0x1300

/** Setup server model ID */
#define LIGHT_LIGHTNESS_SETUP_SERVER_MODEL_ID 0x1301

/* Forward declaration */
typedef struct __light_lightness_server_t light_lightness_server_t;

/* Forward declaration */
typedef struct __light_lightness_setup_server_t light_lightness_setup_server_t;

/**
 * Callback type for Light Lightness Set/Set Unacknowledged message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[in]     p_in              Pointer to the input parameters for the user application.
 * @param[in]     p_in_transition   Pointer to transition parameters, if present in the incoming message,
 *                                  otherwise set to null.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 *                                  If null, indicates that it is UNACKNOWLEDGED message and no
 *                                  output params are required.
 */
typedef void (*light_lightness_state_set_cb_t)(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const light_lightness_set_params_t * p_in,
                                               const model_transition_t * p_in_transition,
                                               light_lightness_status_params_t * p_out);

/**
 * Callback type for Light Lightness Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_lightness_state_get_cb_t)(const light_lightness_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               light_lightness_status_params_t * p_out);

/**
 * Callback type for Light Lightness Last Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_lightness_state_last_get_cb_t)(const light_lightness_setup_server_t * p_self,
                                                    const access_message_rx_meta_t * p_meta,
                                                    light_lightness_last_status_params_t * p_out);

/**
 * Callback type for Light Lightness Default Set/Set Unacknowledged message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[in]     p_in              Pointer to the input parameters for the user application.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 *                                  If null, indicates that it is UNACKNOWLEDGED message and no
 *                                  output params are required.
 */
typedef void (*light_lightness_state_default_set_cb_t)(const light_lightness_setup_server_t * p_self,
                                                       const access_message_rx_meta_t * p_meta,
                                                       const light_lightness_default_set_params_t * p_in,
                                                       light_lightness_default_status_params_t * p_out);

/**
 * Callback type for Light Lightness Default Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_lightness_state_default_get_cb_t)(const light_lightness_setup_server_t * p_self,
                                                       const access_message_rx_meta_t * p_meta,
                                                       light_lightness_default_status_params_t * p_out);

/**
 * Callback type for Light Lightness Range Set/Set Unacknowledged message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[in]     p_in              Pointer to the input parameters for the user application.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 *                                  If null, indicates that it is UNACKNOWLEDGED message and no
 *                                  output params are required.
 */
typedef void (*light_lightness_state_range_set_cb_t)(const light_lightness_setup_server_t * p_self,
                                                     const access_message_rx_meta_t * p_meta,
                                                     const light_lightness_range_set_params_t * p_in,
                                                     light_lightness_range_status_params_t  * p_out);

/**
 * Callback type for Light Lightness Range Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_lightness_state_range_get_cb_t)(const light_lightness_setup_server_t * p_self,
                                                     const access_message_rx_meta_t * p_meta,
                                                     light_lightness_range_status_params_t  * p_out);

/**
 * Callback type for Light Lightness delta Set/Set Unacknowledged "message".
 *
 * This isn't a real message, but light lightness converts a level delta set to this
 * to allow the app layer to properly handle level delta functionality
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[in]     p_in              Pointer to the input parameters for the user application.
 * @param[in]     p_in_transition   Pointer to transition parameters, if present in the incoming message,
 *                                  otherwise set to null.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 *                                  If null, indicates that it is UNACKNOWLEDGED message and no
 *                                  output params are required.
 */
typedef void (*light_lightness_state_delta_set_cb_t)(const light_lightness_setup_server_t * p_self,
                                                     const access_message_rx_meta_t * p_meta,
                                                     const light_lightness_delta_set_params_t * p_in,
                                                     const model_transition_t * p_in_transition,
                                                     light_lightness_status_params_t * p_out);


/**
 * Callback type for Light Lightness move Set/Set Unacknowledged "message".
 *
 * This isn't a real message, but light lightness converts a level move set to this
 * to allow the app layer to properly handle level move functionality
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[in]     p_in              Pointer to the input parameters for the user application.
 * @param[in]     p_in_transition   Pointer to transition parameters, if present in the incoming message,
 *                                  otherwise set to null.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 *                                  If null, indicates that it is UNACKNOWLEDGED message and no
 *                                  output params are required.
 */
typedef void (*light_lightness_state_move_set_cb_t)(const light_lightness_setup_server_t * p_self,
                                                    const access_message_rx_meta_t * p_meta,
                                                    const light_lightness_move_set_params_t * p_in,
                                                    const model_transition_t * p_in_transition,
                                                    light_lightness_status_params_t * p_out);

/**
 * Transaction callbacks for the Light Lightness states.
 */
typedef struct
{
    light_lightness_state_get_cb_t          get_cb;
    light_lightness_state_set_cb_t          set_cb;
    light_lightness_state_last_get_cb_t     last_get_cb;
    light_lightness_state_default_get_cb_t  default_get_cb;
    light_lightness_state_default_set_cb_t  default_set_cb;
    light_lightness_state_range_get_cb_t    range_get_cb;
    light_lightness_state_range_set_cb_t    range_set_cb;
    light_lightness_state_delta_set_cb_t    delta_set_cb;
    light_lightness_state_move_set_cb_t     move_set_cb;
} light_lightness_setup_server_state_cbs_t;

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

    /* There are no callbacks for the state for this model, these
     * callbacks are defined for the setup server. */
} light_lightness_server_settings_t;

/**  */
struct __light_lightness_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Tid tracker structure. */
    tid_tracker_t tid_tracker;

    /** Parent model context - Generic Level server, user must provide
     * a state callback. */
    generic_level_server_t generic_level_srv;

    /** Settings and callbacks for this instance. */
    light_lightness_server_settings_t settings;
};

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated
 * state value as a result of local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lightness_server_status_publish(const light_lightness_server_t * p_server,
                                               const light_lightness_status_params_t * p_params);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated
 * state value as a result of local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lightness_server_linear_status_publish(light_lightness_server_t * p_server,
                                                      const light_lightness_linear_status_params_t * p_params);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated
 * state value as a result of local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect message parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lightness_server_last_status_publish(light_lightness_server_t * p_server,
                                                    const light_lightness_last_status_params_t * p_params);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated
 * state value as a result of local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lightness_server_default_status_publish(const light_lightness_server_t * p_server,
                                                       const light_lightness_default_status_params_t * p_params);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated
 * state value as a result of local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  Incorrect message parameters,
 *                                  the model not bound to application key,
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t light_lightness_server_range_status_publish(const light_lightness_server_t * p_server,
                                                     const light_lightness_range_status_params_t * p_params);

/**
 * Light Lightness server callback list.
 */
typedef struct
{
    /** Transaction callbacks for the Light Lightness states. */
    light_lightness_setup_server_state_cbs_t light_lightness_cbs;
} light_lightness_setup_server_callbacks_t;

/**
 * User provided settings and callbacks for the model instance.
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

    /** Callback list. */
    const light_lightness_setup_server_callbacks_t * p_callbacks;
} light_lightness_setup_server_settings_t;

typedef struct
{
    uint8_t handle;
    bool initialized;
} light_lightness_state_t;

/**  */
struct __light_lightness_setup_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Parent model context for - Light Lightness server. */
    light_lightness_server_t light_lightness_srv;

    /** Parent model context for - Generic Power OnOff Setup server. */
    generic_ponoff_setup_server_t generic_ponoff_setup_srv;

    /** Model settings and callbacks for this instance. */
    light_lightness_setup_server_settings_t settings;

    /** State for this instance. */
    light_lightness_state_t state;
};

/**
 * Initializes Light Lightness Setup server.
 *
 * @note The server handles the model allocation and adding.
 *
 * @param[in]     p_server          Light Lightness server context pointer.
 * @param[in]     element_index     Element index to add the model to.
 *
 * @retval NRF_SUCCESS                  The model is initialized successfully.
 * @retval NRF_ERROR_NULL               NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM             @ref ACCESS_MODEL_COUNT number of models already allocated
 *                                      or no more subscription lists available in memory pool
 *                                      (see @ref ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN          Multiple model instances per element are not allowed
 *                                      or changes to device composition are not allowed.
 *                                      Adding a new model after device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND          Invalid access element index.
 * @retval NRF_ERROR_RESOURCES          No more Light Lightness Setup server instance can be allocated.
 *                                      Increase @ref LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX.
 */
uint32_t light_lightness_setup_server_init(light_lightness_setup_server_t * p_server,
                                           uint8_t element_index);
/**
 * Function to do the OnPowerup binding
 *
 * This is called when the mesh is initialized and stable.  The caller
 * is responsible for reading the saved state values out of flash and
 * pass those so this function can determine what the current
 * lightness should be set to.
 *
 * The following rules are applied when restoring the lightness level:
 *
 * If [onpowerup](@ref light_lightness_saved_values_t::onpowerup) in the @p p_saved_values
 * is equal to @ref GENERIC_ON_POWERUP_OFF, the lightness level is set to 0.
 *
 * If [onpowerup](@ref light_lightness_saved_values_t::onpowerup) in the @p p_saved_values
 * is equal to @ref GENERIC_ON_POWERUP_DEFAULT and
 * [default_lightness](@ref light_lightness_saved_values_t::default_lightness)
 * in @p p_saved_values is **not** zero, the lightness level is set to
 * [default_lightness](@ref light_lightness_saved_values_t::default_lightness)
 * of the @p p_saved_values.
 *
 * If [onpowerup](@ref light_lightness_saved_values_t::onpowerup) in the @p p_saved_values
 * is equal to @ref GENERIC_ON_POWERUP_DEFAULT and
 * [default_lightness](@ref light_lightness_saved_values_t::default_lightness)
 * in @p p_saved_values is zero, the lightness level is set to
 * [last_lightness](@ref light_lightness_saved_values_t::last_lightness)
 * of the @p p_saved_values.
 *
 * If [onpowerup](@ref light_lightness_saved_values_t::onpowerup) in the @p p_saved_values
 * is equal to @ref GENERIC_ON_POWERUP_RESTORE the lightness level is set to
 * [actual_lightness](@ref light_lightness_saved_values_t::actual_lightness)
 * of the @p p_saved_values.
 *
 * The [range](@ref light_lightness_saved_values_t::range) value in the @p p_saved_values
 * is used to restrict the lightness level.
 *
 * @param[in]     p_s_server        Status server context pointer.
 * @param[in]     p_saved_values    Pointer to the structure containing the
 *                                  restored flash value of the lightness states.
 *
 * @retval NRF_SUCCESS      The model is initialized successfully.
 * @retval NRF_ERROR_NULL   NULL pointer given to function.
 */
uint32_t light_lightness_ponoff_binding_setup(light_lightness_setup_server_t * p_s_server,
                                              light_lightness_saved_values_t * p_saved_values);

/**@} end of LIGHT_LIGHTNESS_SETUP_SERVER */
#endif /* LIGHT_LIGHTNESS_SERVER_H__ */
