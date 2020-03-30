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

#ifndef LIGHT_CTL_SETUP_SERVER_H__
#define LIGHT_CTL_SETUP_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "light_ctl_common.h"
#include "model_common.h"

#include "light_lightness_setup_server.h"

#include "generic_ponoff_setup_server.h"
#include "generic_level_server.h"

/**
 * @defgroup LIGHT_CTL_SETUP_SERVER Light CTL Setup Server model interface
 * @ingroup LIGHT_CTL_MODELS
 *
 * This model is defined as a top level model in the SDK to enable implementation of the tunable
 * white light.
 *
 * This model implements Light CTL Server, Light CTL Temperature Server, and Generic Level Server
 * models. This model requires an instance of Light Lightness Setup Server to be initialized
 * seperately and provided as an input during initialization.
 *
 * @{
 */

/** Server model ID */
#define LIGHT_CTL_SERVER_MODEL_ID (0x1303)

/** Setup server model ID */
#define LIGHT_CTL_SETUP_SERVER_MODEL_ID (0x1304)

/** Temperature server model ID */
#define LIGHT_CTL_TEMPERATURE_SERVER_MODEL_ID (0x1306)

/** Forward declaration for @ref __light_ctl_server_t */
typedef struct __light_ctl_server_t light_ctl_server_t;

/* Forward declaration for @ref __light_ctl_setup_server_t */
typedef struct __light_ctl_setup_server_t light_ctl_setup_server_t;

/* Forward declaration for @ref __light_ctl_temperature_server_t */
typedef struct __light_ctl_temperature_server_t light_ctl_temperature_server_t;

/**
 * Callback type for the Light CTL Set/Set Unacknowledged message.
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
typedef void (*light_ctl_state_set_cb_t)(const light_ctl_setup_server_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         const light_ctl_set_params_t * p_in,
                                         const model_transition_t * p_in_transition,
                                         light_ctl_status_params_t * p_out);

/**
 * Callback type for the Light CTL Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_ctl_state_get_cb_t)(const light_ctl_setup_server_t * p_self,
                                         const access_message_rx_meta_t * p_meta,
                                         light_ctl_status_params_t * p_out);

/**
 * Callback type for the Light CTL Temperature Set/Set Unacknowledged message.
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
typedef void (*light_ctl_state_temperature32_set_cb_t)(const light_ctl_setup_server_t * p_self,
                                                       const access_message_rx_meta_t * p_meta,
                                                       const light_ctl_temperature_set_params_t * p_in,
                                                       const model_transition_t * p_in_transition,
                                                       light_ctl_temperature_status_params_t * p_out);

/**
 * Callback type for Light CTL Temperature Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_ctl_state_temperature32_get_cb_t)(const light_ctl_setup_server_t * p_self,
                                                       const access_message_rx_meta_t * p_meta,
                                                       light_ctl_temperature_status_params_t * p_out);

/**
 * Callback type for the Light CTL Temperature Range Set/Set Unacknowledged message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[in]     p_in              Pointer to the input parameters for the user application.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 *                                  If null, indicates that it is UNACKNOWLEDGED message and no
 *                                  output params are required.
 */
typedef void (*light_ctl_state_temperature32_range_set_cb_t)(const light_ctl_setup_server_t * p_self,
                                                             const access_message_rx_meta_t * p_meta,
                                                             const light_ctl_temperature_range_set_params_t * p_in,
                                                             light_ctl_temperature_range_status_params_t * p_out);

/**
 * Callback type for the Light CTL Temperature Range Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_ctl_state_temperature32_range_get_cb_t)(const light_ctl_setup_server_t * p_self,
                                                             const access_message_rx_meta_t * p_meta,
                                                             light_ctl_temperature_range_status_params_t * p_out);

/**
 * Callback type for the Light CTL Default Set/Set Unacknowledged message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[in]     p_in              Pointer to the input parameters for the user application.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 *                                  If null, indicates that it is UNACKNOWLEDGED message and no
 *                                  output params are required.
 */
typedef void (*light_ctl_state_default_set_cb_t)(const light_ctl_setup_server_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 const light_ctl_default_set_params_t * p_in,
                                                 light_ctl_default_status_params_t * p_out);

/**
 * Callback type for the Light CTL Default Get message.
 *
 * @param[in]     p_self            Pointer to the model structure.
 * @param[in]     p_meta            Access metadata for the received message.
 * @param[out]    p_out             Pointer to store the output parameters from the user application.
 */
typedef void (*light_ctl_state_default_get_cb_t)(const light_ctl_setup_server_t * p_self,
                                                 const access_message_rx_meta_t * p_meta,
                                                 light_ctl_default_status_params_t * p_out);


/**
 * Callback type for the Light CTL move Set/Set Unacknowledged "message".
 *
 * This isn't a real message, but CTL converts a level move set to this to allow the app layer to
 * properly handle level move functionality
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
typedef void (*light_ctl_state_move_set_cb_t)(const light_ctl_setup_server_t * p_self,
                                              const access_message_rx_meta_t * p_meta,
                                              const light_ctl_temperature_move_set_params_t * p_in,
                                              const model_transition_t * p_in_transition,
                                              light_ctl_temperature_status_params_t * p_out);

/**
 * Callback type for the Light CTL delta Set/Set Unacknowledged "message".
 *
 * This isn't a real message, but CTL converts a level delta set to this to allow the app layer to
 * properly handle level delta functionality
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
typedef void (*light_ctl_state_delta_set_cb_t)(const light_ctl_setup_server_t * p_self,
                                               const access_message_rx_meta_t * p_meta,
                                               const light_ctl_temperature_delta_set_params_t * p_in,
                                               const model_transition_t * p_in_transition,
                                               light_ctl_temperature_status_params_t * p_out);

/**
 * Transaction callbacks for the Light CTL states.
 */
typedef struct
{
    light_ctl_state_set_cb_t                       set_cb;
    light_ctl_state_get_cb_t                       get_cb;
    light_ctl_state_temperature32_set_cb_t         temperature32_set_cb;
    light_ctl_state_temperature32_get_cb_t         temperature32_get_cb;
    light_ctl_state_temperature32_range_set_cb_t   temperature32_range_set_cb;
    light_ctl_state_temperature32_range_get_cb_t   temperature32_range_get_cb;
    light_ctl_state_default_set_cb_t               default_set_cb;
    light_ctl_state_default_get_cb_t               default_get_cb;
    light_ctl_state_move_set_cb_t                  move_set_cb;
    light_ctl_state_delta_set_cb_t                 delta_set_cb;
} light_ctl_setup_server_state_cbs_t;

/**
 * User provided settings and callbacks for the Light CTL server model instance.
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
} light_ctl_server_settings_t;

/**  */
struct __light_ctl_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Tid tracker structure. */
    tid_tracker_t tid_tracker;

    /** Settings and callbacks for this instance. */
    light_ctl_server_settings_t settings;
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
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message publication.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to
 *                                  the publish address (if force segmentation is enabled). Wait
 *                                  for the transmission to finish.
 */
uint32_t light_ctl_server_status_publish(const light_ctl_server_t * p_server,
                                         const light_ctl_status_params_t * p_params);

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
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message publication.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to
 *                                  the publish address (if force segmentation is enabled). Wait
 *                                  for the transmission to finish.
 */
uint32_t light_ctl_server_temperature_status_publish(const light_ctl_temperature_server_t * p_server,
                                                     const light_ctl_temperature_status_params_t * p_params);

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
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message publication.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to
 *                                  the publish address (if force segmentation is enabled). Wait
 *                                  for the transmission to finish.
 */
uint32_t light_ctl_server_temperature_range_status_publish(const light_ctl_server_t * p_server,
                                                           const light_ctl_temperature_range_status_params_t * p_params);

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
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message publication.
 * @retval NRF_ERROR_NOT_FOUND      Invalid model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to
 *                                  the publish address (if force segmentation is enabled). Wait
 *                                  for the transmission to finish.
 */
uint32_t light_ctl_server_default_status_publish(const light_ctl_server_t * p_server,
                                                 const light_ctl_default_status_params_t * p_params);

/**
 * User provided settings and callbacks for the CTL temperature model instance.
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
} light_ctl_temperature_server_settings_t;

/**  */
struct __light_ctl_temperature_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Parent model context for - Level server. */
    generic_level_server_t generic_level_srv;

    /** Tid tracker structure. */
    tid_tracker_t tid_tracker;

    /** Settings and callbacks for this instance. */
    light_ctl_temperature_server_settings_t settings;
};

/**
 * Light CTL server callback list.
 */
typedef struct
{
    /** Transaction callbacks for the Light CTL states. */
    light_ctl_setup_server_state_cbs_t light_ctl_cbs;
} light_ctl_setup_server_callbacks_t;

/**
 * User provided settings and callbacks for the Light CTL Setup Server model instance.
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
    const light_ctl_setup_server_callbacks_t * p_callbacks;
} light_ctl_setup_server_settings_t;

/** Internal structure to hold state information. */
typedef struct
{
    /** Flash handle - only used by flash system. */
    uint8_t handle;
    /** To keep track if values has been loaded from flash or not. */
    bool initialized;
} light_ctl_state_t;


/**  */
struct __light_ctl_setup_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Parent model context for - CTL server. */
    light_ctl_server_t ctl_srv;

    /** Parent model context for - CTL Temperature server. */
    light_ctl_temperature_server_t ctl_temperature_srv;

    /** Model settings and callbacks for this instance. */
    light_ctl_setup_server_settings_t settings;

    /** State for this instance. */
    light_ctl_state_t state;
};

/**
 * Initializes the Light CTL Setup server.
 *
 * @note The server handles the model allocation and adding.
 *
 * @param[in]     p_s_server          [Light CTL server](@ref __light_ctl_setup_server_t) context pointer.
 * @param[in]     p_ll_s_server       [Light Lightness](@ref __light_lightness_setup_server_t)
 *                                    context pointer.
 * @param[in]     element_index       Element index to add the ctl and ctl setup server models to.
 *
 * The light lightness context pointer is used to obtain the model handles for all of the light
 * lightness and extended models for subscription list sharing.
 *
 * @note This model spans two elements and the API will initialize use secondary element (Temperature
 * element) as next higher element. This helps the third party provisioners to easily identify the
 * Light CTL Setup Server model in its entirety.
 *
 * @retval NRF_SUCCESS                If the model is initialized successfully.
 * @retval NRF_ERROR_NULL             If NULL pointer is provided as input context.
 * @retval NRF_ERROR_NO_MEM           @ref ACCESS_MODEL_COUNT number of models already allocated
 *                                    or no more subscription lists available in memory pool
 *                                    (see @ref ACCESS_SUBSCRIPTION_LIST_COUNT) or
 *                                    no more CTL Setup Server instances can be initialized
 *                                    (see @ref LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX).
 * @retval NRF_ERROR_FORBIDDEN        Multiple model instances per element are not allowed
 *                                    or changes to device composition are not allowed.
 *                                    Adding a new model after device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND        Invalid access element index.
 * @retval NRF_ERROR_INVALID_STATE    If module is unable to initialize the storage for CTL states.
 */
uint32_t light_ctl_setup_server_init(light_ctl_setup_server_t * p_s_server,
                                     light_lightness_setup_server_t * p_ll_s_server,
                                     uint8_t element_index);

/**
 * Function to do the OnPowerup binding
 *
 * The caller is responsible for reading the saved state values out of flash and
 * pass those so this function can determine what the current lightness should be set to.
 *
 * @param[in]     p_s_server        [Light CTL server](@ref __light_ctl_setup_server_t) context pointer.
 * @param[in]     p_saved_values    Pointer to the [structure containing the
 *                                  restored flash value](@ref light_ctl_saved_values_t) of the CTL states.
 * @retval NRF_SUCCESS              If binding operation is performed successfully.
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context.
 * @retval NRF_ERROR_INVALID_DATA   If OnPowerUp state value (see @ref generic_on_powerup_values_t)
 *                                  is not within range.
 */
uint32_t light_ctl_ponoff_binding_setup(light_ctl_setup_server_t * p_s_server,
                                        light_ctl_saved_values_t * p_saved_values);

/**@} end of LIGHT_CTL_SETUP_SERVER  */
#endif /* LIGHT_CTL_SETUP_SERVER_H__ */
