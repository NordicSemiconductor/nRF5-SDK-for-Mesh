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

#ifndef LIGHT_LC_SETUP_SERVER_H__
#define LIGHT_LC_SETUP_SERVER_H__

#include <stdint.h>
#include "access.h"
#include "model_common.h"
#include "fsm.h"

#include "generic_onoff_server.h"
#include "light_lc_common.h"
#include "timer_scheduler.h"

/**
 * @defgroup LIGHT_LC_SETUP_SERVER Light LC (Lightness Control) Setup Server model interface
 * @ingroup LIGHT_LC_MODELS
 *
 * This model extends Light LC server, Light Lightness server, Generic Level server,
 * Generic PowerOnOff Setup server, Generic PowerOnOff server, Generic
 * OnOff server, and Generic Default Transition Time
 * server. Therefore, this model generates events for messages
 * received by its parent model.
 *
 * @{
 */

/** Server model ID */
#define LIGHT_LC_SERVER_MODEL_ID 0x130F

/** Setup server model ID */
#define LIGHT_LC_SETUP_SERVER_MODEL_ID 0x1310

/* Forward declaration */
typedef struct __light_lc_server_t light_lc_server_t;

/* Forward declaration */
typedef struct __light_lc_setup_server_t light_lc_setup_server_t;

/**
 * Callback type for storing data to the persistent memory after Set messages affecting the
 * persistent states are received.
 *
 * Mid app will call the appropriate function to set flash memory.
 *
 * @param[in]     p_s_server  Pointer to the model structure.
 * @param[in]     lc_state    State to be set into persistent memory
 * @param[in]     set_value   Value to set into persistent memory
 */
typedef void (*light_lc_persist_state_set_cb_t)(const light_lc_setup_server_t * p_s_server,
                                                const light_lc_state_t lc_state,
                                                const void *p_set_value);

/**
 * Callback type for getting data from the persistent memory after Get messages are received.
 *
 * Mid app will call the appropriate function to get flash memory
 *
 * @param[in]     p_s_server   Pointer to the model structure.
 * @param[in]     lc_state     State to be set into persistent memory.
 * @param[out]    p_get_value  Return value from persistent memory
 */
typedef void (*light_lc_persist_state_get_cb_t)(const light_lc_setup_server_t * p_s_server,
                                                const light_lc_state_t lc_state,
                                                void * p_get_value);

/**
 * Callback type for setting the actual lightness.
 *
 * Mid app will call the top app (main.c) to set the actual lightness.
 *
 * @param[in]     p_s_server        Pointer to the model structure.
 * @param[in]     actual_lightness  Lightness level to set
 */
typedef void (*light_lc_actual_set_cb_t)(const light_lc_setup_server_t * p_s_server,
                                         uint16_t actual_lightness);

/**
 * Callback type for getting the actual lightness.
 *
 * Mid app will call the top app (main.c) to get the actual lightness.
 *
 * @param[in]     p_s_server          Pointer to the model structure.
 * @param[in]     p_actual_lightness  Lightness level to get.
 */
typedef void (*light_lc_actual_get_cb_t)(const light_lc_setup_server_t * p_s_server,
                                         uint16_t * p_actual_lightness);

/**
 * Transaction callbacks for the Light LC states.
 */
typedef struct
{
    light_lc_persist_state_set_cb_t        light_lc_persist_set_cb;
    light_lc_persist_state_get_cb_t        light_lc_persist_get_cb;
    light_lc_actual_set_cb_t               light_lc_actual_set_cb;
    light_lc_actual_get_cb_t               light_lc_actual_get_cb;
} light_lc_setup_server_state_cbs_t;

/**
 * User provided settings and callbacks for the non-setup server model instance.
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

    /** There are no callbacks for the state for this model, these callbacks are defined for the
     * setup server. */
} light_lc_server_settings_t;

/**  */
struct __light_lc_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Tid tracker structure. */
    tid_tracker_t tid_tracker;

    /** Parent model context for - Generic OnOff server */
    generic_onoff_server_t generic_onoff_srv;

    /** Settings and callbacks for this instance. */
    light_lc_server_settings_t settings;
};


/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of
 * local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           If NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_INVALID_LENGTH Invalid data length(s) supplied.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval NRF_ERROR_INVALID_PARAM  Invalid parameter(s) supplied.
 * @retval NRF_ERROR_INVALID_ADDR   Invalid pointer supplied.
 */
uint32_t light_lc_server_mode_status_publish(const light_lc_server_t * p_server,
                                             const light_lc_mode_status_params_t * p_params);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of
 * local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           If NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_INVALID_LENGTH Invalid data length(s) supplied.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval NRF_ERROR_INVALID_PARAM  Invalid parameter(s) supplied.
 * @retval NRF_ERROR_INVALID_ADDR   Invalid pointer supplied.
 */
uint32_t light_lc_server_occ_mode_status_publish(light_lc_server_t * p_server,
                                                 const light_lc_occupancy_mode_status_params_t * p_params);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of
 * local action.
 *
 * @param[in]     p_server          Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           If NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_INVALID_LENGTH Invalid data length(s) supplied.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval NRF_ERROR_INVALID_PARAM  Invalid parameter(s) supplied.
 * @retval NRF_ERROR_INVALID_ADDR   Invalid pointer supplied.
 */
uint32_t light_lc_server_light_onoff_status_publish(light_lc_server_t * p_server,
                                                    const light_lc_light_onoff_status_params_t * p_params);

/**
 * Light LC setup server callback list.
 */
typedef struct
{
    /** Transaction callbacks for the Light LC states. */
    light_lc_setup_server_state_cbs_t light_lc_cbs;
} light_lc_setup_server_callbacks_t;

/** Internal structure to hold state and timing information. */
typedef struct
{
    /* states not stored in flash */
    /** Data Received from ambient light sensor */
    uint32_t ambient_luxlevel;
    /** if true, ble ambient lux level is valid (written since boot time) */
    bool ambient_luxlevel_valid;
    /** lux level output from FSM */
    uint32_t luxlevel_out;
    /** lightness output from FSM */
    uint16_t lightness_out;

    /** flash handle - only used by flash system */
    uint8_t handle;
} light_lc_setup_server_state_t;

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
    const light_lc_setup_server_callbacks_t * p_callbacks;
} light_lc_setup_server_settings_t;

/** The Light OnOff message is the only one that has transition and delay, though we need to keep
 * information about the initial and target lightness level so we can calculate what to set it to in
 * the meantime.
 */
typedef struct
{
    /** Requested transition from incoming message */
    uint32_t requested_transition_time_ms;
    /** Requested delay from incoming message */
    uint32_t requested_delay_ms;
    /** Transition time is provided. */
    bool transition_time_is_provided;
    /** Requested light_onoff from incoming message */
    bool requested_light_onoff;

    /** Current transition time for currently active command */
    uint32_t transition_time_ms;
    /** Current light_onoff state for currently active command */
    uint8_t present_light_onoff;
    /** Target light_onoff state for currently active command */
    uint8_t target_light_onoff;

    /* The lightness output is in the transition, and both of these
     * lightness values are linear */
    /** Initial present lightness (linear) */
    uint16_t initial_present_lightness;
    /** Target lightness (linear) */
    uint16_t target_lightness;

    /* The lux level output is in the transition, too */
    /** Initial lux level */
    uint32_t initial_present_luxlevel;
    /** Target lux level */
    uint32_t target_luxlevel;
} light_lc_transition_info_t;

/**  */
struct __light_lc_setup_server_t
{
    /** Model handle assigned to this instance. */
    access_model_handle_t model_handle;

    /** Parent model context for - LC server. */
    light_lc_server_t lc_srv;

    /** LC server state variables (not FSM state) */
    light_lc_setup_server_state_t state;

    /** @internal Transition information for light OnOff and lightness levels */
    light_lc_transition_info_t transition_info;

    /** @internal The timer instance pointer for the OnOff event generation */
    timer_event_t onoff_timer;

    /** @internal The state machine  */
    fsm_t fsm;

    /** @internal The timer instance pointer for the state machine */
    model_timer_t fsm_timer;

    /** @internal The timer instance pointer for the light pi server */
    model_timer_t light_pi_timer;

    /** @internal The timer instance pointer for the sensor occupancy delay */
    model_timer_t sensor_delay_timer;

    /** Model settings and callbacks for this instance. */
    light_lc_setup_server_settings_t settings;
};

/**
 * Initializes Light LC Setup server.
 *
 * @note The server handles the model allocation and adding.
 *
 * @param[in]     p_s_server        Light LC Setup Server context pointer.
 * @param[in]     element_index     Element index to add the model to.
 *
 * @retval NRF_SUCCESS              If the model is initialized successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_INVALID_PARAM  Invalid parameter(s) supplied.
 * @retval NRF_ERROR_INVALID_STATE  Invalid state to perform operation.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_FORBIDDEN      Device has been provisioned and changes to model
 *                                  subscription list are not allowed.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval NRF_ERROR_RESOURCES      No more Light LC Setup server instance can be allocated.
 *                                  Increase @ref LIGHT_LC_SETUP_SERVER_INSTANCES_MAX.
 */
uint32_t light_lc_setup_server_init(light_lc_setup_server_t * p_s_server,
                                    uint8_t element_index);

/**
 * Publishes unsolicited Status message.
 *
 * This API can be used to send unsolicited messages to report updated state value as a result of
 * local action.
 *
 * @param[in]     p_s_server        Status server context pointer.
 * @param[in]     p_params          Message parameters.
 *
 * @retval NRF_SUCCESS              If the message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_INVALID_ADDR   Invalid pointer supplied.
 * @retval NRF_ERROR_INVALID_STATE  Invalid state to perform operation.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 * @retval NRF_ERROR_INVALID_LENGTH Attempted to send message larger than @ref LIGHT_LC_PROPERTY_BUF_SIZE.
 */
uint32_t light_lc_setup_server_property_status_publish(const light_lc_setup_server_t * p_s_server,
                                                       const light_lc_property_status_params_t * p_params);


/**
 * Function to do the OnPowerup binding
 *
 * This is called by main.c when the mesh is initialized and stable.  Note that this function must
 * be called from the same IRQ level that is specified for the mesh stack (see @ref
 * mesh_stack_init() API).
 *
 * @param[in]     p_s_server     Status server context pointer.
 * @param[in]     onpowerup      Saved value of onpowerup (read from flash)
 * @param[out]    p_lc_control   Returns false if the LC server is not controlling the light, or
 *                               true if the LC server is controlling the light.
 *
 * @retval NRF_SUCCESS              If the binding is setup successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_INVALID_PARAM  Invalid parameter(s) supplied.
 * @retval NRF_ERROR_INVALID_STATE  Invalid state to perform operation.
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      Access handle invalid.
 */
uint32_t light_lc_setup_server_ponoff_binding_setup(light_lc_setup_server_t * p_s_server,
                                                    uint8_t onpowerup,
                                                    bool * p_lc_control);

/**@} end of LIGHT_LC_SETUP_SERVER */
#endif /* LIGHT_LC_SETUP_SERVER_H__ */
