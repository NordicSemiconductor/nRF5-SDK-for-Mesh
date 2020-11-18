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

#ifndef APP_LIGHT_CTL_H__
#define APP_LIGHT_CTL_H__

#include <stdint.h>

#include "light_ctl_setup_server.h"
#include "light_lightness_setup_server.h"
#include "app_timer.h"
#include "app_transition.h"
#include "app_light_lightness.h"
#include "list.h"
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#include "app_scene.h"
#endif

/**
 * @defgroup APP_LIGHT_CTL Light CTL Setup Server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application Light CTL Setup Server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Light CTL Setup Server model.
 *
 * The CTL server requires a composite state for the hardware.  This composite state has the
 * lightness, temperature, and delta UV values. There are two independent callbacks that pass
 * this information to the application, whenever these values need to be updated. One callback
 * is provided to update the Delta UV (DUV) and Temperature value and another callback is provided
 * to update the Lightness value. The two callbacks are provided as these states can change the
 * values independently.
 *
 * The application should use the set/transition callbacks provided by this module to set the
 * hardware state. The hardware state could be changed by reflecting the value provided by the
 * set/transition callbacks on GPIOs or by sending this value to the connected lighting
 * peripheral using some other interface (e.g. serial interface). Similarly, the application should
 * use the get callback provided by this module to read the hardware state.
 *
 * This module triggers the set/transition callback only when it determines that it is time to
 * inform the user application. It is possible that the client can send multiple overlapping set
 * commands. In such case any transition in progress will be abandoned and fresh transition will be
 * started if required.
 *
 * Using transition_cb:
 * If the underlaying hardware does not support setting of the instantaneous values provided via
 * `set_cb`, the `transition_cb` can be used to implement the transition effect according to
 * provided transition parameters. This callback will be called when transition start with the
 * required transition time and target value. When the transition is complete this callback will be
 * called again with transition time set to 0 and the desired target value.
 * <br>
 * @warning To comply with the @tagMeshMdlSp test cases, the application must adhere to the
 * requirements defined in the following sections:
 * - @tagMeshMdlSp section 6.1.3 (Ligth CTL state), section 6.3.2 (Light CTL Messages), and section 6.4.3-6.4.5.
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/** Light CTL hardware state format for transfer of temperature and delta UV between the mid app and top
 * app (main.c)
 */
typedef struct
{
    /** Temperature32 value */
    uint32_t temperature32;
    /** Delta UV value */
    uint16_t delta_uv;
} app_light_ctl_temperature_duv_hw_state_t;

/**
 * Macro to create application level app_light_ctl_setup_server_t context.
 *
 * Individual timer instances are created for each model instance.
 * An app light lightness structure needs to be created separately for each model instance.
 *
 * @param[in] _name                     Name of the [app_light_ctl_setup_server_t](@ref __app_light_ctl_setup_server_t) instance.
 * @param[in] _force_segmented          If the Light CTL Setup Server shall use force segmentation of messages.
 * @param[in] _mic_size                 MIC size to be used by the Light CTL Setup Server.
 * @param[in] _light_ctl_set_cb         Callback for setting the application state to given value.
 * @param[in] _light_ctl_get_cb         Callback for reading the state from the application.
 * @param[in] _light_ctl_transition_cb  Callback for setting the application transition time and state value to given values.
 */
#define APP_LIGHT_CTL_SETUP_SERVER_DEF(_name, _force_segmented, _mic_size, _light_ctl_set_cb, _light_ctl_get_cb, _light_ctl_transition_cb) \
    APP_TIMER_DEF(_name ## _timer);                                           \
    static app_light_ctl_setup_server_t _name =                               \
    {                                                                         \
        .light_ctl_setup_srv.settings.force_segmented = _force_segmented,     \
        .light_ctl_setup_srv.settings.transmic_size = _mic_size,              \
        .state.transition.timer.p_timer_id = &_name ## _timer,                \
        .app_light_ctl_set_cb = _light_ctl_set_cb,                            \
        .app_light_ctl_get_cb = _light_ctl_get_cb,                            \
        .app_light_ctl_transition_cb = _light_ctl_transition_cb,              \
    };

/** Internal structure to hold state and timing information.
 *
 * @note The present Light CTL state is available from a callback.
 */
typedef struct
{
    /** Present value of the temperature32 state. */
    uint32_t present_temperature32;
    /** Present value of the delta uv state. */
    int16_t present_delta_uv;

    /** Target value of the temperature32 state, as received from the model interface. */
    uint32_t target_temperature32;
    /** Target value of the delta_uv state, as received from the model interface. */
    int16_t target_delta_uv;

    /** Initial present temperature32 required for handling Set/Delta Set message. */
    uint32_t initial_present_temperature32;
    /** Initial present delta_uv required for handling Set/Delta Set message. */
    int16_t initial_present_delta_uv;

    /** Present temperature32 value when message was received */
    uint32_t init_present_temp32_snapshot;
    /** Requested target temperature32 */
    uint32_t target_temp32_snapshot;
    /** Present DUV value when message was received */
    int16_t init_present_duv_snapshot;
    /** Requested target DUV */
    int16_t target_duv_snapshot;

    /** To detect if TID is new while processing delta transition */
    bool new_tid;

    /* Elapsed time at last publication. */
    uint32_t published_ms;

    /** Structure for using transition module functionality. */
    app_transition_t transition;
} app_light_ctl_state_t;

/* Forward declaration */
typedef struct __app_light_ctl_setup_server_t app_light_ctl_setup_server_t;

/** Application Light CTL (temperature/delta UV) state set callback prototype.
 *
 * This callback is called by this module whenever the application is required to be informed to
 * reflect the desired Light CTL composite state (temperature, delta UV), as a result of the
 * received messages (for Light CTL or extended models), depending on the received target state values and
 * timing parameters.
 *
 * @note Since the behavioral module encapsulates functionality required for the compliance with
 * timing behaviour, it is not possible to infer number of Set messages received by the node by
 * counting the number of times this callback is triggered.
 *
 * @param[in]   p_app               Pointer to [app_light_ctl_setup_server_t](@ref __app_light_ctl_setup_server_t) context.
 * @param[in]   p_ctl_state         Pointer to the composite Light CTL state to set. See
 *                                  @ref app_light_ctl_temperature_duv_hw_state_t and @ref model_callback_pointer_note.
 */
typedef void (*app_light_ctl_set_cb_t)(const app_light_ctl_setup_server_t * p_app, app_light_ctl_temperature_duv_hw_state_t * p_ctl_state);


/** Application Light CTL state read callback prototype.
 *
 * This callback is called by the app_model_behaviour.c whenever application Light CTL state is required
 * to be read.
 *
 * @param[in]  p_app                Pointer to [app_light_ctl_setup_server_t](@ref __app_light_ctl_setup_server_t) context.
 * @param[out] p_present_ctl_state  User application fills this value with the values retrieved
 *                                  from the hardware interface. See
 *                                  @ref app_light_ctl_temperature_duv_hw_state_t and @ref model_callback_pointer_note.
 */
typedef void (*app_light_ctl_get_cb_t)(const app_light_ctl_setup_server_t * p_app, app_light_ctl_temperature_duv_hw_state_t * p_present_ctl_state);

/** Application Light CTL transition time callback prototype.
 *
 * This callback is called by the this module whenever application is required to be informed to
 * reflect the desired transition time, as a result of the received messages (for light ctl or
 * encapsulated models), depending on the received target ctl state and timing parameters.
 *
 * @param[in] p_app                 Pointer to [app_light_ctl_setup_server_t](@ref __app_light_ctl_setup_server_t) context.
 * @param[in] transition_time_ms    Transition time (in milliseconds) to be used by the application.
 * @param[in] target_ctl_state      Target Light CTL state to be used by the application. See
 *                                  @ref app_light_ctl_temperature_duv_hw_state_t.
 */
typedef void (*app_light_ctl_transition_cb_t)(const app_light_ctl_setup_server_t * p_app,
                                                   uint32_t transition_time_ms,
                                                   app_light_ctl_temperature_duv_hw_state_t target_ctl_state);

/** Application level structure holding the Light CTL Setup Server model context, Light Lightness
 * application structure (which contains the light_lightness_setup_server model context), and the Light CTL
 * and Light Lightness state representation. */
struct __app_light_ctl_setup_server_t
{
    /** CTL setup server context. */
    light_ctl_setup_server_t light_ctl_setup_srv;

    /** app server for light lightness. */
    app_light_lightness_setup_server_t * p_app_ll;

    /** Set the device temp/duv values. */
    app_light_ctl_set_cb_t app_light_ctl_set_cb;

    /** Get the device temp/duv value .*/
    app_light_ctl_get_cb_t app_light_ctl_get_cb;

    /** The device Light CTL transition time */
    app_light_ctl_transition_cb_t app_light_ctl_transition_cb;

    /** Internal variable. Representation of the Light CTL state related data and transition parameters
     *  required for behavioral implementation, and for communicating with the application. */
    app_light_ctl_state_t state;
    /** Internal variable. To keep track if CTL state set is active. */
    bool ctl_state_set_active;
    /** Internal variable. To keep track if Temperature state set is active. */
    bool ctl_temperature_state_set_active;
    /** Internal variable. Used for scheduling transition abort. */
    bool abort_move;
    /** Internal variable. */
    list_node_t node;
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
    /** Internal variable. Scene callback interface. 
     * @note Available only if  @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1. */
    app_scene_model_interface_t scene_if;
    /** Internal variable. Pointer to app_scene context. 
     * @note Available only if  @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1. */
    app_scene_setup_server_t  * p_app_scene;
#endif
};

/** Initializes the behavioral module for the Light CTL and Light Lightness models.
 *
 * @param[in] p_app                 Pointer to [app_light_ctl_setup_server_t](@ref __app_light_ctl_setup_server_t) context.
 * @param[in] element_index         Element index on which the Light CTL Setup Server will be instantiated.
 *                                  This needs to be the same element index as the one Light
 *                                  Lightness was instantiated on.
 * @param[in] p_app_ll              Pointer to [app_light_lightness_setup_server_t](@ref __app_light_lightness_setup_server_t) context.
 *
 * @retval NRF_SUCCESS              If initialization is successful.
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated
 *                                  or no more subscription lists available in memory pool
 *                                  (see @ref ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed
 *                                  or changes to device composition are not allowed.
 *                                  Adding a new model after device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index.
 * @retval NRF_ERROR_INVALID_STATE  If module is unable to initialize the storage for Light CTL states.
 * @retval NRF_ERROR_RESOURCES      No more instances can be created.
 *                                  In that case, increase value of
 *                                  @ref LIGHT_CTL_SETUP_SERVER_INSTANCES_MAX.
 */
uint32_t app_light_ctl_model_init(app_light_ctl_setup_server_t * p_app, uint8_t element_index,
                                  app_light_lightness_setup_server_t * p_app_ll);

/** Informs the model that the system is ready to have the powerup onoff bindings.
 *
 * @note This function must be called from the same IRQ level that is specified
 * for the mesh stack (see @ref mesh_stack_init() API).
 *
 * @param[in] p_app                 Pointer to [app_light_ctl_setup_server_t](@ref __app_light_ctl_setup_server_t) context.
 *
 * @retval NRF_SUCCESS              Bindings are setup successfully.
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context.
 * @retval NRF_ERROR_INVALID_DATA   If OnPowerUp state value (see @ref generic_on_powerup_values_t)
 *                                  is not within range.
 */
uint32_t app_light_ctl_binding_setup(app_light_ctl_setup_server_t * p_app);

/** Initiates value fetch from the user application by calling a get callback, updates internal state,
 * and publishes the Light CTL Status messages.
 *
 * This API must always be called by an application when user initiated action (e.g. button press)
 * results in the local Level state change. This API should never be called from transition callback.
 * @tagMeshSp mandates that, every
 * local state change must be published if model publication state is configured. If model
 * publication is not configured this API call will not generate any assertion.
 *
 * @param[in] p_app                 Pointer to [app_light_ctl_setup_server_t](@ref __app_light_ctl_setup_server_t) context.
 *
 * @retval NRF_SUCCESS              If status message is successfully published.
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
uint32_t app_light_ctl_current_value_publish(app_light_ctl_setup_server_t * p_app);

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
/** Sets the scene context
 *
 * This is needed for app light ctl to inform app scene when the state change occurs.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in] p_app                 Pointer to [app_light_ctl_setup_server_t](@ref
 *                                  __app_light_ctl_setup_server_t) context.
 * @param[in] p_app_scene           Pointer to scene behavioral moduel context.
 *
 * @retval NRF_SUCCESS              Value is restored successfully
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context
 */
uint32_t app_light_ctl_scene_context_set(app_light_ctl_setup_server_t * p_app, 
                                         app_scene_setup_server_t  * p_app_scene);
#endif

/** @} end of APP_LIGHT_CTL */
#endif /* APP_LIGHT_CTL_H__*/
