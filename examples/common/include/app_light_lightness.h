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

#ifndef APP_LIGHT_LIGHTNESS_H__
#define APP_LIGHT_LIGHTNESS_H__

#include <stdint.h>

#include "light_lightness_setup_server.h"
#include "app_timer.h"
#include "app_transition.h"
#include "list.h"
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#include "app_scene.h"
#endif

/**
 * @defgroup APP_LIGHT_LIGHTNESS Light Lightness Setup Server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application Light Lightness server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Light Lightness server model.
 *
 * The application should use the set/transition callback provided by this module to set the
 * hardware state. The hardware state could be changed by reflecting the value provided by the
 * set/transition callback on the GPIO or by sending this value to the connected lighting
 * peripheral using some other interface (e.g. serial interface). Similarly, the application should
 * use the get callback provided by this module to read the hardware state.
 *
 * This module triggers the set/transition callback only when it determines that it is time to
 * inform the user application. It is possible that the client can send multiple overlapping set
 * commands. In such case any transition in progress will be abandoned and fresh transition will be
 * started if required.
 *
 * Using transition_cb:
 * If the underlaying hardware does not support setting of the instantaneous value provided via
 * `set_cb`, the `transition_cb` can be used to implement the transition effect according to
 * provided transition parameters. This callback will be called when transition start with the
 * required transition time and target value. When the transition is complete this callback will be
 * called again with transition time set to 0 and the desired target value.
 * <br>
 * @warning To comply with the @tagMeshMdlSp test cases, the application must adhere to the
 * requirements defined in the following sections:
 * - @tagMeshMdlSp section 6.1.1 (Light Lightness) and section 6.4.1.2 (Light Lightness state behaviour).
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/**
 * Macro to create application level app_light_lightness_setup_server_t context.
 *
 * Individual timer instances are created for each model instance.
 *
 * @param[in] _name                 Name of the [app_light_lightness_setup_server_t](@ref __app_light_lightness_setup_server_t) instance
 * @param[in] _force_segmented      If the light lightness server shall use force segmentation of messages
 * @param[in] _mic_size             MIC size to be used by Light Lightness server
 * @param[in] _set_cb               Callback for setting the application state to given value.
 * @param[in] _get_cb               Callback for reading the state from the application.
 * @param[in] _transition_cb        Callback for setting the application transition time and state value to given values.
 */
#define APP_LIGHT_LIGHTNESS_SETUP_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb, _get_cb, _transition_cb) \
    APP_TIMER_DEF(_name ## _timer);                                     \
    static app_light_lightness_setup_server_t _name =                   \
    {                                                                   \
        .light_lightness_setup_server.settings.force_segmented = _force_segmented, \
        .light_lightness_setup_server.settings.transmic_size = _mic_size, \
        .app_add_notify.app_add_publish_cb = NULL,                      \
        .app_add_notify.app_notify_set_cb = NULL,                       \
        .state.transition.timer.p_timer_id = &_name ## _timer,          \
        .app_light_lightness_set_cb = _set_cb,                          \
        .app_light_lightness_get_cb = _get_cb,                          \
        .app_light_lightness_transition_cb = _transition_cb             \
    };

/** Internal structure to hold state and timing information.
 *
 * @note The present lightness is available from a callback.
 */
typedef struct
{
    /** Present value of the lightness state */
    uint16_t present_lightness;
    /** Target value of the lightness state, as received from the model interface. */
    uint16_t target_lightness;
    /** Initial present lightness required for handling Set/Delta Set message. */
    uint16_t initial_present_lightness;

    /** Present value when message was received */
    uint16_t init_present_snapshot;
    /** Requested target */
    uint16_t target_snapshot;

    /** To detect if TID is new while processing delta transition */
    bool new_tid;

    /* Elapsed time at last publication. */
    uint32_t published_ms;

    /** Structure for using transition module functionality */
    app_transition_t transition;
} app_light_lightness_state_t;

/* Forward declaration */
typedef struct __app_light_lightness_setup_server_t app_light_lightness_setup_server_t;

/** Application Light Lightness state set callback prototype.
 *
 * This callback is called by the this module whenever application is required to
 * be informed to reflect the desired Level value, as a result of the received messages
 * (for light lightness or encapsulated models), depending on the received target lightness
 * value and timing parameters.
 *
 * Note: Since the behavioral module encapsulates functionality required for the compliance
 * with timing behaviour, it is not possible to infer number of Set messages received by the
 * node by counting the number of times this callback is triggered.
 *
 * @param[in]   p_app             Pointer to [app_light_lightness_setup_server_t](@ref __app_light_lightness_setup_server_t) context.
 * @param[in]   lightness         Lightness value to set
 */
typedef void (*app_light_lightness_set_cb_t)(const app_light_lightness_setup_server_t * p_app,
                                             uint16_t lightness);


/** Application Light Lightness state read callback prototype.
 * This callback is called by the app_model_behaviour.c whenever application light_lightness state is required
 * to be read.
 *
 * @param[in]  p_app                Pointer to [app_light_lightness_setup_server_t](@ref __app_light_lightness_setup_server_t) context.
 * @param[out] p_present_lightness  User application fills this value with the value retrieved from
 *                                  the hardware interface. See @ref model_callback_pointer_note.
 */
typedef void (*app_light_lightness_get_cb_t)(const app_light_lightness_setup_server_t * p_app,
                                             uint16_t * p_present_lightness);

/** Application Light Lightness transition time callback prototype.
 *
 * This callback is called by the this module whenever application is required to be informed to
 * reflect the desired transition time, as a result of the received messages (for light lightness or
 * encapsulated models), depending on the received target lightness value and timing parameters.
 *
 * @param[in] p_app                 Pointer to [app_light_lightness_setup_server_t](@ref __app_light_lightness_setup_server_t) context.
 * @param[in] transition_time_ms    Transition time (in milliseconds) to be used by the application.
 * @param[in] target_lightness      Target Lightness value to be used by the application.
 */
typedef void (*app_light_lightness_transition_cb_t)(const app_light_lightness_setup_server_t * p_app,
                                                         uint32_t transition_time_ms, uint16_t target_lightness);

/** Application publish callback prototype.
 *
 * This callback is called by the light lightness mid app whenever it publishes.  This will inform
 * the mid app that instantiated it that a lightness publish has occurred.  If the mid app needs to
 * publish an additional message, it can do so.
 *
 * Since the light lightness mid app doesn't know who the instantiator is, the p_app is sent as a
 * void pointer.
 *
 * @param[in]   p_app_v       Pointer to the app context stored in the structure.
 * @param[in]   p_pub_data    Pointer to the lightness data structure containing data to publish. See @ref model_callback_pointer_note.
 */
typedef void (*app_additional_publish_cb_t)(const void * p_app_v,
                                            light_lightness_status_params_t * p_pub_data);

/** Application notify callback prototype.
 *
 * This callback is called by the light lightness mid app whenever it sets the lightness.  This will
 * inform the mid app that instantiated it that the lightness has been set.  The mid app can take
 * whatever action is needed.
 *
 * Since the light lightness mid app doesn't know who the instantiator is, the p_app_v is sent as a
 * void pointer.
 *
 * @param[in]   p_app_v       Pointer to the app context stored in the structure.
 * @param[in]   lightness     The lightness value to publish.
 */
typedef void (*app_notify_set_cb_t)(const void * p_app_v, uint16_t lightness);

/** Structure for holding publish and notification callbacks for other states bound to
 * Light Lightness state. */
typedef struct
{
    /** Publish callback to call to before publishing lightness status. */
    app_additional_publish_cb_t app_add_publish_cb;
    /** Context pointer for the publish callback. */
    void * p_app_publish_v;
    /** A notify callback to call after receiving lightness Set messages. */
    app_notify_set_cb_t app_notify_set_cb;
    /** A context pointer for the notify callback. */
    void * p_app_notify_v;
} app_additional_light_lightness_notify_t;

/** Application level structure holding the Light Lightness Setup server model context and Light
 * Lightness state representation */
struct __app_light_lightness_setup_server_t
{
    /** Light lightness setup server context/ */
    light_lightness_setup_server_t light_lightness_setup_server;

    /** Set the device lightness value */
    app_light_lightness_set_cb_t app_light_lightness_set_cb;

    /** Get the device lightness value */
    app_light_lightness_get_cb_t app_light_lightness_get_cb;

    /** The device lightness transition time */
    app_light_lightness_transition_cb_t app_light_lightness_transition_cb;

    /** Additional publishing/notifying requirements structure */
    app_additional_light_lightness_notify_t app_add_notify;

    /** Internal variable. Representation of the Light Lightness state related data and transition
     *  parameters required for behavioral implementation, and for communicating with the
     *  application */
    app_light_lightness_state_t state;
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

/** Initializes the behavioral module for the generic Light Lightness model
 *
 * @param[in] p_app                 Pointer to [app_light_lightness_setup_server_t](@ref
 *                                  __app_light_lightness_setup_server_t) context.
 * @param[in] element_index         Element index on which this server will be instantiated.
 *
 * @retval NRF_SUCCESS              If initialization is successful.
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context
 * @retval NRF_ERROR_RESOURCES      No more instances can be created. In that case, increase value
 *                                  of @ref LIGHT_LIGHTNESS_SETUP_SERVER_INSTANCES_MAX.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated or
 *                                  no more subscription lists available in memory pool (see @ref
 *                                  ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index.
 * @retval NRF_ERROR_INVALID_PARAM  If the application timer module has not been initialized.
 * @retval NRF_ERROR_INVALID_STATE  If the application timer is running.
 */
uint32_t app_light_lightness_model_init(app_light_lightness_setup_server_t * p_app, uint8_t element_index);

/** Informs the model that the system is ready to have the powerup onoff bindings
 *
 * This is called by main.c when the mesh is initialized and stable.
 * Note that this function must be called from the same IRQ level that
 * mesh_init() is set at.
 *
 * @param[in] p_app                  Pointer to [app_light_lightness_setup_server_t](@ref
 *                                   __app_light_lightness_setup_server_t) context.
 *
 * @retval NRF_SUCCESS               Bindings are setup successfully
 * @retval NRF_ERROR_NULL            If NULL pointer is provided as input context
 */
uint32_t app_light_lightness_binding_setup(app_light_lightness_setup_server_t * p_app);

/** Initiates value fetch from the user application by calling a get callback, updates internal
 * state, and publishes the Lightness Actual status message.
 *
 * This API must always be called by an application when user initiated action (e.g. button press)
 * results in the local lightness state change. This API should never be called from transition
 * callback. @tagMeshSp mandates that, every local state change must be
 * published if model publication state is configured. If model publication is not configured this
 * API call will not generate any assertion.
 *
 * @param[in] p_app                 Pointer to [app_light_lightness_setup_server_t](@ref
 *                                  __app_light_lightness_setup_server_t) context.
 *
 * @retval NRF_SUCCESS              If status message is successfully published.
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context
 * @retval NRF_ERROR_NO_MEM         No memory available to send the message at this point.
 * @retval NRF_ERROR_NOT_FOUND      The model is not initialized.
 * @retval NRF_ERROR_INVALID_PARAM  The model not bound to application key
 *                                  or publish address not set.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 *
 */
uint32_t app_light_lightness_current_value_publish(app_light_lightness_setup_server_t * p_app);

/** Function to set the lightness value (sending to the hardware) and writing the flash state values.
 *
 * This API is for extending models to call to set the light lightness value when a state publish is
 * not desired (e.g. LC server will be changing the lightness on a continual basis with light
 * harvesting, so a publish is not desired).  This function also writes the flash state values for
 * last and actual.
 *
 * @warning This API must never be called from model transition callback.
 *
 * @param[in] p_app                 Pointer to [app_light_lightness_setup_server_t](@ref
 *                                  __app_light_lightness_setup_server_t) context.
 * @param[in] lightness_value       Actual lightness value to set
 *
 * @retval NRF_SUCCESS              Bindings are setup successfully
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context
 */
uint32_t app_light_lightness_direct_actual_set(app_light_lightness_setup_server_t * p_app, uint16_t lightness_value);

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
/** Sets the scene context
 *
 * This is needed for app light lightness to inform app scene when the state change occurs.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in] p_app                 Pointer to [app_light_lightness_setup_server_t](@ref 
 *                                  __app_light_lightness_setup_server_t) context.
 * @param[in] p_app_scene           Pointer to scene behavioral moduel context.
 *
 * @retval NRF_SUCCESS              Value is restored successfully
 * @retval NRF_ERROR_NULL           If NULL pointer is provided as input context
 */
uint32_t app_light_lightness_scene_context_set(app_light_lightness_setup_server_t * p_app, 
                                               app_scene_setup_server_t  * p_app_scene);
#endif

/** @} end of APP_LIGHT_LIGHTNESS */
#endif /* APP_LIGHT_LIGHTNESS_H__*/
