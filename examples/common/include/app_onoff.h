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

#ifndef APP_ONOFF_H__
#define APP_ONOFF_H__

#include <stdint.h>

#include "generic_onoff_server.h"
#include "app_transition.h"
#include "app_timer.h"
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
#include "app_scene.h"
#endif


/**
 * @defgroup APP_ONOFF Generic OnOff server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application level OnOff server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Generic OnOff server model.
 *
 * The application should use the set/transition callback provided by this module to set the
 * hardware state. The hardware state could be changed by reflecting the value provided by the
 * set/transiton_time callback on the GPIO or by sending this value to the connected lighting
 * peripheral using some other interface (e.g. serial interface). Similarly, the application should
 * use the get callback provided by this module to read the hardware state.
 *
 * This module triggers the set/transition callback only when it determins that it is time to
 * inform the user application. It is possible that the client can send multiple overlapping
 * set/transition commands. In such case any transition in progress will be abandoned and fresh
 * transition will be started if required.
 *
 * Using transition_cb:
 * If the underlaying hardware does not support setting of the instantaneous value provided via
 * `set_cb`, the `transition_cb` can be used to implement the transition effect according to
 * provided transition parameters. This callback will be called when transition start with the
 * required transition time and target value. When the transition is complete this callback will be
 * called again with transition time set to 0 and the desired target value.
 * <br>
 * @warning To comply with the @tagMeshMdlSp test cases, the application must adhere to
 * the requirements defined in the following sections:
 * - @tagMeshMdlSp section 3.1.1 (Generic OnOff) and section 3.3.1.2 (Generic OnOff state behaviour).
 * - @tagMeshSp section 3.7.6.1 (Publish).
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/**
 * Macro to create application level app_onoff_server_t context.
 *
 * Individual timer instances are created for each model instance.
 *
 * @param[in] _name                 Name of the app_onoff_server_t instance
 * @param[in] _force_segmented      If the Generic OnOff server shall use force segmentation of messages
 * @param[in] _mic_size             MIC size to be used by Generic OnOff server
 * @param[in] _set_cb               Callback for setting the application state to given value.
 * @param[in] _get_cb               Callback for reading the state from the application.
 * @param[in] _transition_cb        Callback for setting the application transition time and state value to given values.
 */
#define APP_ONOFF_SERVER_DEF(_name, _force_segmented, _mic_size, _set_cb, _get_cb, _transition_cb)  \
    APP_TIMER_DEF(_name ## _timer); \
    static app_onoff_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .state.transition.timer.p_timer_id = &_name ## _timer,  \
        .onoff_set_cb = _set_cb,  \
        .onoff_get_cb = _get_cb,  \
        .onoff_transition_cb = _transition_cb  \
    };


/** Internal structure to hold state and timing information. */
typedef struct
{
    /** Present value of the OnOff state */
    bool present_onoff;
    /** Initial value for transition */
    bool initial_present_onoff;
    /** Target value of the OnOff state, as received from the model interface. */
    bool target_onoff;
    /** Structure for using transition module functionality */
    app_transition_t transition;
} app_onoff_state_t;

/* Forward declaration */
typedef struct __app_onoff_server_t app_onoff_server_t;

/** Application state set callback prototype.
 *
 * This callback is called by the this module whenever application is required to
 * be informed to reflect the desired OnOff value, as a result of the received SET message. Depending
 * on the received Target OnOff value and timing parameters, this callback may be triggered after the
 * delay+transition time is over or instantly after the delay if the Target OnOff value is `1`, as
 * required by @tagMeshMdlSp.
 *
 * Note: Since the behavioral module encapsulates functionality required for the compliance with timing
 * behaviour, it is not possible to infer number of Generic OnOff Set messages received by the
 * node by counting the number of times this callback is triggered.
 *
 * @param[in]   p_app           Pointer to [app_onoff_server_t](@ref __app_onoff_server_t) context.
 * @param[in]   onoff           New onoff value to be used by the application
 */
typedef void (*app_onoff_set_cb_t)(const app_onoff_server_t * p_app, bool onoff);

/** Application state read callback prototype.
 * This callback is called by the app_model_behaviour.c whenever application onoff state is required
 * to be read.
 *
 * @param[in]  p_app             Pointer to [app_onoff_server_t](@ref __app_onoff_server_t) context.
 * @param[out] p_present_onoff   User application fills this value with the value retrived from
 *                               the hardware interface. See @ref model_callback_pointer_note.
 */
typedef void (*app_onoff_get_cb_t)(const app_onoff_server_t * p_app, bool * p_present_onoff);

/** Application transition time callback prototype.
 *
 * This callback is called by the this module whenever application is required to be informed to
 * reflect the desired transition time, depending on the received target onoff value and timing
 * parameters.
 *
 * @param[in] p_app              Pointer to [app_onoff_server_t](@ref __app_onoff_server_t) context.
 * @param[in] transition_time_ms Transition time (in milliseconds) to be used by the application.
 * @param[in] target_onoff       Target onoff value to be used by the application.
 *
 */
typedef void (*app_onoff_transition_cb_t)(const app_onoff_server_t * p_app,
                                               uint32_t transition_time_ms,
                                               bool target_onoff);

/** Application level structure holding the OnOff server model context and OnOff state representation */
struct __app_onoff_server_t
{
    /** OnOff server model interface context structure */
    generic_onoff_server_t server;
    /** APP timer instance pointer */
    app_timer_id_t const * p_timer_id;
    /** Callaback to be called for informing the user application to update the value*/
    app_onoff_set_cb_t  onoff_set_cb;
    /** Callback to be called for requesting current value from the user application */
    app_onoff_get_cb_t onoff_get_cb;
    /** Callaback to be called for informing the user application to update the value*/
    app_onoff_transition_cb_t onoff_transition_cb;

    /** Internal variable. Representation of the OnOff state related data and transition parameters
     *  required for behavioral implementation, and for communicating with the application */
    app_onoff_state_t state;
    /** Internal variable. It is used for acquiring RTC counter value. */
    uint32_t last_rtc_counter;
#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)
    /** Internal variable. Scene callback interface.
     * @note Available only if  @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1. */
    app_scene_model_interface_t scene_if;
    /** Internal variable. Pointer to app_scene context.
     * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1. */
    app_scene_setup_server_t  * p_app_scene;
#endif
};

/** Initiates value fetch from the user application by calling a get callback, updates internal
 * state, and publishes the Generic OnOff Status message.
 *
 * This API must always be called by an application when user initiated action (e.g. button press)
 * results in the local OnOff state change. This API should never be called from transition
 * callback. @tagMeshSp mandates that, every local state change must be
 * published if model publication state is configured. If model publication is not configured this
 * API call will not generate any error condition.
 *
 * @param[in] p_app             Pointer to [app_onoff_server_t](@ref __app_onoff_server_t) context.
 */
void app_onoff_status_publish(app_onoff_server_t * p_app);

/** Initializes the behavioral module for the generic OnOff model
 *
 * @param[in] p_app                 Pointer to [app_onoff_server_t](@ref __app_onoff_server_t)
 *                                  context.
 * @param[in] element_index         Element index on which this server will be instantiated.
 *
 * @retval NRF_SUCCESS              If initialization is successful.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated, or
 *                                  no more subscription lists available in memory pool.
 * @retval NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                  member variable pointers.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index, or access handle invalid.
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval  NRF_ERROR_INVALID_PARAM Model not bound to appkey, publish address not set or wrong
 *                                  opcode format. The application timer module has not been
 *                                  initialized or timeout handler is not provided.
 * @retval NRF_ERROR_INVALID_STATE  If the application timer is running.
*/
uint32_t app_onoff_init(app_onoff_server_t * p_app, uint8_t element_index);

/** Restores the onoff value from persistent storage
 *
 * This is called by main.c when the mesh is initialized and stable.
 * Note that this function must be called from the same IRQ level that
 * mesh_init() is set at.
 *
 * @param[in] p_app                  Pointer to [app_onoff_server_t](@ref __app_onoff_server_t)
 *                                   context.
 *
 * @retval NRF_SUCCESS               Value is restored successfully
 * @retval NRF_ERROR_NULL            If NULL pointer is provided as input context
 */
uint32_t app_onoff_value_restore(app_onoff_server_t * p_app);

#if (SCENE_SETUP_SERVER_INSTANCES_MAX > 0) || (DOXYGEN)

/** Sets the scene context
 *
 * This is needed for app onoff to inform app scene when the state change occurs.
 * @note Available only if @ref SCENE_SETUP_SERVER_INSTANCES_MAX is equal or larger than 1.
 *
 * @param[in] p_app                  Pointer to [app_onoff_server_t](@ref __app_onoff_server_t)
 *                                   context.
 * @param[in] p_app_scene            Pointer to scene behavioral moduel context.*
 *
 * @retval NRF_SUCCESS               Value is restored successfully
 * @retval NRF_ERROR_NULL            If NULL pointer is provided as input context
 */
uint32_t app_onoff_scene_context_set(app_onoff_server_t * p_app, app_scene_setup_server_t  * p_app_scene);

#endif

/** @} end of APP_ONOFF */
#endif /* APP_ONOFF_H__ */
