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

#ifndef APP_SCENE_H__
#define APP_SCENE_H__

#include <stdint.h>

#include "scene_setup_server.h"
#include "app_transition.h"
#include "nrf_mesh_config_examples.h"

/**
 * @defgroup APP_SCENE Scene Server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application level Scene server behavioral structures, functions, and callbacks.
 *
 * This module implements the behavioral requirements of the Scene Setup Server model.
 *
 * The Scene server requires a composite state for the Scene Register state, the Current Scene
 * state, and the Target Scene state. The application should use the store/delete callback provided
 * by this module to store/delete scenes states. The Scene is determined by the model associated
 * with the scene. The values of the states that are stored as a scene and can be recalled later.
 *
 * This module triggers the recall callback only when it determines that it is time to inform the
 * user application about a model transition. The Scenes state change may start numerous parallel
 * model transitions. Each model handles the transition internally. The scene transions is in
 * progress when at least one transition from the group of individual model transitions is in
 * progress
 *
 * These callbacks should be implemented by those models that should be stored with scenes (see
 * "Stored with Scene" column in @tagMeshMdlSp for each model).
 * <br>
 * @warning To comply with the @tagMeshMdlSp test cases, the application must adhere to
 * the requirements defined in the following sections:
 * - @tagMeshMdlSp section 5.1.3 (Scenes) and section 5.2.2 (Scene messages).
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/**
 * Macro to create application level app_scene_setup_server_t context.
 *
 * Individual timer instances are created for each model instance.
 *
 * @param[in] _name                     Name of the app_scene_server_t instance
 * @param[in] _force_segmented          If the Scene server shall use force segmentation of messages
 * @param[in] _mic_size                 MIC size to be used by Scene server
 * @param[in] _transition_cb            Callback for setting the application transition time and
 *                                      state value to given values.
 * @param[in] _p_dtt_server             Pointer to default transition time server instance.
*/
#define APP_SCENE_SETUP_SERVER_DEF(_name, _force_segmented, _mic_size, _transition_cb, _p_dtt_server);  \
    APP_TIMER_DEF(_name ## _timer);                                             \
    static app_scene_setup_server_t _name =                                     \
    {                                                                           \
        .scene_setup_server.settings.force_segmented = _force_segmented,        \
        .scene_setup_server.settings.transmic_size = _mic_size,                 \
        .state.transition.timer.p_timer_id = &_name ## _timer,                  \
        .app_scene_transition_cb = _transition_cb,                              \
        .scene_setup_server.p_gen_dtt_server = _p_dtt_server                        \
    };

/** Number of Scene Models to support storage.
 */
#ifndef APP_SCENE_MODEL_COUNT
#define APP_SCENE_MODEL_COUNT (1)
#endif

/** Internal structure to hold state.
 *
 * The information about all the stored scenes for the device resides in [scene_mc](@ref SCENE_MC)
 * module.
 */
typedef struct
{
    /** Current scene number for the active scene. */
    uint16_t current_scene_number;
    /** Target scene number for transition to active scene, as received from the model interface. */
    uint16_t target_scene_number;

    /** Structure for using transition module functionality */
    app_transition_t transition;
} app_scene_state_t;

/* Forward declaration */
typedef struct __app_scene_setup_server_t app_scene_setup_server_t;
/* Forward declaration */
typedef struct __app_scene_model_interface_t app_scene_model_interface_t;

/** Application transition time callback prototype.
 *
 * This callback is called by the this module whenever application is required to be informed to
 * reflect the desired transition time, depending on the received target onoff value and timing
 * parameters.
 *
 * @param[in] p_app              Pointer to [app_scene_setup_server_t](@ref
 *                               __app_scene_setup_server_t) context.
 * @param[in] transition_time_ms Transition time (in milliseconds) to be used by the application.
 * @param[in] target_scene       Target scene number value to be used by the application.
 */
typedef void (*app_scene_transition_cb_t)(const app_scene_setup_server_t * p_app,
                                          uint32_t transition_time_ms,
                                          uint16_t target_scene);

/** Application Scene state store callback prototype.
 *
 * This callback is called by this module whenever the application is required to be informed to
 * reflect the desired scene number to be stored. All the register applications models will also
 * store the state values for this scene number.
 *
 * @param[in] p_app_model_if    Pointer to [app_scene_model_interface_t] (@ref
 *                              __app_scene_model_interface_t) context.
 * @param[in] scene_index       Scene index extraced from storage based on scene number to be
 *                              stored.
 */
typedef void (*app_scene_store_cb_t)(const app_scene_model_interface_t * p_app_model_if,
                                     uint8_t scene_index);


/** Application Scene state recall callback prototype.
 *
 * This callback is called by this module whenever the application is required to be informed to
 * reflect the desired scene number to be recalled. All the register applications models will also
 * recall the state values for this scene number.
 *
 * @param[in] p_app_model_if     Pointer to [app_scene_model_interface_t] (@ref
 *                               __app_scene_model_interface_t) context.
 * @param[in] scene_index        Scene index extraced from storage based on scene number to be
 *                               recalled.
 * @param[in] delay_ms           Delay in milliseconds.
 * @param[in] transition_time_ms Transition time in milliseconds.
 *
 */
typedef void (*app_scene_recall_cb_t)(const app_scene_model_interface_t * p_app_model_if,
                                      uint8_t scene_index,
                                      uint32_t delay_ms,
                                      uint32_t transition_time_ms);

/** Application Scene state delete callback prototype.
 *
 * This callback is called by this module whenever the application is required to be informed to
 * reflect the desired scene number to be deleted. All the register applications models will also
 * delete the state values for this scene number.
 *
 * @param[in] p_app_model_if    Pointer to [app_scene_model_interface_t] (@ref
 *                              __app_scene_model_interface_t) context.
 * @param[in] scene_index       Scene index extraced from storage based on scene number to be
 *                              deleted.
 */
typedef void (*app_scene_delete_cb_t)(const app_scene_model_interface_t * p_app_model_if,
                                      uint8_t scene_index);


typedef struct
{
    app_scene_store_cb_t    scene_store_cb;
    app_scene_recall_cb_t   scene_recall_cb;
    app_scene_delete_cb_t   scene_delete_cb;
} app_scene_callbacks_t;

/* Interface that is to be used by other App Models. */
struct __app_scene_model_interface_t
{
    const app_scene_callbacks_t * p_callbacks;
};

/** Application level structure holding the Scene server model context and sensor state
 * representation */
struct __app_scene_setup_server_t
{
    scene_setup_server_t scene_setup_server;

    /** The device scene transition time */
    app_scene_transition_cb_t app_scene_transition_cb;

    /** Internal variable. Representation of the Scene state related data and transition parameters
     *  required for behavioral implementation, and for communicating with the application. */
    app_scene_state_t state;
    /** Internal variable. App Scene stores pointers to all registered models. */
    app_scene_model_interface_t * scene_models[APP_SCENE_MODEL_COUNT];
    /** Internal variable. Number of registered models in App Scene. */
    uint32_t next_model_interface;
};

/** Initializes the behavioral module for the Scene model
 *
 * @param[in] p_app                 Pointer to [app_scene_setup_server_t](@ref
 *                                  __app_scene_setup_server_t) context.
 * @param[in] element_index         Element index on which this server will be instantiated.
 *
 * @retval NRF_SUCCESS              The model is initialized successfully.
 * @retval NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                  member variable pointers.
 * @retval NRF_ERROR_NO_MEM         @ref ACCESS_MODEL_COUNT number of models already allocated
 *                                  or no more subscription lists available in memory pool
 *                                  (see @ref ACCESS_SUBSCRIPTION_LIST_COUNT).
 * @retval NRF_ERROR_FORBIDDEN      Multiple model instances per element are not allowed or changes
 *                                  to device composition are not allowed. Adding a new model after
 *                                  device is provisioned is not allowed.
 * @retval NRF_ERROR_NOT_FOUND      Invalid access element index.
*/
uint32_t app_scene_model_init(app_scene_setup_server_t * p_app, uint8_t element_index);

/** The API is to be called by application to register a model which is to be stored with scene.
 *
 * @param[in] p_app                         Pointer to [app_scene_setup_server_t](@ref
 *                                          __app_scene_setup_server_t) context.
 * @param[in] p_app_scene_model_interface   Pointer to model that should be registered with scene
 *                                          storage.
 *
 * @retval NRF_SUCCESS              The model is initialized successfully.
 * @retval NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                  member variable pointers.
 * @retval NRF_ERROR_NO_MEM         The (/@ref APP_SCENE_MODEL_COUNT) number of models already
 *                                  added.
 */
uint32_t app_scene_model_add(app_scene_setup_server_t * p_app,
                             app_scene_model_interface_t * p_app_scene_model_interface);

/**
 * This API is called by the behavioral modules of other models to inform that the current state of
 * the device has been changed.
 *
 * To comply with @tagMeshMdlSp section 5.1.3.2.1, when any of the model states that are marked
 * as "stored with scene" change as a result of the operation other than the Scene Recall
 * operation (for example pressing of a button or receiving a SET message), this API must be
 * called by associated modules to inform Scene behavioral module to reset the current scene.
 *
 * @param[in] p_app                 Pointer to [app_scene_setup_server_t](@ref
 *                                  __app_scene_setup_server_t) context.
 */
void app_scene_model_scene_changed(app_scene_setup_server_t * p_app);

/** @} end of APP_SCENE */

#endif /* APP_SCENE_H__ */
