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

#ifndef APP_LEVEL_H__
#define APP_LEVEL_H__

#include <stdint.h>

#include "generic_level_server.h"
#include "app_timer.h"
#include "fsm.h"

/**
 * @defgroup APP_LEVEL Generic Level server behaviour
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * This module implements the behavioral requirements of the Generic Level server model. You may
 * customize this behaviour to fit your application requirements or hardware interfaces.
 *
 * @note This sample implementation manages the incremental changes in the level value and reports
 * it to the example application to write to hardware interface (in this case, a PWM hardware).
 *
 * @note This implementation assumes the application hardware can handle all range of level values.
 * Necessary scaling required, if any, can be performed in the main application.
 *
 * @note This implementation assumes the wrap around behavior for the level state.
 *
 * Interaction with the user application happens through two callbacks:
 * get_cb,
 * set_cb
 *
 * Additionally, user application must call @ref app_level_current_value_publish() API whenever local
 * action results in the change in `present_level` value.
 *
 * The behavioral module will manage all the timing requirements internally (delay + transitions)
 * and call the `set_cb` whenever it is time to change the level value. User application shall
 * use the `present_level` value in the set cb to drive the necessary hardware.
 * This behavioral interface will trigger large number of Set callbacks to report the changing
 * `present_level` value, therefore user must not do time consuming operations inside the callback.
 *
 * The smallest possible callback interval for a given transition time will be limited by
 * @ref MODEL_TIMER_TIMEOUT_MIN_TICKS.
 *
 * This module will call the `get_cb` to fetch the present level value from the application.
 *
 * <br>
 * @warning To comply with the Mesh Model Specification test cases, the application must adhere to
 * the requirements defined in the following sections:
 * - Section 3.1.2 (Generic Level) and Section 3.3.2.2 (Generic Level state behaviour) of Mesh
 * Model Specification v1.0.
 * - Section 3.7.6.1 (Publish) of Mesh Profile Specification v1.0.
 *
 * These requirements are documented at appropriate places in the module source code.
 *
 * @{
 */

/**
 * Macro to create application level app_level_server_t context.
 *
 * Individual timer instances are created for each model instance.
 *
 * @param[in] _name                 Name of the app_level_server_t instance
 * @param[in] _force_segmented      If the Generic Level server shall use force segmentation of messages
 * @param[in] _mic_size             MIC size to be used by Generic Level server
 * @param[in] _p_dtt                Pointer to the default transition time state if present
 * @param[in] _set_cb               Callback for setting the application state to given value.
 * @param[in] _get_cb               Callback for reading the state from the application.
*/

#define APP_LEVEL_SERVER_DEF(_name, _force_segmented, _mic_size, _p_dtt, _set_cb, _get_cb)  \
    APP_TIMER_DEF(_name ## _timer); \
    static app_level_server_t _name =  \
    {  \
        .server.settings.force_segmented = _force_segmented,  \
        .server.settings.transmic_size = _mic_size,  \
        .timer.p_timer_id = &_name ## _timer,  \
        .p_dtt_ms = _p_dtt, \
        .level_set_cb = _set_cb,  \
        .level_get_cb = _get_cb  \
    };

/** Transition types */
typedef enum
{
    /** indicating SET message */
    TRANSITION_SET,
    /** indicating DELTA SET message */
    TRANSITION_DELTA_SET,
    /** indicating MOVE SET message */
    TRANSITION_MOVE_SET,
    /** indicating no transition */
    TRANSITION_NONE
} app_level_transition_type_t;

/** Internal structure for holding Set/Delta Set transition related variables */
typedef struct
{
    /** For storing actual required amount of level change. */
    int32_t required_delta;
    /** Initial present level required for handling Set/Delta Set message. */
    int16_t initial_present_level;
} set_transition_t;

/** Internal structure for holding Move transition related variables */
typedef struct
{
    /** Scaled representation of the Level value. */
    int16_t required_move;
    /** Initial present level required for handling Set/Delta Set message. */
    int16_t initial_present_level;
} move_transition_t;

/** Internal structure to hold state and timing information. */
typedef struct
{
    /** Present value of the Level state */
    int16_t present_level;
    /** Target value of the Level state, as received from the model interface. */
    int16_t target_level;
    /** Remaining time to reach `target_level`. */
    uint32_t transition_time_ms;
    /** Time to delay the processing of received SET message. */
    uint32_t delay_ms;

    /** Transition Type */
    app_level_transition_type_t transition_type;
    union {
        /* Parameters for Set Transition */
        set_transition_t set;
        /* Parameters for Move Transition */
        move_transition_t move;
    } params;
} app_level_state_t;

/* Forward declaration */
typedef struct __app_level_server_t app_level_server_t;

/** Application state set callback prototype.
 *
 * This callback is called by the this module whenever application is required to
 * be informed to reflect the desired Level value, as a result of the received SET/DELTA SET/MOVE SET
 * message, depending on the received Target Level value and timing parameters.
 *
 * Note: Since the behavioral module encapsulates functionality required for the compliance with timing
 * behaviour, it is not possible to infer number of Level messages received by the
 * node by counting the number of times this callback is triggered. If such counting is required,
 * it should be done in the `app_level.c` module.
 *
 * @param[in]   p_server        Pointer to @ref __app_level_server_t [app_level_server_t] context
 * @param[in]   present_level   Instantaneous new level value to be used by the application
 */
typedef void (*app_level_set_cb_t)(const app_level_server_t * p_server, int16_t present_level);

/** Application state read callback prototype.
 * This callback is called by the app_level.c whenever application level state is required
 * to be read.
 *
 * @param[in]  p_server          Pointer to @ref __app_level_server_t [app_level_server_t] context
 * @param[out] p_present_level   User application fills this value with the value retrived from
 *                               the hardware interface.
 */
typedef void (*app_level_get_cb_t)(const app_level_server_t * p_server, int16_t * p_present_level);

/** Application level structure holding the Level server model context and Level state representation */
struct __app_level_server_t
{
    /** Level server model interface context structure */
    generic_level_server_t server;
    /** Timer instance pointer */
    model_timer_t timer;
    /** Callaback to be called for informing the user application to update the value*/
    app_level_set_cb_t  level_set_cb;
    /** Callback to be called for requesting current value from the user application */
    app_level_get_cb_t level_get_cb;

    /** Pointer to the default transition time value (in milliseconds) if present */
    const uint32_t * p_dtt_ms;
    /** Internal variable. Representation of the Level state related data and transition parameters
     *  required for behavioral implementation, and for communicating with the application */
    app_level_state_t state;

    /** Internal. */
    fsm_t fsm;
};

/** Initiates value fetch from the user application by calling a get callback, updates internal state,
 * and publishes the Generic Level Status message.
 *
 * This API must always be called by an application when user initiated action (e.g. button press) results
 * in the local Level state change. Mesh Profile Specification v1.0 mandates that, every local state
 * change must be published if model publication state is configured. If model publication is not
 * configured this API call will not generate any assertion.
 *
 * @param[in] p_server              Pointer to @ref __app_level_server_t [app_level_server_t] context
 *
 * @retval NRF_SUCCESS              If status message is succesfully published.
 * @returns Other return values returned by the lower layer APIs.
 */
uint32_t app_level_current_value_publish(app_level_server_t * p_server);

/** Initializes the behavioral module for the Generic Level model
 *
 * @param[in] p_server               Pointer to the application Level server struture array.
 * @param[in] element_index          Element index on which this server will be instantiated.
 *
 * @retval  NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                   member variable pointers.
 * @retval  NRF_ERROR_INVALID_PARAM  If value of the `server_count` is zero, or other parameters
 *                                   required by lower level APIs are not correct.
 * @returns Other return values returned by the lower layer APIs.
 *
*/
uint32_t app_level_init(app_level_server_t * p_server, uint8_t element_index);

/** @} end of APP_LEVEL */
#endif /* APP_LEVEL_H__ */
