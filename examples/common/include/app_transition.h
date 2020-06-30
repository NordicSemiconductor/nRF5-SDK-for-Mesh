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

#ifndef APP_TRANSITION_H__
#define APP_TRANSITION_H__

#include <stdint.h>

#include "app_timer.h"
#include "model_common.h"
#include "fsm.h"
#include "timer_scheduler.h"

/**
 * @defgroup APP_TRANSITION Generic transition module
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * This module implements the transitional behavior commonly required by mesh models. This
 * transition module provides a generic sample based way to implement incremental value changes.
 *
 * This module provides four callbacks to indicate various stages of transitions. The higher level
 * module should implement the code for the moving the state from present value to target
 * value with the use of these callbacks.
 *
 * Depending upon the transition parameters, this module may trigger large number of
 * @ref app_transition_transition_tick_cb_t callbacks, therefore user must not do time
 * consuming operations inside the callback.
 *
 * The smallest possible callback interval for a given transition time will be limited by
 * @ref MODEL_TIMER_TIMEOUT_MIN_TICKS.
 *
 * @{
 */


/** Transition types */
typedef enum
{
    /** indicating SET message */
    APP_TRANSITION_TYPE_SET,
    /** indicating DELTA SET message */
    APP_TRANSITION_TYPE_DELTA_SET,
    /** indicating MOVE SET message */
    APP_TRANSITION_TYPE_MOVE_SET,
    /** indicating no transition */
    APP_TRANSITION_TYPE_NONE
} app_transition_type_t;

/* Transition parameters */
typedef struct
{
    /** Remaining time to reach `target_level`. */
    uint32_t transition_time_ms;
    /** Computed transition delta */
    int32_t required_delta;
    /** The minimum time for a transition step. */
    uint32_t minimum_step_ms;
    /** Transition Type */
    app_transition_type_t transition_type;
} app_transition_params_t;


/** App transition context data type */
typedef struct __app_transition_t app_transition_t;

/** Delay start callback prototype
 *
 * This callback is called by the app_transition module at the begining of delay interval.
 *
 * @param[out]  p_transition   Pointer to transition context
 */
typedef void (*app_transition_delay_start_cb_t)(const app_transition_t * p_transition);

/** Transition start callback prototype
 *
 * This callback is called by the app_transition module at the start of value transition to reach
 * target state.
 *
 * @param[out]  p_transition   Pointer to transition context
 */
typedef void (*app_transition_transition_start_cb_t)(const app_transition_t * p_transition);

/** Transition tick callback prototype
 *
 * This callback is called by the app_transition module to inform parent module to advance the
 * current value to the next step. Depending on the choice of underlaying lighting peripheral
 * application may or may not use this callback.
 *
 * @param[out]  p_transition   Pointer to transition context
 */
typedef void (*app_transition_transition_tick_cb_t)(const app_transition_t * p_transition);

/** Transition complete callback prototype
 *
 * This callback is called by the app_transition module to indicate the end of the transition.
 *
 * @note In case of MOVE transition, this callback is never called. The move transition stops
 * if it is aborted explicitly or if a new non-move transition is triggered.
 *
 * @param[out]  p_transition   Pointer to transition context
 */
typedef void (*app_transition_transition_complete_cb_t)(const app_transition_t * p_transition);

/** Internal structure to hold transition cbs and timing information. */
struct __app_transition_t
{
    /** Callback to call when transition delay beings */
    app_transition_delay_start_cb_t delay_start_cb;
    /** Callback to call when transition beings */
    app_transition_transition_start_cb_t transition_start_cb;
    /** Callback to call on every transition tick */
    app_transition_transition_tick_cb_t transition_tick_cb;
    /** Callback to call on transition time complete */
    app_transition_transition_complete_cb_t transition_complete_cb;

    /** Requested transition parameters */
    app_transition_params_t requested_params;
    /** Ongoing transition parameters */
    app_transition_params_t ongoing_params;

    /** Time to delay the requested transition. */
    uint32_t delay_ms;

    /** Transition timer */
    model_timer_t timer;
    /** Delay timer */
    timer_event_t delay_timer;
    /** Context to be passed to triggered callbacks */
    void * p_context;
    /** Internal. */
    fsm_t fsm;
};

/** Gets the remaining transition time in milliseconds.
 *
 * @note If delay is being executed, this function will return the total transition time requested
 * at the start of the transition.
 *
 * @param[in]  p_transition   Pointer to transition context.
 *
 * @returns Transition time in milliseconds.
 */
uint32_t app_transition_remaining_time_get(app_transition_t * p_transition);

/** Gets the elapsed transition time in milliseconds.
 *
 * @note If delay is being executed, this function will return zero.
 *
 * @param[in]  p_transition   Pointer to transition context.
 *
 * @returns Transition time in milliseconds.
 */
uint32_t app_transition_elapsed_time_get(app_transition_t * p_transition);

/** Checks if the transition time has been complete.
 *
 *
 * @param[in]  p_transition   Pointer to transition context.
 *
 * @retval  True    If transition has been completed or no transition is being executed.
 * @retval  False   If transition has not been completed.
 */
bool app_transition_time_complete_check(app_transition_t * p_transition);

/** Starts the transition with specified transition parameters
 *
 * @param[in]  p_transition   Pointer to transition context.
 */
void app_transition_trigger(app_transition_t * p_transition);

/** Aborts the transition if any in progress.
 *
 * @param[in]  p_transition   Pointer to transition context.
 */
void app_transition_abort(app_transition_t *p_transition);

/** Initializes the transition module
 *
 * @param[in] p_transition          Pointer to the app_transition_t structure
 *
 * @retval NRF_SUCCESS              The transition module is initialized successfully.
 * @retval NRF_ERROR_NULL           NULL pointer is supplied to the function or to the required
 *                                  member variable pointers.
 * @retval NRF_ERROR_INVALID_PARAM  If the application timer module has not been initialized.
 * @retval NRF_ERROR_INVALID_STATE  If the application timer is running.
*/
uint32_t app_transition_init(app_transition_t * p_transition);

/** Gets a pointer to the structure with parameters for the requested transition.
 *
 * @param[in] p_transition Pointer to the app_transition_t structure
 *
 * @return  Pointer to the parameters.
*/
static inline app_transition_params_t * app_transition_requested_get(app_transition_t * p_transition)
{
    return &p_transition->requested_params;
}

/** Gets a pointer to the structure with parameters of the ongoing transition.
 *
 * @param[in] p_transition Pointer to the app_transition_t structure
 *
 * @return  Pointer to the parameters.
*/
static inline app_transition_params_t * app_transition_ongoing_get(app_transition_t * p_transition)
{
    return &p_transition->ongoing_params;
}

/** @} end of APP_LEVEL */
#endif /* APP_TRANSITION_H__ */
