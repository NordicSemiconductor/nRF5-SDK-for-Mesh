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
#ifndef BEARER_HANDLER_H__
#define BEARER_HANDLER_H__

#include <stdint.h>
#include <stdbool.h>
#include "timeslot_timer.h"
#include "queue.h"
#include "nrf_mesh_config_bearer.h"
#include "nrf_soc.h"
#include "timeslot.h"

/**
 * @defgroup BEARER_HANDLER Bearer context handler
 * @ingroup MESH_BEARER
 * Handles bearer events, and forwards hardware events to the right handlers.
 * @{
 */

/** Estimated maximum runtime of the bearer handler end-of-action postprocessing. */
#define BEARER_ACTION_POST_PROCESS_TIME_US      (100)
/** Maximal duration of a single bearer action. */
#define BEARER_ACTION_DURATION_MAX_US  (NRF_RADIO_LENGTH_MAX_US - BEARER_ACTION_POST_PROCESS_TIME_US - TIMESLOT_STARTUP_OVERHEAD_US - TIMESLOT_END_TIMER_OVERHEAD_US)

#define BEARER_ACTION_TIMER             CONCAT_2(NRF_TIMER, BEARER_ACTION_TIMER_INDEX)
#define BEARER_ACTION_TIMER_IRQn        CONCAT_3(TIMER, BEARER_ACTION_TIMER_INDEX, _IRQn)
#define BEARER_ACTION_TIMER_IRQHandler  CONCAT_3(TIMER, BEARER_ACTION_TIMER_INDEX, _IRQHandler)
/**
 * @defgroup BEARER_CONTEXT_CALLBACKS Callbacks for bearer actions.
 * @{
 */

/**
 * Called at the start of the action.
 *
 * @param[in] start_time Timestamp of the start event, according to the
 * handler. The action must call @ref bearer_handler_action_end() before its
 * start_time + duration. The @ref BEARER_ACTION_TIMER is started at the given start time,
 * and can be used freely throughout the action.
 * @param[in] p_args Argument pointer, as specified by the caller.
 */
typedef void (*bearer_start_cb_t)(ts_timestamp_t start_time, void* p_args);

/**
 * Radio interrupt handler function.
 *
 * @param[in] p_args Argument pointer, as specified by the caller.
 */
typedef void (*bearer_radio_irq_handler_t)(void* p_args);
/**
 * Action timer interrupt handler function.
 *
 * @param[in] p_args Argument pointer, as specified by the caller.
 */
typedef void (*bearer_timer_irq_handler_t)(void* p_args);

/**
 * @}
 */

#ifdef BEARER_HANDLER_DEBUG
typedef struct
{
    ts_timestamp_t prev_duration_us;
    ts_timestamp_t prev_margin_us;
    uint32_t event_count;
} bearer_action_debug_t;
#endif

/**
 * Bearer action parameters. User owned structure that is used to communicate action entry points
 * and parameters. The action's start callback is called as soon as the action is ready for
 * execution, and the action is responsible for calling @ref bearer_handler_action_end() within its
 * set @c duration_us, starting at the @p start_time parameter in the start callback.
 */
typedef struct
{
    bearer_start_cb_t          start_cb;          /**< Start of action-callback for the action. */
    bearer_radio_irq_handler_t radio_irq_handler; /**< Radio interrupt handler for the action. */
    bearer_timer_irq_handler_t timer_irq_handler; /**< Timer interrupt handler for the action. */
    ts_timestamp_t             duration_us;       /**< Upper limit on action execution time in microseconds. Must be lower than @ref BEARER_ACTION_DURATION_MAX_US.*/
    void*                      p_args;            /**< Arguments pointer provided to the callbacks. */

#ifdef BEARER_HANDLER_DEBUG
    bearer_action_debug_t      debug;
#endif

    queue_elem_t               queue_elem;        /**< Linked list queue element, set and used by the module. */
} bearer_action_t;

/** Callback type being called once the bearer handler has been stopped. */
typedef void (*bearer_handler_stopped_cb_t)(void);

/** Initialize the bearer handler. */
void bearer_handler_init(void);

/**
 * Start bearer handler operation.
 *
 * @warning Requires that the bearer_handler_init function has been called.
 *
 * @retval NRF_SUCCESS The bearer handler operation was successfully started.
 * @retval NRF_ERROR_INVALID_STATE The bearer handler is already running.
 */
uint32_t bearer_handler_start(void);

/**
 * Stop bearer handler operation, suspending all radio activity.
 *
 * @note The bearer handler will finish any ongoing actions before stopping.
 * @warning Requires that the bearer_handler_init function has been called.
 *
 * @param[in] cb Stop callback called once the bearer handler has come to rest, or NULL.
 *
 * @retval NRF_SUCCESS Successfully stopped the bearer handler. All radio
 * activity will be suspended, and the callback will be called.
 * @retval NRF_ERROR_INVALID_STATE The bearer handler is already stopped.
 */
uint32_t bearer_handler_stop(bearer_handler_stopped_cb_t cb);

/**
 * Enqueue a single bearer action. The action will go to the back of the action
 * queue, and executed as soon as the handler can fit it in a timeslot.
 *
 * @warning This function requires that:
 *      - The bearer handler has been initialized.
 *      - p_action isn't NULL.
 *      - The action's start callback pointer isn't NULL.
 *      - Duration_us isn't 0 or > @ref BEARER_ACTION_DURATION_MAX_US.
 *
 * @param[in] p_action The action to enqueue. Must be statically allocated.
 *
 * @retval NRF_SUCCESS The event was successfully enqueued.
 * @retval NRF_ERROR_INVALID_STATE The action is already in the queue, and will be executed as soon as possible.
 */
uint32_t bearer_handler_action_enqueue(bearer_action_t* p_action);


/**
 * Wake up the bearer handler.
 *
 * Makes the bearer handler start a timeslot if there's any activity pending.
 */
void bearer_handler_wake_up(void);

/**
 * Fire a single bearer action immediately. If something is blocking the
 * action, like an ongoing action or we don't have the radio for the duration
 * of the action, this function returns @c NRF_ERROR_BUSY, and the action
 * doesn't fire.
 *
 * @warning This function requires that:
 *      - The bearer handler has been initialized.
 *      - p_action isn't NULL.
 *      - The action's start callback pointer isn't NULL.
 *      - Duration_us isn't 0 or > @ref BEARER_ACTION_DURATION_MAX_US.
 *
 * @param[in] p_action The action to fire. Must be statically allocated.
 *
 * @retval NRF_SUCCESS The action was successfully scheduled, and has started execution.
 * @retval NRF_ERROR_BUSY The handler is busy, and can't run the action right
 * away. Try again later.
 */
uint32_t bearer_handler_action_fire(bearer_action_t* p_action);

/**
 * End the current bearer action.
 *
 * @warning Must be called from the RADIO or TIMER interrupt contexts.
 */
void bearer_handler_action_end(void);

/**
 * @internal
 * @defgroup BEARER_HANDLER_INTERNAL Internal bearer handler functions
 * Should not be used by the application.
 * @{
 */

/**
 * Timeslot begin event handler. Called by the timeslot handler module.
 *
 * @warning Should only be called by the timeslot module.
 */
void bearer_handler_on_ts_begin(void);

/**
 * Timeslot end event handler. Called by the timeslot handler module.
 *
 * @warning Should only be called by the timeslot module.
 */
void bearer_handler_on_ts_end(void);

/**
 * Timeslot session closed handler. Called by the timeslot handler module.
 *
 * @warning Should only be called by the timeslot module.
 */
void bearer_handler_on_ts_session_closed(void);

/**
 * Radio IRQ handler. Called by the timeslot handler module.
 *
 * @warning Should only be called by the timeslot module.
 */
void bearer_handler_radio_irq_handler(void);

/**
 * Timer IRQ handler. Called by the timeslot handler module.
 *
 * @warning Should only be called by the timeslot module.
 */
void bearer_handler_timer_irq_handler(void);

/**
 * Enable the force mode of the bearer handler.
 *
 * @note That prevents the time slot stopping after finishing activity
 * even scanner is disabled and there are not queued actions.
 * If the next action is added in this mode the bearer handler does not require
 * to request a new time slot. It tries to fit action in the ongoing slot if possible.
 *
 * It is used to speed up storing the unstored data in case of the power down.
 *
 * @warning This mode enforces power consumption.
 */
void bearer_handler_force_mode_enable(void);

/**
 * Disable the force mode.
 */
void bearer_handler_force_mode_disable(void);

/** @} */

/** @} */

#endif /* BEARER_HANDLER_H__ */

