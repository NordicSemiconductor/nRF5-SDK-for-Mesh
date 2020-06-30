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
#ifndef LONG_TIMER_H__
#define LONG_TIMER_H__

#include "timer_scheduler.h"

/**
 * @defgroup LONG_TIMER Long timer
 * Long running timer for the mesh.
 * @{
 */


/** Callback function type */
typedef void (*lt_callback_t)(void * p_context);

/** Long running timer context. */
typedef struct
{
    uint64_t remaining_time_us; /**< Number of microseconds until the timer fires at the previous tick.
                                     Use @ref lt_remaining_time to get actual value. */
    lt_callback_t callback;     /**< Callback to call when the timer expires. */
    void * p_context;           /**< User specified context passed in the callback. */
    timer_event_t event;        /**< Timer event used to keep track of the remaining time. */
} long_timer_t;

/**
 * Schedule a long running timer with a delay.
 *
 * The memory associated with the @p p_timer must be valid until the timer either fires or is aborted.
 *
 * @param[in,out] p_timer Timer to schedule. Rescheduling a running timer will cancel its original timeout value.
 * @param[in] callback Callback to call when the timer finishes.
 * @param[in,out] p_context Context to pass to the timer.
 * @param[in] delay_us Delay in microseconds before the timer should fire.
 */
void lt_schedule(long_timer_t * p_timer, lt_callback_t callback, void * p_context, uint64_t delay_us);

/**
 * Get the time remaining until the timer fires.
 *
 * @param[in] p_timer Timer to check
 *
 * @returns The number of microseconds until the timer fires.
 */
uint64_t lt_remaining_time_get(const long_timer_t * p_timer);

/**
 * Abort a scheduled timer.
 *
 * The timer callback will not fire, and the memory can safely be freed.
 *
 * @param[in,out] p_timer Timer to abort.
 */
void lt_abort(long_timer_t * p_timer);

/** @} */

#endif /* LONG_TIMER_H__ */
