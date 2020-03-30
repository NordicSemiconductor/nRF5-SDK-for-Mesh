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
#ifndef MESH_TIMER_H__
#define MESH_TIMER_H__

#include <stdint.h>

/**
 * @defgroup TIMER Hardware dependent part of the timer scheduler and the system time
 * @ingroup MESH_CORE
 * Abstract timer API which is based on SDK app_timer (RTC).
 * @{
 */

/** Maximum error that can be introduced while converting RTC ticks to microsecond timestamps by the
 * use of timer_now() API */
#define TIMER_NOW_MAX_ERROR_US   (31)

/** Get timestamp - ref, including rollover. */
#define TIMER_DIFF(timestamp, reference) timer_diff(timestamp, reference)

/**
 * Checks whether the given time is older than the reference (occurs before in time).
 *
 * @verbatim
 -----------|---------------------|-----------> t
            t1                    t2

 TIMER_OLDER_THAN(t1, t2) => true
 TIMER_OLDER_THAN(t2, t1) => false
 @endverbatim
 */
#define TIMER_OLDER_THAN(time, ref) (((uint32_t) (time)) - ((uint32_t) (ref)) > UINT32_MAX / 2)

/** Timestamp type for all time-values */
typedef uint32_t timestamp_t;

/** Callback type for callbacks at finished timers */
typedef void(* timer_callback_t)(timestamp_t);

/**
 * Get the absolute difference between two lf_timestamps, regardless of order.
 *
 * @param[in] time1 First timestamp to compare
 * @param[in] time2 Second timestamp to compare
 *
 * @returns The difference between the two time parameters.
 */
static inline timestamp_t timer_diff(timestamp_t time1, timestamp_t time2)
{
    if (time1 - time2 > UINT32_MAX / 2)
    {
        return time2 - time1;
    }
    else
    {
        return time1 - time2;
    }
}

/**
 * Initialize SDK app timer. Create timer instances and run repeatable timer.
 */
void timer_init(void);

/**
 * Return timestamp of the system time since initialization moment.
 *
 * @returns timestamp in us.
 */
timestamp_t timer_now(void);

/**
 * Start rest timeout according to the provided timestamp.
 * If timestamp has already expired then timer will start on minimum margin time
 * (till 2 ticks = approximately 62us).
 * Callback will be invoked after expiration
 *
 * @note Registered callback is invoked from RTC1 interrupt handler
 *       with appropriate priority (differ to mesh stack priorities).
 *
 * @param[in] timestamp timestamp for calculation rest of time
 * @param[in] cb        callback pointer
 */
void timer_start(timestamp_t timestamp, timer_callback_t cb);

/**
 * Stop the ongoing timeout if it exists.
 */
void timer_stop(void);

/** @} */

#endif /* MESH_TIMER_H__ */
