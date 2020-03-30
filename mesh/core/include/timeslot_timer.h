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
#ifndef TS_TIMER_H__
#define TS_TIMER_H__
#include <stdint.h>
#include <stdbool.h>
#include "timer.h"

/**
* @defgroup TS_TIMER HF Timer module abstraction
* @ingroup MESH_CORE
* Allocates timers and schedules PPI signals in PPI channels 8-10. Can also do callbacks at timeout.
* @{
*/

/** First channel to use for timer PPI triggering */
#define TS_TIMER_PPI_CH_START  (8)
#define TS_TIMER_PPI_CH_STOP   (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_TIMESTAMP)
/** Invalid timestamp */
#define TS_TIMER_TIMEOUT_INVALID (0xFFFFFFFF)

/** Timer index for end of timeslot timing */
#define TS_TIMER_INDEX_TS_END      (0)
/** Timer index for scheduler timing */
#define TS_TIMER_INDEX_SCHEDULER   (1)
/** Timer index for radio timing */
#define TS_TIMER_INDEX_RADIO       (2)
/** Timer index for getting timestamps */
#define TS_TIMER_INDEX_TIMESTAMP   (3)

/** Timestamp type for time-values based on the timer working within timeslots*/
typedef uint32_t ts_timestamp_t;

/** Callback type for callbacks at finished timers */
typedef void(*ts_timer_callback_t)(ts_timestamp_t timestamp);

/** Hardware event handler, should be called at all TIMER0 events. */
void ts_timer_event_handler(void);

/**
 * Return timestamp of the time within timeslots.
 *
 * @returns timestamp in us.
 */
ts_timestamp_t ts_timer_now(void);

/**
 * Get the device timestamp for when the ts_timer was started.
 * @returns Start time of the current ts_timer.
 */
timestamp_t ts_timer_start_get(void);

/**
 * Convert a timeslot timestamp to a device timestamp.
 *
 * @warning Should only be used in the timeslot IRQ level, within the timeslot the timestamp was
 * generated in, as the conversion is based on the start time of the current timeslot.
 * @param[in] timestamp Timeslot timestamp to convert
 *
 * @returns The device timestamp corresponding to the given timeslot timestamp.
 */
timestamp_t ts_timer_to_device_time(ts_timestamp_t timestamp);

/**
 * Order a timer with callback.
 *
 * @param[in] timer Timer index to register timeout on. Overrides any previous timeout on this index.
 * @param[in] time Timestamp at which the action should trigger.
 * @param[in] callback Function pointer to the callback function called when the timer triggers.
 *
 * @retval NRF_SUCCESS The callback was successfully scheduled.
 * @retval NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 */
uint32_t ts_timer_order_cb(uint8_t timer,
                           ts_timestamp_t time,
                           ts_timer_callback_t callback);

/**
 * Order a timer with callback and a PPI task.
 *
 * @param[in] timer Timer index to register timeout on. Overrides any previous timeout on this index.
 * @param[in] time Timestamp at which the action should trigger.
 * @param[in] callback Function pointer to the callback function called when the timer triggers.
 * @param[in] p_task PPI task to trigger when the timer expires.
 *
 * @retval NRF_SUCCESS The callback and PPI task were successfully scheduled.
 * @retval NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 */
uint32_t ts_timer_order_cb_ppi(uint8_t timer,
                               ts_timestamp_t time,
                               ts_timer_callback_t callback,
                               uint32_t* p_task);

/**
 * Order a timer with a PPI task.
 *
 * @param[in] timer Timer index to register timeout on. Overrides any previous timeout on this index.
 * @param[in] time Timestamp at which the action should trigger.
 * @param[in] p_task PPI task to trigger when the timer expires.
 *
 * @retval NRF_SUCCESS The PPI task was successfully scheduled.
 * @retval NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 */
uint32_t ts_timer_order_ppi(uint8_t timer,
                            ts_timestamp_t time,
                            uint32_t* p_task);

/**
 * Abort timer with given index.
 *
 * @param[in] timer Index to abort.
 *
 * @retval NRF_SUCCESS The timer was successfully aborted.
 * @retval NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 * @retval NRF_ERROR_NOT_FOUND The given timer wasn't scheduled.
 */
uint32_t ts_timer_abort(uint8_t timer);

/**
* Initialize timer hardware. Must be called at the beginning of each
*   SD granted timeslot. Flushes all timer slots.
*/
void ts_timer_on_ts_begin(void);

/**
* Halt timer module operation. Must be called at the end of each
*   SD granted timeslot.
*
* @param[in] timeslot_end_time Timestamp for the end of the current
*            timeslot.
*/
void ts_timer_on_ts_end(ts_timestamp_t timeslot_end_time);

/** @} */
#endif /* TS_TIMER_H__ */
