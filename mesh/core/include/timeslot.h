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
#ifndef MESH_TIMESLOT_H__
#define MESH_TIMESLOT_H__

#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "timeslot_timer.h"
#include "nrf_mesh.h"
#include "toolchain.h"

/**
 * @defgroup TIMESLOT Timeslot handler
 * @ingroup MESH_CORE
 *   Module responsible for providing a safe interface to Softdevice Timeslot
 *   API. Handles all timeslot-related system events, makes sure all timeslots
 *   are ended in time, provides some simple functions for manipulating the way
 *   timeslots behave.
 * @{
 */

/** Base timeslot length for long (default) timeslots. Longer than the 10ms
 * jitter of advertisements to provide stability. */
#define TIMESLOT_BASE_LENGTH_LONG_US  (14000UL)
/** Base timeslot length for short timeslots. Should be small enough to fit
 * between the shortest allowed BLE connection interval (7.5ms). */
#define TIMESLOT_BASE_LENGTH_SHORT_US (3800UL)
/** Allocated time between end timer timeout and actual timeslot end. Should be
 * longer than the longest global IRQ lock in the system. */
#define TIMESLOT_END_SAFETY_MARGIN_US (100UL)
/** The upper limit for the length of a single timeslot. Has to be lower than
 * the 24bit TIMER0 rollover, as inforced by the Softdevice. */
#define TIMESLOT_MAX_LENGTH_US        (10000000UL)
/** Shortest length to try to extend before giving up. Smaller increments are
 * considered too insignificant to be worth the overhead. */
#define TIMESLOT_EXTEND_LENGTH_MIN_US (3800UL)
/** The worst case for the shortest possible timeout for the earliest timeslot
 * request. This is the time between sd_radio_request(earliest) call and
 * the actual timeslot START signal. */
#define TIMESLOT_SHORTEST_START_TIME_US  (1900)

#if defined(HOST)
/** Time spent inside the end-timer handler. */
#define TIMESLOT_END_TIMER_OVERHEAD_US (0)
/** Time required to start up the timeslot. */
#define TIMESLOT_STARTUP_OVERHEAD_US  (0)
#elif defined(NRF51)
/** Time spent inside the end-timer handler. */
#define TIMESLOT_END_TIMER_OVERHEAD_US  (55)
/** Time required to start up the timeslot. */
#define TIMESLOT_STARTUP_OVERHEAD_US  (450)
#elif defined(NRF52_SERIES)
/** Time spent inside the end-timer handler. */
#define TIMESLOT_END_TIMER_OVERHEAD_US  (20)
/** Time required to start up the timeslot. */
#define TIMESLOT_STARTUP_OVERHEAD_US  (200)
#endif

/** Priority to use when ordering timeslots. */
typedef enum
{
    TIMESLOT_PRIORITY_LOW,
    TIMESLOT_PRIORITY_HIGH,
} timeslot_priority_t;

/**
 * Event handler for softdevice events.
 *
 * @param[in] evt Softdevice event to process.
 */
void timeslot_sd_event_handler(uint32_t evt);

/**
 * Initialize timeslot handler.
 *
 * @param[in] lfclk_accuracy_ppm Accuracy of the low frequency clock used by the Softdevice.
 */
void timeslot_init(uint32_t lfclk_accuracy_ppm);

/**
 * Start ordering timeslots.
 *
 * @warning Calling this function when interrupts are disabled causes HardFault.
 *
 * @retval NRF_SUCCESS The timeslot was successfully ordered.
 * @retval NRF_ERROR_BUSY There is already a timeslot session in progress.
 */
uint32_t timeslot_start(void);

/**
 * Forcibly stop the timeslot execution.
 *
 * @note The stop action will not execute until any ongoing timeslot state
 * locks have been lifted.
 */
void timeslot_stop(void);

/**
 * Restart the current timeslot.
 *
 * @note The restart action will not execute until any ongoing timeslot state
 * locks have been lifted.
 *
 * @param[in] priority Priority to order the new timeslot with.
 */
void timeslot_restart(timeslot_priority_t priority);

/**
 * Lock the current timeslot state, preventing the timeslot from ending from
 * user-triggered stop or restart functions. Any requests to stop or restart
 * the timeslot will be executed as soon as the state lock is lifted.
 *
 * @param[in] lock Decides whether the timeslot state is locked.
 */
void timeslot_state_lock(bool lock);

/**
 * Check whether someone has requested that the timeslot should end.
 *
 * @return Whether the timeslot has an event pending that will make it end.
 */
bool timeslot_end_is_pending(void);

/**
 * Get the timestamp for the projected end of the current timeslot.
 *
 * @return The projected end of the current timeslot, or 0 if not in a
 * timeslot.
 */
ts_timestamp_t timeslot_end_time_get(void);

/**
 * Get the expected length of the current timeslot.
 *
 * @returns The expected length of the current timeslot.
 */
ts_timestamp_t timeslot_length_get(void);

/**
 * Get the remaining time of the current timeslot.
 *
 * @return The remaining time of the current timeslot, or 0 if not in a
 * timeslot.
 */
ts_timestamp_t timeslot_remaining_time_get(void);

/**
 * Get whether the framework is currently in a timeslot.
 *
 * @return Whether the framework is currently in a timeslot.
 */
bool timeslot_is_in_ts(void);

/**
 * Get whether the framework is currently in the timeslot signal callback.
 *
 * @return Whether the framework is currently in the timeslot signal callback.
 */
bool timeslot_is_in_cb(void);

/**
 * Get whether the timeslot session is active.
 *
 * The timeslot session is considered active if the Softdevice radio session is open. It does not
 * mean that a timeslot is currently in progress.
 *
 * Changes in this state can be observed by implementing the @ref timeslot_state_listener.
 *
 * @retval true The timeslot session is active.
 * @retval false The timeslot session is not active.
 */
bool timeslot_session_is_active(void);

/**
 * Get the number of timeslots that have been started since the device started.
 *
 * @note The count is not protected from rollover, and comparisons between two timeslot count
 * numbers should consider rollovers.
 *
 * @returns The number of timeslots since we started the device.
 */
uint32_t timeslot_count_get(void);

/**
 * Trigger the timeslot signal handler.
 *
 * @note Has no effect if there's no timeslot in progress.
 */
void timeslot_trigger(void);
/** @} */

#endif /* TIMESLOT_H__ */
