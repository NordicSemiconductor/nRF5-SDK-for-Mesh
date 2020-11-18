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
#ifndef TIMER_SCHEDULER_H__
#define TIMER_SCHEDULER_H__

#include <stdint.h>
#include <stdbool.h>

#include "timer.h"


/**
 * @defgroup TIMER_SCHEDULER Asynchronous event scheduler
 * @ingroup MESH_CORE
 * Scalable event scheduling on the high frequency timer.
 * @{
 */

/** Constant to use in the interval field of @ref timer_event_t for single-shot timers. */
#define TIMER_EVENT_INTERVAL_SINGLE_SHOT    (0)

/**
 * Function type for the generic scheduler timeout callback.
 * @param[in] timestamp Timestamp when the timeout triggered.
 * @param[in] p_context Pointer to the data provided when starting the timer. */
typedef void (*timer_sch_callback_t)(timestamp_t timestamp, void * p_context);

/** State of a timer. */
typedef enum
{
    TIMER_EVENT_STATE_UNUSED,      /**< Not present in the scheduler */
    TIMER_EVENT_STATE_ADDED,       /**< Added for processing */
    TIMER_EVENT_STATE_IN_CALLBACK  /**< Currently being called */
} timer_event_state_t;

/**
 * Timer event structure for schedulable timers.
 */
typedef struct timer_event
{
    volatile timer_event_state_t state;     /**< Timer event state. */
    timestamp_t                  timestamp; /**< Timestamp at which to fire. Is updated by the scheduler if periodic.  */
    timer_sch_callback_t         cb;        /**< Callback function to call when the timer fires. Called asynchronously. */
    uint32_t                     interval;  /**< Interval in us between each fire for periodic timers, or 0 if single-shot. */
    void *                       p_context; /**< Pointer to data passed on to the callback. */
    struct timer_event*          p_next;    /**< Pointer to next event in linked list. Only for internal usage. */
} timer_event_t;

/**
 * Initializes the scheduler module.
 */
void timer_sch_init(void);

/**
 * Schedules a timer event.
 *
 * @warning This function must be called from the configured mesh IRQ level.
 *
 * @warning The structure parameters must not be changed after the structure has been added to the
 *          scheduler, as this may cause race conditions. If a change in timing is needed, please use the
 *          @ref timer_sch_reschedule() function. If any other changes are needed, abort the event, change the
 *          parameter, and schedule it again.
 *
 * @param[in] p_timer_evt A pointer to a statically allocated timer event, which will be used as
 *                        context for the schedulable event.
 */
void timer_sch_schedule(timer_event_t* p_timer_evt);

/**
 * Aborts a previously scheduled event.
 *
 * @warning This function must be called from the configured mesh IRQ level.
 *
 * @param[in] p_timer_evt A pointer to a previously scheduled event.
 */
void timer_sch_abort(timer_event_t* p_timer_evt);

/**
 * Reschedules a previously scheduled event.
 *
 * @warning This function must be called from the configured mesh IRQ level.
 *
 * @param[in] p_timer_evt   A pointer to a previously scheduled event.
 * @param[in] new_timestamp When the event should time out, instead of the old time.
 */
void timer_sch_reschedule(timer_event_t* p_timer_evt, timestamp_t new_timestamp);

/**
 * Checks if a timer event is scheduled.
 *
 * @param[in] p_timer_evt Timer scheduler event context pointer.
 *
 * @returns @c true if the event is active, @c false otherwise.
 */
bool timer_sch_is_scheduled(const timer_event_t * p_timer_evt);

/**
 * The function causes timer scheduler to abandon the currently scheduled timers.
 * Additionally, it stops the hardware timer.
 *
 * @warning The user application should not call @ref timer_sch_reschedule() or
 *          @ref timer_sch_schedule() APIs after this point.
 */
void timer_sch_stop(void);

/** @} */

#endif /* TIMER_SCHEDULER_H__ */
