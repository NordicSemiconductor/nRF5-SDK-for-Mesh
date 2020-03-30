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
#ifndef BEARER_EVENT_H__
#define BEARER_EVENT_H__

#include <stdint.h>

#include "timer.h"
#include "timer_scheduler.h"
#include "nrf_mesh_config_core.h"
#include "queue.h"

/**
 * @defgroup BEARER_EVENT Event handler for bearer layer
 * @ingroup MESH_CORE
 * Schedules bearer events for asynchronous processing in configured IRQ priority level.
 * @{
 */

/** Invalid bearer event flag, for indicating a flag that hasn't been added. */
#define BEARER_EVENT_FLAG_INVALID   0xFFFFFFFF

/** Callback function type for generic event processing. */
typedef void(*bearer_event_callback_t)(void* p_context);
/** Callback function type for flag events. */
typedef bool(*bearer_event_flag_callback_t)(void);

/** Bearer event flag type. */
typedef uint32_t bearer_event_flag_t;

/** Bearer event sequential type. */
typedef struct
{
    queue_elem_t queue_elem; /**< Used for linked list operation, should not be altered by the user. */
    bearer_event_callback_t callback; /**< Callback function to be called. */
    void * p_context; /**< Pointer to a context variable to be passed to the callback. */
    volatile bool event_pending; /**< Flag for indicating if an event is currently pending. */
} bearer_event_sequential_t;

/**
 * Initialize the bearer event module.
 *
 * @param[in] irq_priority Bearer event IRQ priority (NRF_MESH_IRQ_PRIORITY_THREAD if thread mode).
 */
void bearer_event_init(uint8_t irq_priority);

/**
 * Start the bearer event module.
 *
 * @note Enables bearer event IRQ if IRQ priority level is not NRF_MESH_IRQ_PRIORITY_THREAD.
 */
void bearer_event_start(void);

/**
 * Post generic event for asynchronous processing.
 * @param[in] callback Callback function to call.
 * @param[in] p_context Pointer to a context variable to be passed to the callback.
 *
 * @retval NRF_SUCCESS The event was successfully posted for processing.
 * @retval NRF_ERROR_NO_MEM The event fifo was full, and the event will not be processed.
 */
uint32_t bearer_event_generic_post(bearer_event_callback_t callback, void* p_context);

/**
 * Post timer scheduler type event for asynchronous processing.
 *
 * @param[in] callback Callback function to call.
 * @param[in] timestamp Expiration timestamp to give to callback function.
 * @param[in] p_context Context pointer given in the callback.
 *
 * @retval NRF_SUCCESS The event was successfully posted for processing.
 * @retval NRF_ERROR_NO_MEM The event fifo was full, and the event will not be processed.
 */
uint32_t bearer_event_timer_sch_post(timer_sch_callback_t callback, timestamp_t timestamp, void * p_context);

/**
 * Prevent the event handler from firing. Start of critical section.
 */
void bearer_event_critical_section_begin(void);

/**
 * Prevent the event handler from firing. End of critical section.
 */
void bearer_event_critical_section_end(void);

/**
 * Add a bearer_event flag callback.
 *
 * @note Will assert if there are no more flags available for allocation. If
 * this happens, increase @ref BEARER_EVENT_FLAG_COUNT appropriately.
 *
 * @param[in] callback Callback function pointer that will be called every time
 * the returned flag is set.
 *
 * @returns A flag that can be referenced in @ref bearer_event_flag_set to trigger the given callback.
 */
bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t callback);

/**
 * Set the given event flag, triggering the corresponding flag callback as soon as possible.
 *
 * @note Will assert if the given flag doesn't have a callback.
 *
 * @param[in] flag Flag to trigger.
 */
void bearer_event_flag_set(bearer_event_flag_t flag);

/**
 * Add a sequential bearer event object.
 *
 * This event type guarantees that a new event can not be posted before the previous event on this
 * event object has been processed (i.e. the callback function has been executed).
 *
 * @param[out] p_seq Sequential event handle.
 * @param[in] callback Callback function pointer that will be called every time the event is posted.
 * @param[in] p_context Pointer to a context variable to be passed to the callback.
 */
void bearer_event_sequential_add(bearer_event_sequential_t * p_seq, bearer_event_callback_t callback, void * p_context);

/**
 * Post a sequential bearer event.
 *
 * @note Any event data must be written so that it is available through the context pointer that was
 *       passed to bearer_event_sequential_add() before calling bearer_event_sequential_post().
 *
 * @param[in] p_seq Sequential event handle.
 *
 * @retval NRF_SUCCESS The event was successfully posted for processing.
 * @retval NRF_ERROR_BUSY An event is already pending.
 */
uint32_t bearer_event_sequential_post(bearer_event_sequential_t * p_seq);

/**
 * Check if a sequential bearer event is pending.
 *
 * @param[in] p_seq Sequential event handle.
 *
 * @retval true Event is pending.
 * @retval false Event is not pending.
 */
bool bearer_event_sequential_pending(bearer_event_sequential_t * p_seq);

/**
 * Handle pending bearer events.
 *
 * @retval true Handling is done, i.e. no more events are pending.
 * @retval false Handling is not done, i.e. events are still pending.
 */
bool bearer_event_handler(void);

/**
 * Check whether the processor is currently executing in the bearer event IRQ priority.
 *
 * @returns Whether the processor is currently executing in the bearer event IRQ priority.
 */
bool bearer_event_in_correct_irq_priority(void);

/** @} */

#endif /* BEARER_EVENT_H__ */
