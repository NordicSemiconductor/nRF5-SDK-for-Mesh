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
#include "bearer_handler.h"
#include "queue.h"
#include "nrf_mesh_assert.h"
#include "timeslot.h"
#include "toolchain.h"
#include "nrf.h"
#include "scanner.h"
#include "debug_pins.h"
/*****************************************************************************
* Local defines
*****************************************************************************/
/** Estimated maximum runtime of the bearer handler end-of-action post processing. */
#define BEARER_ACTION_POST_PROCESS_TIME_US      (100)
/** Minimum time window required for the scanner to start. */
#define BEARER_SCANNER_MIN_TIME_US              (500)
/*****************************************************************************
* Static globals
*****************************************************************************/
static queue_t          m_action_queue; /**< Queue of actions to execute. */
static volatile bool    m_stopped;      /**< Current state of the bearer handler. */
static bearer_action_t* mp_action;      /**< Ongoing bearer action. */
static timestamp_t      m_end_time;     /**< Latest end time for the ongoing action. */
static bool             m_scanner_is_active;
/*****************************************************************************
* Static functions
*****************************************************************************/

static inline bool action_in_progress(void)
{
    return (queue_peek(&m_action_queue) != NULL || mp_action);
}

static void scanner_start(void)
{
    if (!m_scanner_is_active)
    {
        DEBUG_PIN_BEARER_HANDLER_ON(DEBUG_PIN_BEARER_HANDLER_SCANNER);
        m_scanner_is_active = true;
        scanner_radio_start();
    }
}

static void scanner_stop(void)
{
    if (m_scanner_is_active)
    {
        DEBUG_PIN_BEARER_HANDLER_OFF(DEBUG_PIN_BEARER_HANDLER_SCANNER);

        DEBUG_PIN_BEARER_HANDLER_ON(DEBUG_PIN_BEARER_HANDLER_SCANNER_STOP);
        scanner_radio_stop();
        m_scanner_is_active = false;
        DEBUG_PIN_BEARER_HANDLER_OFF(DEBUG_PIN_BEARER_HANDLER_SCANNER_STOP);
    }
}

static void action_start(bearer_action_t* p_action)
{
    timeslot_state_lock(true);
    mp_action = p_action;
    /* Clear radio state before entering */
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    (void) NVIC_ClearPendingIRQ(RADIO_IRQn);
    const timestamp_t time_now = timer_now();
    m_end_time = time_now + mp_action->duration_us;
#ifdef BEARER_HANDLER_DEBUG
    mp_action->debug.event_count++;
#endif
    DEBUG_PIN_BEARER_HANDLER_ON(DEBUG_PIN_BEARER_HANDLER_ACTION);
    mp_action->start_cb(time_now, mp_action->p_args);
}

static void action_switch(void)
{
    if (!timeslot_end_is_pending())
    {
        const queue_elem_t* p_elem = queue_peek(&m_action_queue);
        bearer_action_t* p_action = NULL;
        if (p_elem != NULL)
        {
            p_action = p_elem->p_data;
        }

        const timestamp_t available_time = timeslot_remaining_time_get();

        if (p_action && (p_action->duration_us + BEARER_ACTION_POST_PROCESS_TIME_US) < available_time)
        {
            scanner_stop();

            NRF_MESH_ASSERT(queue_pop(&m_action_queue) == p_elem);
            p_action->queue_elem.p_data = NULL;

            action_start(p_action);
        }
        else if (available_time > BEARER_SCANNER_MIN_TIME_US)
        {
            scanner_start();
        }
    }
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void bearer_handler_init(void)
{
    queue_init(&m_action_queue);
    mp_action = NULL;
    m_scanner_is_active = false;
    m_stopped = true;
}

uint32_t bearer_handler_start(void)
{
    uint32_t status = NRF_SUCCESS;
    if (m_stopped)
    {
        m_stopped = false;
        timeslot_trigger();
    }
    else
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    return status;
}

uint32_t bearer_handler_stop(void)
{
    uint32_t status = NRF_SUCCESS;
    if (m_stopped)
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    else
    {
        /* Will stop the timeslot when the current action ends. */
        m_stopped = true;
        timeslot_trigger();
    }
    return status;
}

uint32_t bearer_handler_action_enqueue(bearer_action_t* p_action)
{
    NRF_MESH_ASSERT(p_action != NULL);
    NRF_MESH_ASSERT(p_action->start_cb != NULL);
    NRF_MESH_ASSERT(p_action->duration_us != 0);
    NRF_MESH_ASSERT(p_action->duration_us <= BEARER_ACTION_DURATION_MAX_US);

    uint32_t status;

    /* We always set the queue element data pointer to point to the action
       itself, then set it to NULL when popping the event. If it's already
       pointing to the action here, it has already been enqueued, and adding it
       to the queue will cause a loop in the linked list. */
    if (p_action->queue_elem.p_data == p_action)
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    else
    {
        p_action->queue_elem.p_data = p_action;
        queue_push(&m_action_queue, &p_action->queue_elem);
        if (!m_stopped && mp_action == NULL)
        {
            timeslot_trigger();
        }
        status = NRF_SUCCESS;
    }
    return status;
}

uint32_t bearer_handler_action_fire(bearer_action_t* p_action)
{
    NRF_MESH_ASSERT(p_action != NULL);
    NRF_MESH_ASSERT(p_action->start_cb != NULL);
    NRF_MESH_ASSERT(p_action->duration_us != 0);
    NRF_MESH_ASSERT(p_action->duration_us <= BEARER_ACTION_DURATION_MAX_US);

    uint32_t status = NRF_SUCCESS;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (m_stopped)
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    else if (!action_in_progress() && timeslot_is_in_ts() &&
             (timeslot_remaining_time_get() >
              p_action->duration_us + BEARER_ACTION_POST_PROCESS_TIME_US))
    {
        p_action->queue_elem.p_data = p_action;
        queue_push(&m_action_queue, &p_action->queue_elem);
        timeslot_trigger();
    }
    else
    {
        status = NRF_ERROR_BUSY;
    }
    _ENABLE_IRQS(was_masked);
    return status;
}

void bearer_handler_action_end(void)
{
    NRF_MESH_ASSERT(mp_action != NULL);
    /* Ensure that we're in signal handler context: */
    NRF_MESH_ASSERT(timeslot_is_in_cb());
    /* Ensure the action didn't last too long: */
    uint32_t time_now = timer_now();
    NRF_MESH_ASSERT(TIMER_OLDER_THAN(time_now, m_end_time));

    timeslot_state_lock(false);

#ifdef BEARER_HANDLER_DEBUG
    timestamp_t start_time = m_end_time - mp_action->duration_us;
    mp_action->debug.prev_duration_us = time_now - start_time;
    mp_action->debug.prev_margin_us = m_end_time - time_now;
#endif

    DEBUG_PIN_BEARER_HANDLER_OFF(DEBUG_PIN_BEARER_HANDLER_ACTION);
    mp_action = NULL;
    if (m_stopped)
    {
        timeslot_stop();
    }
    else
    {
        action_switch();
    }
}

void bearer_handler_radio_irq_handler(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());

    if (mp_action != NULL)
    {
        NRF_MESH_ASSERT(mp_action->radio_irq_handler != NULL);
        mp_action->radio_irq_handler(mp_action->p_args);
    }
    else
    {
        NRF_MESH_ASSERT(m_scanner_is_active);
        scanner_radio_irq_handler();
    }
}

void bearer_handler_timer_irq_handler(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());

    if (mp_action == NULL)
    {
        action_switch();
    }
}

void bearer_handler_on_ts_begin(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());
    NRF_MESH_ASSERT(mp_action == NULL);

    (void) NVIC_EnableIRQ(RADIO_IRQn);

    action_switch();
}

void bearer_handler_on_ts_end(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());
    NRF_MESH_ASSERT(mp_action == NULL);

    scanner_stop();
}
