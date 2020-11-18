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
#include "bearer_handler.h"
#include "queue.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_config_bearer.h"
#include "timeslot.h"
#include "timeslot_timer.h"
#include "toolchain.h"
#include "nrf.h"
#include "scanner.h"
#include "debug_pins.h"
/*****************************************************************************
* Local defines
*****************************************************************************/
/** Minimum time window required for the scanner to start. */
#define BEARER_SCANNER_MIN_TIME_US              (500)

/*****************************************************************************
* Static globals
*****************************************************************************/
static queue_t          m_action_queue; /**< Queue of actions to execute. */
static volatile bool    m_stopped;      /**< Current state of the bearer handler. */
static bearer_action_t* mp_action;      /**< Ongoing bearer action. */
static ts_timestamp_t   m_end_time;     /**< Latest end time for the ongoing action. */
static bool             m_scanner_is_active;
static bool             m_action_ended; /**< The event end function has been called */
static bool             m_in_callback; /**< In the callback. */
static bearer_handler_stopped_cb_t m_stopped_callback; /**< Single fire stop callback to call when the bearer handler has been stopped */
static bool             m_is_in_force_mode;

/*****************************************************************************
* Static functions
*****************************************************************************/
static void action_switch(void);

static inline bool action_in_progress(void)
{
    return (queue_peek(&m_action_queue) != NULL || (mp_action && !m_action_ended));
}

static void timer_irq_clear(void)
{
    BEARER_ACTION_TIMER->TASKS_STOP = 1;
    BEARER_ACTION_TIMER->TASKS_CLEAR = 1;
    BEARER_ACTION_TIMER->INTENCLR = 0xFFFFFFFF;
    BEARER_ACTION_TIMER->SHORTS = 0;
    (void) NVIC_ClearPendingIRQ(BEARER_ACTION_TIMER_IRQn);
}

/**
 * Start the action timer.
 *
 * @returns Device timestamp at the start of the timer.
 */
static ts_timestamp_t action_timer_start(void)
{
    DEBUG_PIN_BEARER_HANDLER_ON(DEBUG_PIN_BEARER_HANDLER_TIMER_SETUP);

    timer_irq_clear();
    /* Start the action timer with the first CC set to 1 us, and make TIMER0 capture a timestamp at that time with PPI.
     * With this, we can determine the exact start time of the action timer on the TIMER0 timeline, as we can read out
     * the captured value of TIMER0 when the action timer is at T=1us. */
    BEARER_ACTION_TIMER->EVENTS_COMPARE[0] = 0;
    BEARER_ACTION_TIMER->CC[0] = 1; /* Trigger compare event at 1us */
    BEARER_ACTION_TIMER->PRESCALER = 4; /* 1 us step */
    BEARER_ACTION_TIMER->MODE = TIMER_MODE_MODE_Timer;
    BEARER_ACTION_TIMER->BITMODE = TIMER_BITMODE_BITMODE_16Bit;

    NRF_PPI->CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].EEP = (uint32_t) &BEARER_ACTION_TIMER->EVENTS_COMPARE[0];
    NRF_PPI->CH[TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO].TEP = (uint32_t) &NRF_TIMER0->TASKS_CAPTURE[TS_TIMER_INDEX_RADIO];
    NRF_PPI->CHENSET = (1UL << (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO));

    BEARER_ACTION_TIMER->TASKS_START = 1;

#ifndef HOST
    /* Wait for the compare event */
    while (BEARER_ACTION_TIMER->EVENTS_COMPARE[0] == 0);
#endif

    /* Cleanup */
    BEARER_ACTION_TIMER->EVENTS_COMPARE[0] = 0;
    NRF_PPI->CHENCLR = (1UL << (TS_TIMER_PPI_CH_START + TS_TIMER_INDEX_RADIO));

    DEBUG_PIN_BEARER_HANDLER_OFF(DEBUG_PIN_BEARER_HANDLER_TIMER_SETUP);

    return NRF_TIMER0->CC[TS_TIMER_INDEX_RADIO] - BEARER_ACTION_TIMER->CC[0];
}

static void radio_irq_clear(void)
{
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;

    (void) NVIC_ClearPendingIRQ(RADIO_IRQn);
}

static void scanner_start(void)
{
    if (!m_scanner_is_active)
    {
        radio_irq_clear();
        DEBUG_PIN_BEARER_HANDLER_ON(DEBUG_PIN_BEARER_HANDLER_SCANNER);
        m_scanner_is_active = true;
        scanner_radio_start(action_timer_start());
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


static void end_handle(void)
{
    /* Ensure the action didn't last too long: */
    ts_timestamp_t time_now = ts_timer_now();
    NRF_MESH_ASSERT(TIMER_OLDER_THAN(time_now, m_end_time));

    timeslot_state_lock(false);

#ifdef BEARER_HANDLER_DEBUG
    ts_timestamp_t start_time = m_end_time - mp_action->duration_us;
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

static void action_start(bearer_action_t* p_action)
{
    timeslot_state_lock(true);
    mp_action = p_action;
    radio_irq_clear();

    const ts_timestamp_t time_now = action_timer_start();
    m_end_time = time_now + mp_action->duration_us;

    m_action_ended = false;

#ifdef BEARER_HANDLER_DEBUG
    mp_action->debug.event_count++;
#endif
    DEBUG_PIN_BEARER_HANDLER_ON(DEBUG_PIN_BEARER_HANDLER_ACTION);

    m_in_callback = true;
    mp_action->start_cb(time_now, mp_action->p_args);
    m_in_callback = false;

    if (m_action_ended)
    {
        end_handle();
    }
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

        const ts_timestamp_t available_time = timeslot_remaining_time_get();
        const ts_timestamp_t total_time = timeslot_length_get();

        if (p_action && (p_action->duration_us + BEARER_ACTION_POST_PROCESS_TIME_US) > total_time)
        {
            /* The current timeslot doesn't have space for this action, restart the timeslot to get a longer one. */
            timeslot_restart(TIMESLOT_PRIORITY_HIGH);
        }
        else if (p_action && (p_action->duration_us + BEARER_ACTION_POST_PROCESS_TIME_US) < available_time)
        {
            scanner_stop();

            NRF_MESH_ASSERT(queue_pop(&m_action_queue) == p_elem);
            p_action->queue_elem.p_data = NULL;

            action_start(p_action);
        }
        else if (available_time > BEARER_SCANNER_MIN_TIME_US)
        {
            if (scanner_is_enabled())
            {
                scanner_start();
            }
            else
            {
                /* There's no scanner activity and no actions pending, stop the timeslot to save power. */
                if (!m_is_in_force_mode)
                {
                    timeslot_stop();
                }
            }
        }
    }
}

static void notify_stop(void)
{
    if (m_stopped_callback)
    {
        bearer_handler_stopped_cb_t stop_cb = m_stopped_callback;
        m_stopped_callback = NULL;
        stop_cb();
    }
}

void BEARER_ACTION_TIMER_IRQHandler(void)
{
    if (mp_action != NULL)
    {
        NRF_MESH_ASSERT(mp_action->timer_irq_handler != NULL);

        m_in_callback = true;
        mp_action->timer_irq_handler(mp_action->p_args);
        m_in_callback = false;

        if (m_action_ended)
        {
            end_handle();
        }
    }
    else
    {
        NRF_MESH_ASSERT(m_scanner_is_active);
        scanner_timer_irq_handler();
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
    m_in_callback = false;
    m_action_ended = false;

    NVIC_SetPriority(BEARER_ACTION_TIMER_IRQn, 0);
}

uint32_t bearer_handler_start(void)
{
    uint32_t status = NRF_SUCCESS;
    if (m_stopped)
    {
        m_stopped = false;
        bearer_handler_wake_up();
    }
    else
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    return status;
}

uint32_t bearer_handler_stop(bearer_handler_stopped_cb_t cb)
{
    uint32_t status = NRF_SUCCESS;
    if (m_stopped)
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    else
    {
        /* Will stop the timeslot when the current action ends. */
        m_stopped_callback = cb;
        m_stopped = true;
        if (timeslot_is_in_ts())
        {
            timeslot_trigger();
        }
        else if (timeslot_session_is_active())
        {
            timeslot_stop();
        }
        else
        {
            notify_stop();
        }
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
            bearer_handler_wake_up();
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

    uint32_t status;
    if (m_stopped)
    {
        status = NRF_ERROR_INVALID_STATE;
    }
    else
    {
        if (action_in_progress())
        {
            status = NRF_ERROR_BUSY;
        }
        else
        {
            p_action->queue_elem.p_data = p_action;
            queue_push(&m_action_queue, &p_action->queue_elem);
            bearer_handler_wake_up();
            status = NRF_SUCCESS;
        }
    }
    return status;
}

void bearer_handler_wake_up(void)
{
    if (!m_stopped && (scanner_is_enabled() || action_in_progress()))
    {
        if (timeslot_start() != NRF_SUCCESS)
        {
            timeslot_trigger();
        }
    }
}

void bearer_handler_action_end(void)
{
    NRF_MESH_ASSERT(mp_action != NULL);
    /* Ensure that we're in signal handler context: */
    NRF_MESH_ASSERT(timeslot_is_in_cb());
    NRF_MESH_ASSERT(!m_action_ended);

    if (m_in_callback)
    {
        m_action_ended = true;
    }
    else
    {
        end_handle();
    }
}

void bearer_handler_radio_irq_handler(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());

    if (mp_action != NULL)
    {
        NRF_MESH_ASSERT(mp_action->radio_irq_handler != NULL);

        m_in_callback = true;
        mp_action->radio_irq_handler(mp_action->p_args);
        m_in_callback = false;

        if (m_action_ended)
        {
            end_handle();
        }
    }
    else
    {
        NRF_MESH_ASSERT(m_scanner_is_active);
        scanner_radio_irq_handler();

        if (!scanner_is_enabled())
        {
            timeslot_stop();
        }
    }
}

void bearer_handler_timer_irq_handler(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());

    if (mp_action == NULL)
    {
        if (m_stopped)
        {
            timeslot_stop();
        }
        else
        {
            action_switch();
        }
    }
}

void bearer_handler_on_ts_begin(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());
    NRF_MESH_ASSERT(mp_action == NULL);

    NVIC_EnableIRQ(BEARER_ACTION_TIMER_IRQn);
    (void) NVIC_EnableIRQ(RADIO_IRQn);

    action_switch();
}

void bearer_handler_on_ts_end(void)
{
    NRF_MESH_ASSERT(timeslot_is_in_cb());
    NRF_MESH_ASSERT(mp_action == NULL);

    scanner_stop();
    timer_irq_clear();
    radio_irq_clear();
    BEARER_ACTION_TIMER->TASKS_SHUTDOWN = 1;

    NVIC_DisableIRQ(BEARER_ACTION_TIMER_IRQn);
}

void bearer_handler_on_ts_session_closed(void)
{
    if (m_stopped)
    {
        notify_stop();
    }
}

void bearer_handler_force_mode_enable(void)
{
    m_is_in_force_mode = true;
}

void bearer_handler_force_mode_disable(void)
{
    m_is_in_force_mode = false;
}
