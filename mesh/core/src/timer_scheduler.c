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
#include <string.h>
#include "timer_scheduler.h"
#include "nrf_error.h"
#include "nrf_mesh_assert.h"
#include "bearer_event.h"
#include "toolchain.h"


/** Time in us to regard as immidiate when firing several timers at once */
#define TIMER_MARGIN    (100)

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef struct
{
    timer_event_t* p_head;
    timer_event_t* p_add_head; /**< List of timers waiting to be added to the fire list. */
    uint32_t dirty_events; /**< Events in the fire list that needs processing */
    uint32_t event_count;
} scheduler_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static volatile scheduler_t m_scheduler; /**< Global scheduler instance */
static bearer_event_flag_t m_event_flag;
/*****************************************************************************
* Static functions
*****************************************************************************/
static inline bool is_dirty_state(timer_event_state_t state)
{
    return (state == TIMER_EVENT_STATE_RESCHEDULED ||
            state == TIMER_EVENT_STATE_ABORTED);
}
static void timer_cb(timestamp_t timestamp)
{
    bearer_event_flag_set(m_event_flag);
}

static void add_evt(timer_event_t* p_evt)
{
    NRF_MESH_ASSERT(p_evt->p_next == NULL);

    if (m_scheduler.p_head == NULL ||
        TIMER_OLDER_THAN(p_evt->timestamp, m_scheduler.p_head->timestamp))
    {
        p_evt->p_next = m_scheduler.p_head;
        m_scheduler.p_head = p_evt;
        NRF_MESH_ASSERT(++m_scheduler.event_count > 0);
        return;
    }

    timer_event_t* p_temp = m_scheduler.p_head;
    while (p_temp->p_next &&
         TIMER_OLDER_THAN(p_temp->p_next->timestamp, p_evt->timestamp))
    {
        p_temp = p_temp->p_next;
    }

    p_evt->p_next = p_temp->p_next;
    p_temp->p_next = p_evt;
    NRF_MESH_ASSERT(++m_scheduler.event_count > 0);
}

/** Takes all events in the add list, and inserts them in the fire list. */
static void process_add_list(void)
{
    /* insert all pending events */
    while (m_scheduler.p_add_head != NULL)
    {
        timer_event_t * p_evt = (timer_event_t *) m_scheduler.p_add_head;
        m_scheduler.p_add_head = p_evt->p_next;
        p_evt->p_next = NULL;
        if (p_evt->state == TIMER_EVENT_STATE_IGNORED)
        {
            p_evt->state = TIMER_EVENT_STATE_UNUSED;
        }
        else
        {
            NRF_MESH_ASSERT(p_evt->state == TIMER_EVENT_STATE_ADDED);
            p_evt->state = TIMER_EVENT_STATE_QUEUED;
            add_evt(p_evt);
        }
    }
}

static inline void add_to_add_list(timer_event_t * p_evt)
{
    NRF_MESH_ASSERT(p_evt->state == TIMER_EVENT_STATE_UNUSED ||
                    p_evt->state == TIMER_EVENT_STATE_IN_CALLBACK);
    NRF_MESH_ASSERT(p_evt->p_next == NULL);
    p_evt->state = TIMER_EVENT_STATE_ADDED;
    p_evt->p_next = m_scheduler.p_add_head;
    m_scheduler.p_add_head = p_evt;
}

/** Run through the fire list, pop the aborted events and move the events that
 * have been rescheduled to the add-list. */
static void process_dirty_events(void)
{
    timer_event_t ** pp_evt = (timer_event_t **) &m_scheduler.p_head;
    while (*pp_evt != NULL && m_scheduler.dirty_events > 0)
    {
        timer_event_state_t state = (*pp_evt)->state;
        if (is_dirty_state(state))
        {
            timer_event_t * p_evt = *pp_evt;
            *pp_evt = p_evt->p_next; /* pop */
            p_evt->p_next = NULL;
            NRF_MESH_ASSERT(m_scheduler.event_count-- > 0);

            m_scheduler.dirty_events--;
            p_evt->state = TIMER_EVENT_STATE_UNUSED;

            if (state == TIMER_EVENT_STATE_RESCHEDULED)
            {
                add_to_add_list(p_evt);
            }
        }
        else
        {
            pp_evt = &((*pp_evt)->p_next); /* step */
        }
    }
}

static void fire_timers(timestamp_t time_now)
{
    while (m_scheduler.p_head &&
            TIMER_OLDER_THAN(m_scheduler.p_head->timestamp, time_now + TIMER_MARGIN))
    {
        timer_event_t* p_evt = m_scheduler.p_head;

        NRF_MESH_ASSERT(p_evt->state == TIMER_EVENT_STATE_QUEUED);

        /* iterate */
        m_scheduler.p_head = p_evt->p_next;
        p_evt->p_next = NULL;
        NRF_MESH_ASSERT(m_scheduler.event_count-- > 0);

        NRF_MESH_ASSERT(p_evt->cb != NULL);
        p_evt->state = TIMER_EVENT_STATE_IN_CALLBACK;

        p_evt->cb(time_now, p_evt->p_context);

        /* Re-sample the time to avoid lagging behind after long running timer callbacks. */
        time_now = timer_now();

        /* We let the user execute, so we have to check whether they've changed something: */
        process_dirty_events();
        process_add_list();

        /* Only re-add the event if it wasn't added back in in the callback. */
        if (p_evt->state == TIMER_EVENT_STATE_IN_CALLBACK)
        {
            if (p_evt->interval == 0)
            {
                p_evt->state = TIMER_EVENT_STATE_UNUSED;
            }
            else
            {
                do
                {
                    p_evt->timestamp += p_evt->interval;
                } while (TIMER_OLDER_THAN(p_evt->timestamp, time_now + TIMER_MARGIN));

                p_evt->state = TIMER_EVENT_STATE_QUEUED;
                add_evt(p_evt);
            }
        }
    }
}

static void setup_timeout(timestamp_t time_now)
{
    if (m_scheduler.p_head)
    {
        if (TIMER_OLDER_THAN(time_now + TIMER_MARGIN, m_scheduler.p_head->timestamp))
        {
            NRF_MESH_ERROR_CHECK(timer_order_cb(TIMER_INDEX_SCHEDULER, m_scheduler.p_head->timestamp, timer_cb, TIMER_ATTR_SYNCHRONOUS));
        }
        else
        {
            NRF_MESH_ERROR_CHECK(timer_order_cb(TIMER_INDEX_SCHEDULER, time_now + TIMER_MARGIN, timer_cb, TIMER_ATTR_SYNCHRONOUS));
        }
    }
}

static bool flag_event_cb(void)
{
    process_dirty_events();

    /* add all the popped events back in, at their right places. */
    process_add_list();

    fire_timers(timer_now());
    setup_timeout(timer_now());

    return true;
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void timer_sch_init(void)
{
    memset((scheduler_t*) &m_scheduler, 0, sizeof(m_scheduler));
    m_event_flag = bearer_event_flag_add(flag_event_cb);
}

void timer_sch_schedule(timer_event_t* p_timer_evt)
{
    NRF_MESH_ASSERT(p_timer_evt != NULL);
    NRF_MESH_ASSERT(p_timer_evt->cb != NULL);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    p_timer_evt->p_next = NULL;
    add_to_add_list(p_timer_evt);
    _ENABLE_IRQS(was_masked);

    bearer_event_flag_set(m_event_flag);
}

void timer_sch_abort(timer_event_t* p_timer_evt)
{
    NRF_MESH_ASSERT(p_timer_evt != NULL);
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (p_timer_evt->state == TIMER_EVENT_STATE_IN_CALLBACK)
    {
        p_timer_evt->state = TIMER_EVENT_STATE_UNUSED;
    }
    else if (p_timer_evt->state == TIMER_EVENT_STATE_ADDED)
    {
        p_timer_evt->state = TIMER_EVENT_STATE_IGNORED;
    }
    else if (p_timer_evt->state != TIMER_EVENT_STATE_UNUSED)
    {
        if (!is_dirty_state(p_timer_evt->state))
        {
            m_scheduler.dirty_events++;
        }
        p_timer_evt->state = TIMER_EVENT_STATE_ABORTED;
        bearer_event_flag_set(m_event_flag);
    }
    _ENABLE_IRQS(was_masked);
}

void timer_sch_reschedule(timer_event_t* p_timer_evt, timestamp_t new_timeout)
{
    NRF_MESH_ASSERT(p_timer_evt != NULL);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    /* The events in the added queue will reinsert themselves in the processing. */
    if (p_timer_evt->state == TIMER_EVENT_STATE_UNUSED ||
        p_timer_evt->state == TIMER_EVENT_STATE_IN_CALLBACK)
    {
        add_to_add_list(p_timer_evt);
    }
    else if (p_timer_evt->state == TIMER_EVENT_STATE_ADDED ||
             p_timer_evt->state == TIMER_EVENT_STATE_IGNORED)
    {
        p_timer_evt->state = TIMER_EVENT_STATE_ADDED;
    }
    else
    {
        /* Mark the rescheduled event as dirty, will be processed at the next opportunity. */
        if (!is_dirty_state(p_timer_evt->state))
        {
            m_scheduler.dirty_events++;
        }
        p_timer_evt->state = TIMER_EVENT_STATE_RESCHEDULED;
    }
    p_timer_evt->timestamp = new_timeout;
    bearer_event_flag_set(m_event_flag);
    _ENABLE_IRQS(was_masked);
}

