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
#include <string.h>
#include "timer_scheduler.h"
#include "nrf_error.h"
#include "nrf_mesh_assert.h"
#include "bearer_event.h"
#include "toolchain.h"

/** Time in us to regard as immediate when firing several timers at once */
#define TIMER_MARGIN    (100)

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef struct
{
    timer_event_t * p_head;
    uint16_t event_count;
} scheduler_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static volatile scheduler_t m_scheduler; /**< Global scheduler instance */
static bearer_event_flag_t m_event_flag;
static bool m_is_power_down_triggered;

/*****************************************************************************
* Static functions
*****************************************************************************/
static void timer_cb(timestamp_t timestamp)
{
    bearer_event_flag_set(m_event_flag);
}

static void add_evt(timer_event_t* p_evt)
{
    if (m_scheduler.p_head == NULL ||
        TIMER_OLDER_THAN(p_evt->timestamp, m_scheduler.p_head->timestamp))
    {
        p_evt->p_next = m_scheduler.p_head;
        m_scheduler.p_head = p_evt;
    }
    else
    {
        timer_event_t* p_temp = m_scheduler.p_head;
        while (p_temp->p_next &&
            TIMER_OLDER_THAN(p_temp->p_next->timestamp, p_evt->timestamp))
        {
            p_temp = p_temp->p_next;
        }

        p_evt->p_next = p_temp->p_next;
        p_temp->p_next = p_evt;
    }

    NRF_MESH_ASSERT(++m_scheduler.event_count > 0);
    p_evt->state = TIMER_EVENT_STATE_ADDED;
}

static void remove_evt(timer_event_t * p_evt)
{
    if (p_evt == m_scheduler.p_head)
    {
        m_scheduler.p_head = p_evt->p_next;
        NRF_MESH_ASSERT(m_scheduler.event_count-- > 0);
    }
    else
    {
        timer_event_t * p_it = m_scheduler.p_head;
        while (p_it != NULL)
        {
            if (p_it->p_next == p_evt)
            {
                p_it->p_next = p_evt->p_next;
                NRF_MESH_ASSERT(m_scheduler.event_count-- > 0);
                break;
            }
            p_it = p_it->p_next;
        }
    }

    p_evt->state = TIMER_EVENT_STATE_UNUSED;
}

static void fire_timers(timestamp_t time_now)
{
    while (m_scheduler.p_head &&
            TIMER_OLDER_THAN(m_scheduler.p_head->timestamp, time_now + TIMER_MARGIN))
    {
        timer_event_t* p_evt = m_scheduler.p_head;

        NRF_MESH_ASSERT(p_evt->state == TIMER_EVENT_STATE_ADDED);

        /* iterate */
        m_scheduler.p_head = p_evt->p_next;
        p_evt->p_next = NULL;
        NRF_MESH_ASSERT(m_scheduler.event_count-- > 0);

        NRF_MESH_ASSERT(p_evt->cb != NULL);
        p_evt->state = TIMER_EVENT_STATE_IN_CALLBACK;

        p_evt->cb(time_now, p_evt->p_context);

        /* Re-sample the time to avoid lagging behind after long running timer callbacks. */
        time_now = timer_now();

        /* Only re-add the event if it wasn't added back in the callback. */
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

                add_evt(p_evt);
            }
        }
    }
}

static void setup_timeout(timestamp_t time_now)
{
    if (m_scheduler.p_head != NULL)
    {
        timer_start(m_scheduler.p_head->timestamp, timer_cb);
    }
    else
    {
        timer_stop();
    }
}

static bool flag_event_cb(void)
{
    fire_timers(timer_now());
    setup_timeout(timer_now());
    return true;
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void timer_sch_init(void)
{
    m_is_power_down_triggered = false;
    memset((scheduler_t*) &m_scheduler, 0, sizeof(m_scheduler));
    m_event_flag = bearer_event_flag_add(flag_event_cb);
    timer_init();
}

void timer_sch_schedule(timer_event_t* p_timer_evt)
{
    NRF_MESH_ASSERT_DEBUG(!m_is_power_down_triggered);
    NRF_MESH_ASSERT_DEBUG(bearer_event_in_correct_irq_priority());
    NRF_MESH_ASSERT(p_timer_evt != NULL);
    NRF_MESH_ASSERT(p_timer_evt->cb != NULL);
    NRF_MESH_ASSERT(p_timer_evt->state != TIMER_EVENT_STATE_ADDED);

    p_timer_evt->p_next = NULL;
    add_evt(p_timer_evt);
    setup_timeout(timer_now());
}

void timer_sch_abort(timer_event_t* p_timer_evt)
{
    NRF_MESH_ASSERT_DEBUG(bearer_event_in_correct_irq_priority());
    NRF_MESH_ASSERT(p_timer_evt != NULL);
    remove_evt(p_timer_evt);
    p_timer_evt->p_next = NULL;
    setup_timeout(timer_now());
}

void timer_sch_reschedule(timer_event_t* p_timer_evt, timestamp_t new_timeout)
{
    NRF_MESH_ASSERT_DEBUG(!m_is_power_down_triggered);
    NRF_MESH_ASSERT_DEBUG(bearer_event_in_correct_irq_priority());
    NRF_MESH_ASSERT(p_timer_evt != NULL);
    remove_evt(p_timer_evt);
    p_timer_evt->p_next = NULL;
    p_timer_evt->timestamp = new_timeout;
    add_evt(p_timer_evt);
    setup_timeout(timer_now());
}

bool timer_sch_is_scheduled(const timer_event_t * p_timer_evt)
{
    return (p_timer_evt->state == TIMER_EVENT_STATE_ADDED ||
            p_timer_evt->state == TIMER_EVENT_STATE_IN_CALLBACK);
}

void timer_sch_stop(void)
{
    m_is_power_down_triggered = true;
    m_scheduler.p_head = NULL;
    timer_stop();
}
