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

#include "unity.h"
#include "timer_scheduler.h"
#include "timer.h"
#include "bearer_event.h"
#include "nrf_error.h"
#include "fifo.h"
#include "nrf_mesh.h"
#include "test_assert.h"

#define TIMER_MARGIN    (100)

typedef struct
{
    bearer_event_callback_t cb;
    void* p_context;
} async_evt_t;

static bool             m_async_exec;
static timestamp_t      m_last_timer_order;
static timestamp_t      m_last_timestamp;
static timer_callback_t m_timer_cb;
static timestamp_t      m_time_now;
static uint32_t         m_ret_val;
static uint32_t         m_cb_count;
static uint32_t         m_timer_stop_count;
static  bearer_event_flag_callback_t m_flag_cb;

void setUp(void)
{
    m_time_now = 0;
    m_cb_count = 0;
    m_timer_stop_count = 0;
    m_ret_val = NRF_SUCCESS;
    m_last_timestamp = 0xFFFFFFFF;
    m_last_timer_order = 0xFFFFFFFF;
    m_async_exec = false;
    timer_sch_init();
}

void tearDown(void)
{

}

/********************************/

static void timer_sch_cb(timestamp_t timestamp, void * p_context)
{
    m_last_timestamp = timestamp;
    m_cb_count++;
}

static void exec_async(void)
{
    m_flag_cb();
}

static void timer_callback_call_abort(timestamp_t timestamp, void * p_context)
{
    TEST_ASSERT_NOT_NULL(p_context);
    timer_sch_abort((timer_event_t*) p_context);
}

static void timer_callback_call_reschedule(timestamp_t timestamp, void * p_context)
{
    TEST_ASSERT_NOT_NULL(p_context);
    timer_sch_reschedule((timer_event_t*) p_context, timestamp + 4000);
}

static void timer_callback_call_reschedule_earlier(timestamp_t timestamp, void * p_context)
{
    TEST_ASSERT_NOT_NULL(p_context);
    ((timer_event_t *) p_context)->cb = timer_sch_cb; /* change callback to avoid infinite recursion */
    timer_sch_reschedule((timer_event_t*) p_context, timestamp - 4000);
}

static void timer_callback_call_schedule(timestamp_t timestamp, void * p_context)
{
    TEST_ASSERT_NOT_NULL(p_context);
    ((timer_event_t *) p_context)->timestamp += 4000;
    timer_sch_schedule((timer_event_t *) p_context);
}

static bool event_is_in_loop(timer_event_t * p_evt)
{
    for (uint32_t i = 0; i < 1000; i++)
    {
        if (p_evt == NULL)
        {
            return false;
        }
        p_evt = p_evt->p_next;
    }
    return true;
}
/********************************/
void timer_init(void)
{
}

void timer_stop(void)
{
    m_timer_stop_count++;
}

void timer_start(timestamp_t timestamp, timer_callback_t cb)
{
    m_last_timer_order = timestamp;
    m_timer_cb = cb;
}

timestamp_t timer_now(void)
{
    return m_time_now;
}

uint32_t bearer_event_flag_add(bearer_event_flag_callback_t callback)
{
    TEST_ASSERT_NOT_NULL(callback);
    m_flag_cb = callback;
    return 0;
}

bool bearer_event_in_correct_irq_priority(void)
{
    return true;
}

void bearer_event_flag_set(uint32_t flag)
{
    TEST_ASSERT_EQUAL(0, flag);
    if (!m_async_exec)
    {
        m_flag_cb();
    }
}
/**********************************/

void test_timer_sch_add(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(timer_sch_schedule(NULL));
    timer_event_t evts[15];
    for (uint32_t i = 0; i < 15; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 0;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
    }

    /* single */
    timer_sch_schedule(&evts[0]);
    TEST_ASSERT_EQUAL(1000, m_last_timer_order);
    TEST_ASSERT_EQUAL(0, m_cb_count);
    m_time_now = 1000;

    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(1000, m_last_timestamp);

    /* multiple */
    timer_sch_schedule(&evts[1]);
    timer_sch_schedule(&evts[2]);
    timer_sch_schedule(&evts[3]);

    TEST_ASSERT_EQUAL(1, m_cb_count);
    m_time_now = 2000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 3000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(3, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 4000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(4, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    /* wrong order */
    timer_sch_schedule(&evts[5]);
    timer_sch_schedule(&evts[7]);
    timer_sch_schedule(&evts[4]);
    timer_sch_schedule(&evts[6]);

    TEST_ASSERT_EQUAL(4, m_cb_count);
    m_time_now = 5000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(5, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 6000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(6, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 7000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(7, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 8000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(8, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    m_time_now = 8500;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(8, m_cb_count); /* unchanged */
    TEST_ASSERT_EQUAL(8000, m_last_timestamp);


    /* multifire */
    timer_sch_schedule(&evts[8]);
    timer_sch_schedule(&evts[9]);
    timer_sch_schedule(&evts[10]);

    TEST_ASSERT_EQUAL(8, m_cb_count);
    m_time_now = 11000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(11, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    /* too late */
    timer_sch_schedule(&evts[11]);
    TEST_ASSERT_EQUAL(11, m_cb_count);
    timer_sch_schedule(&evts[0]);
    m_time_now += TIMER_MARGIN;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(12, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    m_time_now = 12000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(13, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    /* Test events overflow */
    for (uint32_t i = 0; i < UINT16_MAX + 1; i++)
    {
        timer_sch_reschedule(&evts[0], m_time_now + ((i + 1) *2));
    }
}

void test_timer_sch_abort(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(timer_sch_abort(NULL));
    timer_event_t evts[13];
    for (uint32_t i = 0; i < 13; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 0;
        evts[i].p_next = NULL;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
    }

    /* not found */
    timer_sch_abort(&evts[0]);

    /* single head */
    timer_sch_schedule(&evts[0]);
    timer_sch_abort(&evts[0]);
    m_time_now = 1000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(0, m_cb_count);

    /* head with follower */
    timer_sch_schedule(&evts[1]);
    timer_sch_schedule(&evts[2]);
    timer_sch_abort(&evts[1]);
    m_time_now = 2000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(0, m_cb_count);
    m_time_now = 3000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count);

    /* not head */
    timer_sch_schedule(&evts[3]);
    timer_sch_schedule(&evts[4]);
    timer_sch_schedule(&evts[5]);
    timer_sch_abort(&evts[4]);
    m_time_now = 4000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(2, m_cb_count);
    m_time_now = 5000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(2, m_cb_count);
    m_time_now = 6000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(3, m_cb_count);

    /* last */
    timer_sch_schedule(&evts[6]);
    timer_sch_schedule(&evts[7]);
    timer_sch_abort(&evts[7]);
    TEST_ASSERT_EQUAL(NULL, evts[6].p_next);
    m_time_now = 7000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(4, m_cb_count);
    m_time_now = 8000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(4, m_cb_count); /* unchanged */

    /* number N for N > 1 */
    timer_sch_schedule(&evts[8]);
    timer_sch_schedule(&evts[9]);
    timer_sch_schedule(&evts[10]);
    timer_sch_schedule(&evts[11]);
    timer_sch_abort(&evts[10]);
    m_time_now = 9000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(5, m_cb_count);
    m_time_now = 10000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(6, m_cb_count);
    m_time_now = 11000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(6, m_cb_count); /* unchanged */
    m_time_now = 12000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(7, m_cb_count);
}

void test_timer_sch_reschedule(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(timer_sch_reschedule(NULL, 0));
    timer_event_t evts[15];
    for (uint32_t i = 0; i < 15; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 0;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
    }

    /* reschedule single */
    timer_sch_schedule(&evts[0]);
    timer_sch_reschedule(&evts[0], 1500);

    m_time_now = 1000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(0, m_cb_count);
    m_time_now = 1500;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, evts[0].state);

    m_cb_count = 0;
    /* reschedule unscheduled */
    timer_sch_reschedule(&evts[0], 1600);
    m_time_now = 1600;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, evts[0].state);

    /* reschedule unscheduled, should have fired already. */
    timer_sch_reschedule(&evts[0], 1400);
    m_time_now = 1700;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, evts[0].state);

    m_cb_count = 0;
    /* reschedule, change order (later) */
    timer_sch_schedule(&evts[1]);
    timer_sch_schedule(&evts[2]);
    timer_sch_reschedule(&evts[1], 3500); /* go past evt[2] */
    m_time_now = 2000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(0, m_cb_count); /* no change */
    m_time_now = 3000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 3500;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    m_cb_count = 0;
    /* reschedule, change order (earlier) */
    timer_sch_schedule(&evts[3]);
    timer_sch_schedule(&evts[4]);
    timer_sch_reschedule(&evts[4], 3700); /* go before evt[3] */
    m_time_now = 3700;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 4000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    m_cb_count = 0;
    /* reschedule race condition (earlier) */
    m_async_exec = true;
    timer_sch_schedule(&evts[5]);
    timer_sch_schedule(&evts[6]);
    timer_sch_reschedule(&evts[6], 4500); /* go before evt[5] */
    exec_async();
    m_time_now = 4500;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(0, m_cb_count); /* unchanged */
    exec_async(); /* do reschedule */
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 6000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count); /* unchanged */
    exec_async(); /* execute fire */
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);

    m_cb_count = 0;
    /* reschedule race condition (later) */
    m_async_exec = true;
    timer_sch_schedule(&evts[7]);
    timer_sch_schedule(&evts[8]);
    timer_sch_reschedule(&evts[7], 9500); /* go past evt[8] */
    exec_async();
    m_time_now = 8000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(0, m_cb_count); /* unchanged */
    exec_async(); /* do reschedule */
    TEST_ASSERT_EQUAL(0, m_cb_count); /* unchanged */
    m_time_now = 9000;
    m_timer_cb(m_time_now);
    exec_async(); /* execute fire */
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    m_time_now = 9500;
    m_timer_cb(m_time_now);
    exec_async(); /* execute fire */
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
}

void test_timer_sch_periodic(void)
{
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 1100;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
    }
    timer_sch_schedule(&evts[0]);
    TEST_ASSERT_EQUAL(1000, m_last_timer_order);
    m_time_now = 1000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    TEST_ASSERT_EQUAL(2100, evts[0].timestamp);
    m_time_now = 2100;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    TEST_ASSERT_EQUAL(3200, evts[0].timestamp);

    timer_sch_schedule(&evts[2]);
    m_time_now = 3000;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(3, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    TEST_ASSERT_EQUAL(4100, evts[2].timestamp);

    m_time_now = 3200;
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(4, m_cb_count);
    TEST_ASSERT_EQUAL(m_time_now, m_last_timestamp);
    TEST_ASSERT_EQUAL(4300, evts[0].timestamp);
}

void test_abort_self_from_callback(void)
{
    /* Lots of recursion here if we execute the calls inline. On target, calling
     * bearer_event_flag_set() from bearer_event context will trigger the call later, so we'll set
     * async execution to mimic this behavior: */
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
    }

    /* Make the timer call abort on itself from its own callback */
    evts[1].cb = timer_callback_call_abort;
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }

    m_cb_count = 0;
    for (uint32_t i = 0; i < 3; i++)
    {
        m_time_now = (i + 1) * 1000;
        m_timer_cb(m_time_now);
        exec_async();
    }
    TEST_ASSERT_FALSE(event_is_in_loop(&evts[1]));
    TEST_ASSERT_EQUAL(2, m_cb_count);
    /* evts[1] should have aborted itself successfully: */
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, evts[1].state);
    /* ensure that it's no longer in the list: */
    evts[1].cb = timer_sch_cb;
    for (uint32_t i = 0; i < 3; i++)
    {
        m_time_now = 10000 + (i + 1) * 1000;
        m_timer_cb(m_time_now);
        exec_async();
    }
    TEST_ASSERT_EQUAL(4, m_cb_count);

}

void test_abort_other_from_callback(void)
{
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
    }

    /* Abort other timers from inside the callback */
    evts[1].cb = timer_callback_call_abort;
    evts[1].p_context = &evts[2];
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }

    m_cb_count = 0;
    for (uint32_t i = 0; i < 3; i++)
    {
        m_time_now = (i + 1) * 1000;
        m_timer_cb(m_time_now);
        exec_async();
    }
    TEST_ASSERT_FALSE(event_is_in_loop(&evts[1]));
    TEST_ASSERT_EQUAL(1, m_cb_count); /* evt 2 never got to fire */
    /* evts[2] should have been aborted successfully: */
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, evts[2].state);

    /* schedule evts[2] again, but make it fire right after evts[1] (in the same callback context) */
    evts[2].timestamp = evts[1].timestamp + 1;
    timer_sch_schedule(&evts[2]);
    exec_async();
    m_cb_count = 0;
    m_time_now = evts[1].timestamp;
    m_timer_cb(m_time_now);
    exec_async();
    TEST_ASSERT_EQUAL(1, m_cb_count); /* evt 2 never got to fire, as we aborted it right before */
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, evts[2].state);

    /* schedule evts[2] again, but make it fire right BEFORE evts[1] (in the same callback context) */
    evts[2].timestamp = evts[1].timestamp - 1;
    timer_sch_schedule(&evts[2]);
    exec_async();
    m_cb_count = 0;
    m_time_now = evts[1].timestamp;
    m_timer_cb(m_time_now);
    exec_async();
    TEST_ASSERT_EQUAL(2, m_cb_count); /* evt 2 got to fire, as we didn't abort it in time.  */
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, evts[2].state); /* its next interval was cancelled though. */
    TEST_ASSERT_FALSE(event_is_in_loop(&evts[2]));
}

void test_reschedule_self_from_callback(void)
{
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
    }

    /* Make the timer call reschedule on itself from its own callback */
    evts[1].cb = timer_callback_call_reschedule;
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    m_cb_count = 0;
    for (uint32_t i = 0; i < 3; i++)
    {
        m_time_now = (i + 1) * 1000;
        m_timer_cb(m_time_now);
        exec_async();
    }
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[1].state);
    TEST_ASSERT_EQUAL(6000, evts[1].timestamp);
    TEST_ASSERT_FALSE(event_is_in_loop(&evts[1]));
}

void test_reschedule_self_earlier_from_callback(void)
{
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
    }

    /* Make the timer call reschedule on itself to an earlier time from its own callback */
    evts[1].cb = timer_callback_call_reschedule_earlier;
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    m_cb_count = 0;
    /* the first timer will fire normally */
    m_time_now = 1000;
    m_timer_cb(m_time_now);
    exec_async();
    /* the second timer will fire, reschedule itself to an earlier time, fire again with the earlier
     * time, and add its own interval, ending up at T = 2000 - 4000 + 10000 = 8000 */
    m_time_now = 2000;
    m_timer_cb(m_time_now);
    exec_async();
    /* The third timer will fire normally */
    m_time_now = 3000;
    m_timer_cb(m_time_now);
    exec_async();
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[1].state); /* ready for next interval */
    TEST_ASSERT_EQUAL(3, m_cb_count); /* the "earlier" call was to the normal callback */
    TEST_ASSERT_EQUAL(8000, evts[1].timestamp);
    TEST_ASSERT_FALSE(event_is_in_loop(&evts[1]));
}

void test_reschedule_other_from_callback(void)
{
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
    }

    /* Make the timer reschedule an other event from its callback */
    evts[1].cb = timer_callback_call_reschedule;
    evts[1].p_context = &evts[2];
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    m_cb_count = 0;
    for (uint32_t i = 0; i < 3; i++)
    {
        m_time_now = (i + 1) * 1000;
        m_timer_cb(m_time_now);
        exec_async();
    }
    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[2].state);
    TEST_ASSERT_EQUAL(6000, evts[2].timestamp);
    TEST_ASSERT_FALSE(event_is_in_loop(&evts[1]));
    TEST_ASSERT_FALSE(event_is_in_loop(&evts[2]));
}

void test_reschedule_other_earlier_from_callback(void)
{
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
    }

    /* Make the timer reschedule an other event to an earlier time from its own callback */
    evts[1].cb = timer_callback_call_reschedule_earlier;
    evts[1].p_context = &evts[2];
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    m_cb_count = 0;
    /* the first timer will fire normally */
    m_time_now = 1000;
    m_timer_cb(m_time_now);
    exec_async();
    /* the second timer will fire, reschedule the third timer to an earlier time, fire that timer
     * with the earlier time, and add its interval, ending up at T = 2000 - 4000 + 10000 = 8000 for
     * evt[2]. */
    m_time_now = 2000;
    m_timer_cb(m_time_now);
    exec_async();
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[1].state); /* ready for next interval */
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[2].state); /* ready for next interval */
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(12000, evts[1].timestamp); /* normal */
    TEST_ASSERT_EQUAL(8000, evts[2].timestamp);
    /* firing at evt[2]'s original timestamp doesn't do anything: */
    m_time_now = 3000;
    m_timer_cb(m_time_now);
    exec_async();
    TEST_ASSERT_EQUAL(2, m_cb_count); /* unchanged */
}

void test_schedule_self_from_callback(void)
{
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
    }

    /* Make the timer call schedule on itself from its own callback */
    evts[1].cb = timer_callback_call_reschedule;
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
    }
    m_cb_count = 0;
    for (uint32_t i = 0; i < 3; i++)
    {
        m_time_now = (i + 1) * 1000;
        m_timer_cb(m_time_now);
        exec_async();
    }
    TEST_ASSERT_EQUAL(2, m_cb_count);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[1].state);
    TEST_ASSERT_EQUAL(6000, evts[1].timestamp);
}

void test_schedule_other_from_callback(void)
{
    m_async_exec = true;
    timer_event_t evts[3];
    for (uint32_t i = 0; i < 3; i++)
    {
        evts[i].cb = timer_sch_cb;
        evts[i].timestamp = (i + 1) * 1000;
        evts[i].interval = 10000;
        evts[i].state = TIMER_EVENT_STATE_UNUSED;
        evts[i].p_context = &evts[i];
        TEST_ASSERT_FALSE(timer_sch_is_scheduled(&evts[i]));
    }

    /* Make the timer schedule an other event from its callback */
    evts[1].cb = timer_callback_call_schedule;
    evts[1].p_context = &evts[2];
    for (uint32_t i = 0; i < 3; i++)
    {
        timer_sch_schedule(&evts[i]);
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
        TEST_ASSERT_TRUE(timer_sch_is_scheduled(&evts[i]));
    }
    exec_async();
    for (uint32_t i = 0; i < 3; i++)
    {
        TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[i].state);
        TEST_ASSERT_TRUE(timer_sch_is_scheduled(&evts[i]));
    }
    m_cb_count = 0;
    /* the first timer will fire normally */
    m_time_now = 1000;
    m_timer_cb(m_time_now);
    exec_async();
    /* In the second, we'll attempt to schedule an event that's already scheduled. This causes a
     * hardfault when attempting to add it to the add-list. */
    m_time_now = 2000;
    m_timer_cb(m_time_now);
    TEST_NRF_MESH_ASSERT_EXPECT(exec_async());

    TEST_ASSERT_EQUAL(1, m_cb_count);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_ADDED, evts[2].state); /* still queued. */
}

void test_timer_sch_stop(void)
{
    timer_event_t evt =
    {
        .cb = timer_sch_cb,
        .timestamp = 1000,
        .interval = 0,
        .state = TIMER_EVENT_STATE_UNUSED
    };

    /* single */
    timer_sch_schedule(&evt);
    TEST_ASSERT_EQUAL(1000, m_last_timer_order);
    TEST_ASSERT_EQUAL(0, m_cb_count);
    m_time_now = 1000;

    timer_sch_stop();
    m_timer_cb(m_time_now);
    TEST_ASSERT_EQUAL(0, m_cb_count);
    TEST_ASSERT_EQUAL(2, m_timer_stop_count);
}
