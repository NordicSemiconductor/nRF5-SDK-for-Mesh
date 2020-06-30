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

#include "long_timer.h"

#include <stddef.h>
#include <unity.h>
#include "utils.h"

#include "timer_scheduler_mock.h"
#include "timer_mock.h"

#define TIMER_STEP  ((UINT32_MAX / 2) - 1)
#define TIME_NOW    (0x2a)

static long_timer_t m_timer;
static bool m_is_cb_expected;

void setUp(void)
{
    timer_scheduler_mock_Init();
    timer_mock_Init();
}

void tearDown(void)
{
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
}

static void lt_cb(void * p_context)
{
    TEST_ASSERT_EQUAL(m_timer.p_context, p_context);
    TEST_ASSERT_TRUE(m_is_cb_expected);
    m_is_cb_expected = false;
}

static void single_timer_schedule(uint64_t time)
{
    timer_now_ExpectAndReturn(TIME_NOW);
    timer_sch_reschedule_Expect(&m_timer.event, TIME_NOW + MIN(TIMER_STEP, time));
    lt_schedule(&m_timer, lt_cb, single_timer_schedule, time);

    TEST_ASSERT_EQUAL(m_timer.callback, lt_cb);
    TEST_ASSERT_EQUAL(m_timer.p_context, single_timer_schedule);
    TEST_ASSERT_EQUAL(m_timer.remaining_time_us, time);
    TEST_ASSERT_EQUAL(m_timer.event.p_context, &m_timer);
    TEST_ASSERT_NOT_NULL(m_timer.event.cb);
}

static void timer_handler_rounds(uint64_t rounds)
{
    uint64_t remaining_time_us = m_timer.remaining_time_us;

    while (rounds != 0)
    {
        remaining_time_us -= MIN(TIMER_STEP, remaining_time_us);

        if (rounds == 1)
        {
            m_is_cb_expected = true;
        }
        m_timer.event.cb(0, (void *)&m_timer);

        rounds--;
        TEST_ASSERT_EQUAL(remaining_time_us, m_timer.remaining_time_us);
        TEST_ASSERT_EQUAL(rounds == 0 ? 0 : MIN(TIMER_STEP, remaining_time_us), m_timer.event.interval);
    }

    TEST_ASSERT_EQUAL(0, remaining_time_us);
}

void test_lt_schedule(void)
{
    single_timer_schedule(TIMER_STEP - 1);
    single_timer_schedule(TIMER_STEP + 1);
}

void test_lt_timer_handler(void)
{
    single_timer_schedule(0);
    timer_handler_rounds(CEIL_DIV(0, TIMER_STEP));

    single_timer_schedule(TIMER_STEP);
    timer_handler_rounds(CEIL_DIV(TIMER_STEP, TIMER_STEP));

    single_timer_schedule(TIMER_STEP + 1);
    timer_handler_rounds(CEIL_DIV(TIMER_STEP + 1, TIMER_STEP));

    single_timer_schedule(100ull * TIMER_STEP + 42);
    timer_handler_rounds(CEIL_DIV(100ull * TIMER_STEP + 42, TIMER_STEP));
}

void test_lt_abort(void)
{
    timer_sch_abort_Expect(&m_timer.event);
    lt_abort(&m_timer);
}

void test_lt_remaining_time_get(void)
{
    single_timer_schedule(0);
    timer_now_ExpectAndReturn(TIME_NOW);
    m_timer.event.timestamp = TIME_NOW;
    TEST_ASSERT_EQUAL(0, lt_remaining_time_get(&m_timer));

    single_timer_schedule(TIMER_STEP);
    m_timer.event.timestamp = TIME_NOW;
    timer_now_ExpectAndReturn(2 * TIME_NOW);
    TEST_ASSERT_EQUAL(TIMER_STEP - TIME_NOW, lt_remaining_time_get(&m_timer));
}
