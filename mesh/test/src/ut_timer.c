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
#include <stdbool.h>
#include "unity.h"
#include "timer.h"
#include "nrf.h"
#include "nrf_error.h"

static uint32_t         m_callbacks_called;
static uint32_t         m_async_callbacks_called;
static bool             m_is_in_ts;

/* dummy hw modules */
NRF_TIMER_Type *  NRF_TIMER0;
NRF_PPI_Type * NRF_PPI;
static NRF_TIMER_Type   m_dummy_timer;
static NRF_PPI_Type     m_dummy_ppi;

static void callback3000(timestamp_t timestamp)
{
    TEST_ASSERT_EQUAL(3000, timestamp);
    m_callbacks_called++;
}

static void callback5000(timestamp_t timestamp)
{
    TEST_ASSERT_EQUAL(5000, timestamp);
    m_callbacks_called++;
}

static void callback_unexpected(timestamp_t timestamp)
{
    TEST_FAIL_MESSAGE("Unexpected callback");
}

static void s_timer_reset(void)
{
    memset(NRF_TIMER0, 0, sizeof(NRF_TIMER_Type));
#ifdef NRF51
    NRF_TIMER0->POWER = 1;
#endif
    NRF_TIMER0->MODE = TIMER_MODE_MODE_Timer;
    NRF_TIMER0->BITMODE = TIMER_BITMODE_BITMODE_24Bit;
    NRF_TIMER0->PRESCALER = 4; /* 1MHz */
}

static void s_timer_event_trigger(uint8_t index, timestamp_t timestamp)
{
    TEST_ASSERT_TRUE_MESSAGE(m_is_in_ts, "Attempted to call event handler outside TS, rewrite the test.");
    TEST_ASSERT_TRUE_MESSAGE(index < 3, "Illegal index in test, rewrite the test.");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(NULL, NRF_TIMER0, "NRF_TIMER0 is NULL, rewrite the test");
    NRF_TIMER0->EVENTS_COMPARE[index] = 1;
    NRF_TIMER0->CC[index] = timestamp;
    NRF_TIMER0->CC[TIMER_INDEX_TIMESTAMP] = timestamp;
    timer_event_handler();
}

static void s_ts_end(timestamp_t timestamp)
{
    timer_on_ts_end(timestamp);
    m_is_in_ts = false;
}

static void s_ts_begin(timestamp_t timestamp)
{
    TEST_ASSERT_EQUAL_MESSAGE(&m_dummy_timer, NRF_TIMER0, "NRF_TIMER0 is NULL, rewrite the test");
    TEST_ASSERT_EQUAL_MESSAGE(&m_dummy_ppi, NRF_PPI, "NRF_PPI is NULL, rewrite the test");
    timer_on_ts_begin(timestamp);
    m_is_in_ts = true;
}

/****** STUB FOR BEARER EVENT HANDLER ******/
uint32_t bearer_event_timer_post(timer_callback_t callback, timestamp_t timestamp)
{
    /* call synchronously */
    if (callback)
    {
        callback(timestamp);
        m_async_callbacks_called++;
    }
    return NRF_SUCCESS;
}

/***********************************************/

void setUp(void)
{
    m_callbacks_called = 0;
    m_async_callbacks_called = 0;
    NRF_TIMER0 = &m_dummy_timer;
    NRF_PPI = &m_dummy_ppi;
    memset(NRF_TIMER0, 0, sizeof(NRF_TIMER_Type));
    memset(NRF_PPI, 0, sizeof(NRF_PPI_Type));
    s_timer_reset();
    m_is_in_ts = false;
}

void tearDown(void)
{
    (void) timer_abort(0);
    (void) timer_abort(1);
    (void) timer_abort(2);
    s_ts_end(0xFFFFFF);
}

void test_timer_time_now(void)
{
    timer_on_ts_begin(0);

    NRF_TIMER0->CC[TIMER_INDEX_TIMESTAMP] = 100;
    uint32_t time1 = timer_now();
    TEST_ASSERT_EQUAL(100, time1);
    TEST_ASSERT_EQUAL(1, NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP]);
    NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP] = 0;

    /* repeated: */
    NRF_TIMER0->CC[TIMER_INDEX_TIMESTAMP] = 500;
    uint32_t time2 = timer_now();
    TEST_ASSERT_EQUAL(500, time2);
    TEST_ASSERT_EQUAL(1, NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP]);
    NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP] = 0;

    /* should show end of timeslot outside of timeslot */
    NRF_TIMER0->CC[TIMER_INDEX_TIMESTAMP] = 900;
    s_ts_end(800);
    uint32_t end_time = timer_now();
    TEST_ASSERT_EQUAL(800, end_time);

    /* with reference offset: */
    s_ts_begin(1000);
    NRF_TIMER0->CC[TIMER_INDEX_TIMESTAMP] = 200;
    uint32_t time3 = timer_now();
    TEST_ASSERT_EQUAL(1000 + 200, time3);
    TEST_ASSERT_EQUAL(1, NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP]);
    NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP] = 0;
}

void test_timer_return_codes(void)
{
    s_ts_begin(0);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, timer_abort(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, timer_abort(3));

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, timer_order_cb(3, 3000, callback3000, TIMER_ATTR_NONE));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, timer_order_cb_ppi(3, 3000, callback3000, (uint32_t*) &(NRF_TIMER0->TASKS_CAPTURE[0]), TIMER_ATTR_NONE));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, timer_order_ppi(3, 1000, (uint32_t*) &(NRF_TIMER0->TASKS_CAPTURE[0]), TIMER_ATTR_NONE));

    /*lint -save -e64 Invalid value used for enum parameter (0xff) */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_FLAGS, timer_order_cb(1, 3000, callback3000, 0xFF));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_FLAGS, timer_order_cb_ppi(1, 3000, callback3000, (uint32_t*) &(NRF_TIMER0->TASKS_CAPTURE[0]), 0xFF));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_FLAGS, timer_order_ppi(1, 1000, (uint32_t*) &(NRF_TIMER0->TASKS_CAPTURE[0]), 0xFF));
    /*lint -restore */

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_FLAGS, timer_order_ppi(1, 1000, (uint32_t*) &(NRF_TIMER0->TASKS_CAPTURE[0]), TIMER_ATTR_SYNCHRONOUS));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 3000, callback3000, TIMER_ATTR_NONE));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 3000, callback3000, TIMER_ATTR_SYNCHRONOUS));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 3000, callback3000, TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 3000, callback3000, TIMER_ATTR_TIMESLOT_LOCAL));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb_ppi(1, 3000, callback3000, (uint32_t*) (uint32_t) &(NRF_TIMER0->TASKS_CAPTURE[0]), TIMER_ATTR_NONE));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb_ppi(1, 3000, callback3000, (uint32_t*) (uint32_t) &(NRF_TIMER0->TASKS_CAPTURE[0]), TIMER_ATTR_TIMESLOT_LOCAL));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_ppi(1, 1000, (uint32_t*) (uint32_t) &(NRF_TIMER0->TASKS_CAPTURE[0]), TIMER_ATTR_NONE));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_ppi(1, 1000, (uint32_t*) (uint32_t) &(NRF_TIMER0->TASKS_CAPTURE[0]), TIMER_ATTR_TIMESLOT_LOCAL));
}

void test_timer_callback(void)
{
    s_ts_begin(0);
    /* test sync */
    TEST_ASSERT_EQUAL(0, m_callbacks_called);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 3000, callback3000, TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL));

    s_timer_event_trigger(1, 3000);

    TEST_ASSERT_EQUAL(1, m_callbacks_called);
    TEST_ASSERT_EQUAL(0, m_async_callbacks_called);

    /* test async */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 5000, callback5000, TIMER_ATTR_TIMESLOT_LOCAL));

    s_timer_event_trigger(1, 5000);
    TEST_ASSERT_EQUAL(2, m_callbacks_called);
    TEST_ASSERT_EQUAL(1, m_async_callbacks_called);
}

void test_timer_ppi(void)
{
    s_ts_begin(0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_ppi(1, 3000, (uint32_t*) 0xAABBCCDD, TIMER_ATTR_NONE));
    TEST_ASSERT_EQUAL(0xAABBCCDD, NRF_PPI->CH[TIMER_PPI_CH_START + 1].TEP);
    TEST_ASSERT_EQUAL((uint32_t*) (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[1]), NRF_PPI->CH[TIMER_PPI_CH_START + 1].EEP);
    TEST_ASSERT_EQUAL((1 << (PPI_CHEN_CH0_Pos + TIMER_PPI_CH_START + 1)), NRF_PPI->CHENSET);
}

void test_timer_ppi_cb(void)
{
    s_ts_begin(0);
    TEST_ASSERT_EQUAL(0, m_callbacks_called);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb_ppi(1, 3000, callback3000, (uint32_t*) 0xAABBCCDD, TIMER_ATTR_NONE));

    TEST_ASSERT_EQUAL((1 << (PPI_CHEN_CH0_Pos + TIMER_PPI_CH_START + 1)), NRF_PPI->CHENSET);
    TEST_ASSERT_EQUAL(0xAABBCCDD, NRF_PPI->CH[TIMER_PPI_CH_START + 1].TEP);
    TEST_ASSERT_EQUAL((uint32_t*) (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[1]), NRF_PPI->CH[TIMER_PPI_CH_START + 1].EEP);
    TEST_ASSERT_EQUAL((1 << (TIMER_INTENSET_COMPARE0_Pos + 1)), NRF_TIMER0->INTENSET);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb_ppi(2, 5000, callback5000, (uint32_t*) 0x00AABBCC, TIMER_ATTR_NONE));
    TEST_ASSERT_EQUAL(0x00AABBCC, NRF_PPI->CH[TIMER_PPI_CH_START + 2].TEP);
    TEST_ASSERT_EQUAL((uint32_t*) (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[2]), NRF_PPI->CH[TIMER_PPI_CH_START + 2].EEP);
    TEST_ASSERT_EQUAL((1 << (PPI_CHEN_CH0_Pos + TIMER_PPI_CH_START + 2)), NRF_PPI->CHENSET);
    TEST_ASSERT_EQUAL((1 << (TIMER_INTENSET_COMPARE0_Pos + 2)), NRF_TIMER0->INTENSET);

    /* Manually set the INTENSET register to mark both triggered timers, as the HW would do: */
    NRF_TIMER0->INTENSET = ((1 << (TIMER_INTENSET_COMPARE0_Pos + 1)) | (1 << (TIMER_INTENSET_COMPARE0_Pos + 2)));

    s_timer_event_trigger(1, 3000);
    TEST_ASSERT_EQUAL(1, m_callbacks_called);
    s_timer_event_trigger(2, 5000);
    TEST_ASSERT_EQUAL(2, m_callbacks_called);
}

/** Timer gets a start reference from the timeslot handler, must be tested. */
void test_timer_reference(void)
{
    s_ts_begin(1000);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 3000, callback3000, TIMER_ATTR_SYNCHRONOUS));
    TEST_ASSERT_EQUAL(0, m_callbacks_called);
    s_timer_event_trigger(1, 3000 - 1000);
    TEST_ASSERT_EQUAL(1, m_callbacks_called);
}

void test_timer_timeslot_carryover(void)
{
    s_ts_begin(1000);
    TEST_ASSERT_EQUAL(0, m_callbacks_called);
    /* setup callbacks */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(0, 5000, callback5000, TIMER_ATTR_SYNCHRONOUS)); /* expires in second timeslot */
    TEST_ASSERT_EQUAL((1 << (TIMER_INTENSET_COMPARE0_Pos + 0)), NRF_TIMER0->INTENSET);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(1, 2500, callback3000, TIMER_ATTR_SYNCHRONOUS)); /* expires between timeslots */
    TEST_ASSERT_EQUAL((1 << (TIMER_INTENSET_COMPARE0_Pos + 1)), NRF_TIMER0->INTENSET);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, timer_order_cb(2, 6000, callback_unexpected, TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL)); /* should never fire */
    TEST_ASSERT_EQUAL((1 << (TIMER_INTENSET_COMPARE0_Pos + 2)), NRF_TIMER0->INTENSET);


    /* Manually set the INTENSET register to mark all enabled timers, as the HW would do: */
    NRF_TIMER0->INTENSET =
        ((1 << (TIMER_INTENSET_COMPARE0_Pos + 0)) | (1 << (TIMER_INTENSET_COMPARE0_Pos + 1)) |
         (1 << (TIMER_INTENSET_COMPARE0_Pos + 2)));

    /* End and restart timeslot */
    s_ts_end(2000);
    TEST_ASSERT_EQUAL(0, m_callbacks_called);

    s_ts_begin(3000);
    TEST_ASSERT_EQUAL(1, m_callbacks_called);

    s_timer_event_trigger(0, 5000 - 3000);
    TEST_ASSERT_EQUAL(2, m_callbacks_called);

    s_timer_event_trigger(2, 6000 - 3000); /* shouldn't trigger a callback, as it's timeslot local */
    TEST_ASSERT_EQUAL(2, m_callbacks_called);
}

