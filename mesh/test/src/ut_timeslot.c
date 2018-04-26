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
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unity.h>
#include <cmock.h>

#include "timeslot.h"
#include "nrf.h"
#include "nrf_mesh.h"

#include "nordic_common.h"
#include "test_assert.h"

#include "timer_mock.h"
#include "bearer_handler_mock.h"
#include "nrf_mesh_cmsis_mock_mock.h"

#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_error.h"

static nrf_radio_signal_callback_t m_signal_handler;
static uint32_t m_radio_session_open_expect;
static uint32_t m_radio_session_close_expect;
static uint32_t m_radio_request_expect;
static uint32_t m_return_code_session_open;
static uint32_t m_return_code_session_close;
static uint32_t m_return_code_request;
static timer_callback_t m_end_timer_callback;
static nrf_radio_request_t m_req;
static uint32_t m_req_len; /**< Number of bytes to compare in m_req (it has a union that can contain some garbage outside of the set fields) */

NRF_RTC_Type* NRF_RTC0;

void setUp(void)
{
    timer_mock_Init();
    bearer_handler_mock_Init();
    nrf_mesh_cmsis_mock_mock_Init();

    NRF_RTC0 = calloc(1, sizeof(NRF_RTC_Type));
    m_end_timer_callback = NULL;
    timeslot_init(250);
    m_radio_session_open_expect = 0;
    m_radio_session_close_expect = 0;
    m_radio_request_expect = 0;
    m_return_code_session_open = 0;
    m_return_code_session_close = 0;
    m_return_code_request = 0;

}

void tearDown(void)
{
    free(NRF_RTC0);
    timer_mock_Verify();
    timer_mock_Destroy();
    bearer_handler_mock_Verify();
    bearer_handler_mock_Destroy();
    nrf_mesh_cmsis_mock_mock_Verify();
    nrf_mesh_cmsis_mock_mock_Destroy();
}

/******* Softdevice mocks **********/
uint32_t sd_radio_session_open(nrf_radio_signal_callback_t callback)
{
    TEST_ASSERT_NOT_EQUAL(0, m_radio_session_open_expect);
    m_radio_session_open_expect--;
    m_signal_handler = callback;
    return m_return_code_session_open;
}

uint32_t sd_radio_session_close(void)
{
    TEST_ASSERT_NOT_EQUAL(0, m_radio_session_close_expect);
    m_radio_session_close_expect--;
    return m_return_code_session_close;
}

uint32_t sd_radio_request(nrf_radio_request_t const * p_req)
{
    TEST_ASSERT_NOT_EQUAL(0, m_radio_request_expect);
    m_radio_request_expect--;
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t*) &m_req, (uint8_t*) p_req, m_req_len);

    return m_return_code_request;
}

/*********** Helpers ************/
/** Get the number of microseconds of expected end margin, based on clock source. */
static uint32_t end_timer_drift_margin(uint32_t time, uint32_t ppm)
{
    return ((40 + ppm) * time) / 1000000;
}

static void expect_end_timer_order(uint32_t timeslot_length, uint32_t ppm)
{
    timer_order_cb_ExpectAndReturn(TIMER_INDEX_TS_END,
                                   timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US -
                                       end_timer_drift_margin(timeslot_length, 250),
                                   NULL,
                                   (timer_attr_t)(TIMER_ATTR_TIMESLOT_LOCAL |
                                                  TIMER_ATTR_SYNCHRONOUS),
                                   NRF_SUCCESS);
    timer_order_cb_IgnoreArg_callback();
}

/** Start timeslot operation. Will give us a function pointer to the signal callback. */
static void m_start_ts(void)
{
    m_req.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_req.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_req.params.earliest.length_us = TIMESLOT_BASE_LENGTH_LONG_US;
    m_req.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
    m_req.params.earliest.timeout_us = 15000;
    m_req_len = 1 + sizeof(nrf_radio_request_earliest_t);

    m_return_code_session_open = NRF_SUCCESS;
    m_radio_session_open_expect = 1;
    m_return_code_request = NRF_SUCCESS;
    m_radio_request_expect = 1;

    timeslot_start();
    TEST_ASSERT_NOT_NULL(m_signal_handler);
}

static void m_start_ts_with_maxlen(void)
{
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    timer_on_ts_begin_Expect(timeslot_start_time_get());
    bearer_handler_on_ts_begin_Expect();
    timer_order_cb_IgnoreAndReturn(NRF_SUCCESS);

    while (m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED)->callback_action ==
           NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND); // send the timeslot into max length
}

static void m_expect_ts_trigger(void)
{
    NVIC_SetPendingIRQ_Expect(TIMER0_IRQn);
}

/** Timer mock callback, getting the end-of-timeslot mock */
uint32_t timer_order_cb_mock(uint8_t timer,
                        timestamp_t time,
                        timer_callback_t callback,
                        timer_attr_t attributes,
                        int count)
{
    TEST_ASSERT_EQUAL(TIMER_INDEX_TS_END, timer);
    TEST_ASSERT_EQUAL((timer_attr_t) (TIMER_ATTR_TIMESLOT_LOCAL | TIMER_ATTR_SYNCHRONOUS), attributes);
    m_end_timer_callback = callback;
    return NRF_SUCCESS;
}

void timer_event_handler_mock(int count)
{
    TEST_ASSERT_NOT_NULL(m_end_timer_callback);
    TEST_ASSERT_TRUE(timeslot_is_in_cb());
    m_end_timer_callback(TIMESLOT_MAX_LENGTH_US);
}
/******************************************/
void test_init(void)
{
    timeslot_init(250);
}

void test_start(void)
{
    m_req.request_type = NRF_RADIO_REQ_TYPE_EARLIEST;
    m_req.params.earliest.hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED;
    m_req.params.earliest.length_us = TIMESLOT_BASE_LENGTH_LONG_US;
    m_req.params.earliest.priority = NRF_RADIO_PRIORITY_NORMAL;
    m_req.params.earliest.timeout_us = 15000;
    m_req_len = 1 + sizeof(nrf_radio_request_earliest_t);

    /* Fail to open session */
    m_return_code_session_open = 0x1234;
    m_radio_session_open_expect = 1;
    TEST_ASSERT_EQUAL_HEX32(0x1234, timeslot_start());
    TEST_ASSERT_EQUAL(0, m_radio_session_open_expect);
    /* Fail to request slot */
    m_return_code_session_open = 0;
    m_radio_session_open_expect = 1;
    m_return_code_request = 0x5678;
    m_radio_request_expect = 1;

    TEST_ASSERT_EQUAL_HEX32(0x5678, timeslot_start());
    TEST_ASSERT_EQUAL(0, m_radio_session_open_expect);
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);

    /* Successfully start */
    m_return_code_session_open = NRF_SUCCESS;
    m_radio_session_open_expect = 1;
    m_return_code_request = NRF_SUCCESS;
    m_radio_request_expect = 1;

    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, timeslot_start());
    TEST_ASSERT_EQUAL(0, m_radio_session_open_expect);
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);

    /* Shouldn't keep any internal state preventing us from starting twice */
    m_return_code_session_open = NRF_SUCCESS;
    m_radio_session_open_expect = 1;
    m_return_code_request = NRF_SUCCESS;
    m_radio_request_expect = 1;

    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, timeslot_start());
    TEST_ASSERT_EQUAL(0, m_radio_session_open_expect);
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);
}

void test_rtc_read(void)
{
    m_start_ts();

    NRF_RTC0->COUNTER = 0x1AAA;
    /* The start of the first timeslot is regarded the epoch. */
    TEST_ASSERT_EQUAL(0, timeslot_start_time_get());
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL_HEX32(0, timeslot_start_time_get());

    /* The subsequent timeslots pick up the RTC counter, and adjusts the timestamp relatively to the first. */
    NRF_RTC0->COUNTER = 0x2AAA;
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL_HEX32((0x1000ULL * 1000000ULL) >> 15ULL, timeslot_start_time_get());

    NRF_RTC0->COUNTER = 0x3AAA;
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL_HEX32((0x2000ULL * 1000000ULL) >> 15ULL, timeslot_start_time_get());

    /* Emulate a long hold (> RTC-RANGE / 2) without a new timeslot. */
    NRF_RTC0->COUNTER = 0xFF3AAA;
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL_HEX32((0xFF2000ULL * 1000000ULL) >> 15ULL, timeslot_start_time_get());

    /** Roll over the RTC, but not the us-level timestamp. */
    NRF_RTC0->COUNTER = 0x1AAA;
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL_HEX32((0x1000000ULL * 1000000ULL) >> 15ULL, timeslot_start_time_get());
}

void test_extensions(void)
{
    m_start_ts();
    nrf_radio_signal_callback_return_param_t* p_ret = NULL;
    /* Start should attempt to extend to the length of the previous timeslot.
       On init, the length of the previous timeslot is set to
       TIMESLOT_MAX_LENGTH_US. */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(TIMESLOT_MAX_LENGTH_US - TIMESLOT_BASE_LENGTH_LONG_US, p_ret->params.extend.length_us);

    /* We never confirmed the extension, and as a result, the previous slot
       length == base length. In order to have this timeslot match the
       previous, we shouldn't extend at all. If we don't attempt to extend,
       we'll never know whether we can increase our slot. Therefore, we attempt
       to extend by the smallest amount, to see if we can extend any. */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(1000, p_ret->params.extend.length_us);

    /* Confirm the extension, to start the negotiation process.  */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(TIMESLOT_EXTEND_LENGTH_MIN_US, p_ret->params.extend.length_us);

    /* By failing the second extend, the module won't attempt to extend any
       more. Instead, it'll start the timeslot operation. */
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(TIMESLOT_BASE_LENGTH_LONG_US + 1000, 250);
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Reset the module to go back to max length */
    timeslot_init(250);

    /* Expect full length, grant it. The module shouldn't attempt to grab more
       than this, as it caps at TIMESLOT_MAX_LENGTH. Instead, it'll start the timeslot operation. */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(TIMESLOT_MAX_LENGTH_US - TIMESLOT_BASE_LENGTH_LONG_US, p_ret->params.extend.length_us);

    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(TIMESLOT_MAX_LENGTH_US, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Simulate a shorter max-timeslot by denying extensions. First, the module
       will attempt to get a timeslot the size of the previous slot. We'll make
       it fail this, so that we can start negotiations. */
    uint32_t req_time = TIMESLOT_MAX_LENGTH_US - TIMESLOT_BASE_LENGTH_LONG_US;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    req_time /= 2;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);
    uint32_t total_time = TIMESLOT_BASE_LENGTH_LONG_US; /* No successful extensions yet. */

    /* The module will keep requesting until it has cut the extension request
       time to be shorter than TIMESLOT_EXTEND_LENGTH_MIN_US or the number of
       requests have surpassed TIMESLOT_EXTEND_MAX_COUNT. This happens
       regardless of whether the extend is successful, so we'll just grant and
       deny extensions in an arbitrary pattern. Since the extension behaves
       like a binary search (by cutting the search area in half each time),
       each timeslot length can only be reached through the exact grant pattern
       we give it. This means that a change in the algorithm can be detected by
       any arbitrary pattern, as the ending length wouldn't match that
       pattern's length. */

    /* The arbitrary grant/deny pattern, every bit is an extension attempt. */
    const uint64_t extension_success = 0x3126479316364378ULL;

    for (uint32_t i = 0; i < 64 && req_time/2 > TIMESLOT_EXTEND_LENGTH_MIN_US; ++i)
    {
        bool success = (extension_success >> (uint64_t) i) & 0x01ULL;
        if (success)
        {
            total_time += req_time;
        }
        req_time /= 2;
        p_ret = m_signal_handler(success? NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED : NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
        TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
        TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);
    }

    /* Finally, the module will stop extending, and instead start the timeslot. */
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(total_time, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Remove the upper limit on the timeslot again. The module should detect
       this by first attempting to get the previous length, then attempt to add
       50%. If that works, it'll request maxlength. Everything after this is
       business as usual. */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(total_time - TIMESLOT_BASE_LENGTH_LONG_US, p_ret->params.extend.length_us);

    /* Adds an additional 50%, to try the waters: */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL((total_time - TIMESLOT_BASE_LENGTH_LONG_US) / 2, p_ret->params.extend.length_us);

    /* Gets really greedy, really fast, immediately asks for as much as possible: */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(TIMESLOT_MAX_LENGTH_US - (total_time + (total_time - TIMESLOT_BASE_LENGTH_LONG_US) / 2), p_ret->params.extend.length_us);

    /* Grant it. Can't extend further, so the timeslot operation starts. */
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(TIMESLOT_MAX_LENGTH_US, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Run two MAX-timeslots in a row */
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(TIMESLOT_MAX_LENGTH_US - TIMESLOT_BASE_LENGTH_LONG_US, p_ret->params.extend.length_us);
    /* Got MAX-timeslot, so the timeslot operation starts. */
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(TIMESLOT_MAX_LENGTH_US, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Set up the module to have a non-max timeslot again: */
    req_time = TIMESLOT_MAX_LENGTH_US - TIMESLOT_BASE_LENGTH_LONG_US;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    req_time /= 2;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);
    req_time /= 2;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    /* Fail a lot to stop the ordering, and we'll have a MAX/2 long timeslot */
    for (uint32_t i = 0; i < 64 && req_time/2 > TIMESLOT_EXTEND_LENGTH_MIN_US; ++i)
    {
        req_time /= 2;
        p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
        TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
        TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);
    }
    total_time = TIMESLOT_BASE_LENGTH_LONG_US + (TIMESLOT_MAX_LENGTH_US - TIMESLOT_BASE_LENGTH_LONG_US)/2;
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(total_time, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Alright! */

    /* Now give it the same non-max length twice. It should detect this by
       first extending to the previous, then attempting to add 50%. This will
       fail, and it'll assume that nothing could be added, and start the
       timeslot operation. */
    req_time = total_time - TIMESLOT_BASE_LENGTH_LONG_US;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    /* The original length is met */
    req_time /= 2;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);
    /* The additional 50% is not met, so the timeslot operation starts. */
    req_time /= 2;
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(total_time, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Bring the timeslot down to the base length by only accepting the start,
     * but rejecting all extension attempts: */
    req_time = total_time - TIMESLOT_BASE_LENGTH_LONG_US;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    req_time /= 2;
    while (req_time >= TIMESLOT_EXTEND_LENGTH_MIN_US)
    {
        p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
        TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
        TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);
        req_time /= 2;
    }
    /* Now starting timeslot with base time. */
    total_time = TIMESLOT_BASE_LENGTH_LONG_US;
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(total_time, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* When the module finds that the previous timeslot was the length of base
     * time, it will attempt to make a tiny increase, sort of "testing the
     * waters". Reject this attempt, and it should just accept its short timeslot. */
    req_time = 1000;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    /* Reject the tiny increment, this starts operation. */
    total_time = TIMESLOT_BASE_LENGTH_LONG_US;
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(total_time, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Trigger the signal handler with a blocked signal. This should make the
     * module think it requested a long timeslot, but that this failed. This
     * prompts the module to order a short timeslot, which we will grant. */
    m_radio_request_expect = 1;
    m_return_code_request = NRF_SUCCESS;
    m_req.params.earliest.length_us = TIMESLOT_BASE_LENGTH_SHORT_US;
    timeslot_sd_event_handler(NRF_EVT_RADIO_BLOCKED);
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);

    /* Was able to start the short timeslot. The module will attempt to get the
     * full length of the previous. Reject it! */
    req_time = total_time - TIMESLOT_BASE_LENGTH_SHORT_US;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    req_time /= 2;
    while (req_time >= TIMESLOT_EXTEND_LENGTH_MIN_US)
    {
        p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
        TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
        TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);
        req_time /= 2;
    }

    /* Now starting timeslot with the short base time. */
    total_time = TIMESLOT_BASE_LENGTH_SHORT_US;
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    timer_order_cb_StubWithCallback(timer_order_cb_mock);
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);
    timer_order_cb_StubWithCallback(NULL);

    /* End the short timeslot. The module should request another short one. */
    /* Do the little dance to trigger the end timer callback: */
    timer_event_handler_StubWithCallback(timer_event_handler_mock);
    bearer_handler_timer_irq_handler_Expect();
    timer_on_ts_end_Expect(total_time - TIMESLOT_END_SAFETY_MARGIN_US - end_timer_drift_margin(total_time, 250));
    bearer_handler_on_ts_end_Expect();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    timer_event_handler_StubWithCallback(NULL);

    /* Verify the short timeslot order */
    m_req.params.earliest.length_us = TIMESLOT_BASE_LENGTH_SHORT_US;
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END, p_ret->callback_action);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t*) &m_req, (uint8_t*) p_ret->params.request.p_next, m_req_len);

    /* When the module finds that the previous timeslot was the length of base
     * time, it will attempt to make a tiny increase, sort of "testing the
     * waters". Reject this attempt, and it should just accept its short timeslot. */
    req_time = 1000;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    /* Reject the tiny increment, this starts operation. */
    total_time = TIMESLOT_BASE_LENGTH_SHORT_US;
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    expect_end_timer_order(total_time, 250);
    timer_order_cb_IgnoreArg_callback();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* The module continues getting these small timeslots. This time we'll grant the tiny extension attempt. */
    req_time = 1000;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    /* Grant the tiny extension. The module asks for the smallest normal extension */
    req_time = TIMESLOT_EXTEND_LENGTH_MIN_US;
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    /* Grant the normal extension. The module asks for maximum extension */
    req_time = TIMESLOT_MAX_LENGTH_US - (TIMESLOT_BASE_LENGTH_SHORT_US + 1000 + TIMESLOT_EXTEND_LENGTH_MIN_US);
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND, p_ret->callback_action);
    TEST_ASSERT_EQUAL(req_time, p_ret->params.extend.length_us);

    /* Grant the max extension. The module starts the timeslot operation, as it can't ask for more. */
    total_time = TIMESLOT_MAX_LENGTH_US;
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    /* Get the end timer. */
    timer_order_cb_StubWithCallback(timer_order_cb_mock);
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);
    timer_order_cb_StubWithCallback(NULL);

    /* End the long timeslot. The module should request another long one. */
    /* Do the little dance to trigger the end timer callback: */
    timer_event_handler_StubWithCallback(timer_event_handler_mock);
    bearer_handler_timer_irq_handler_Expect();
    timer_on_ts_end_Expect(total_time - TIMESLOT_END_SAFETY_MARGIN_US - end_timer_drift_margin(total_time, 250));
    bearer_handler_on_ts_end_Expect();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    timer_event_handler_StubWithCallback(NULL);

    /* Verify the short timeslot order */
    m_req.params.earliest.length_us = TIMESLOT_BASE_LENGTH_LONG_US;
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END, p_ret->callback_action);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t*) &m_req, (uint8_t*) p_ret->params.request.p_next, m_req_len);

}

void test_forced_commands(void)
{
    NRF_RTC0->COUNTER = 0;
    nrf_radio_signal_callback_return_param_t* p_ret = NULL;

    timer_on_ts_begin_Ignore();
    bearer_handler_on_ts_begin_Ignore();
    timer_order_cb_IgnoreAndReturn(NRF_SUCCESS);

    /* This should happen regardless of which signal we're getting: */
    const uint8_t signals[] = {
        NRF_RADIO_CALLBACK_SIGNAL_TYPE_START,
        NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO,
        NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0,
        NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED,
        NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED
    };

    NRF_RTC0->COUNTER = 0;

    for (uint32_t i = 0; i < ARRAY_SIZE(signals); ++i)
    {
        m_start_ts_with_maxlen();

        m_expect_ts_trigger();
        timeslot_restart();
        bearer_handler_on_ts_end_Expect();
        uint32_t end_time = timeslot_start_time_get() + 1000;
        timer_now_ExpectAndReturn(end_time);
        timer_on_ts_end_Expect(end_time - TIMESLOT_END_SAFETY_MARGIN_US);
        p_ret = m_signal_handler(signals[i]);
        TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END, p_ret->callback_action);
        timer_mock_Verify();

        /* Start the timeslot, but don't extend enough to start operations. No modules should get
         * the on_end() call, as they didn't get the on_begin() */
        m_start_ts();
        timeslot_stop();
        end_time = timeslot_start_time_get() + 1000;
        timer_now_ExpectAndReturn(end_time - TIMESLOT_END_SAFETY_MARGIN_US);
        p_ret = m_signal_handler(signals[i]);
        TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_END, p_ret->callback_action);

        /* Restart the timeslot */
        m_start_ts_with_maxlen();

        m_expect_ts_trigger();

        timeslot_stop();

        bearer_handler_on_ts_end_Expect();
        end_time = timeslot_start_time_get() + 1000;
        timer_now_ExpectAndReturn(end_time);
        timer_on_ts_end_Expect(end_time - TIMESLOT_END_SAFETY_MARGIN_US);
        p_ret = m_signal_handler(signals[i]);
        TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_END, p_ret->callback_action);

        /* Check the slot start time, should NOT account for the stopped time, only the length of the previous timeslot: */
        NRF_RTC0->COUNTER = (i + 1) * 10000;
        m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
        TEST_ASSERT_EQUAL((i + 1) * (2000 - TIMESLOT_END_SAFETY_MARGIN_US), timeslot_start_time_get());
        timer_mock_Verify();
    }

}

void test_signal_propagation(void)
{
    m_start_ts();

    bearer_handler_radio_irq_handler_Expect();
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO);

    timer_event_handler_Expect();
    bearer_handler_timer_irq_handler_Expect();
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
}

void test_end_timer(void)
{
    m_start_ts();


    /* Grant a max-len timeslot */
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    timer_order_cb_StubWithCallback(timer_order_cb_mock);
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);

    TEST_ASSERT_NOT_NULL(m_end_timer_callback);
    timer_mock_Verify();

    /* Fire the end timer, should cause the timeslot to end. */
    timer_event_handler_StubWithCallback(timer_event_handler_mock);
    bearer_handler_timer_irq_handler_Expect();
    timer_on_ts_end_Expect(TIMESLOT_MAX_LENGTH_US - TIMESLOT_END_SAFETY_MARGIN_US - end_timer_drift_margin(TIMESLOT_MAX_LENGTH_US, 250));
    bearer_handler_on_ts_end_Expect();
    nrf_radio_signal_callback_return_param_t* p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END, p_ret->callback_action);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t*) &m_req, (uint8_t*) p_ret->params.request.p_next, m_req_len);
    timer_mock_Verify();

    /* Ensure that the next timeslot works the same way: */
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();
    timer_order_cb_StubWithCallback(NULL); /* Stop the callback */
    expect_end_timer_order(TIMESLOT_MAX_LENGTH_US, 250);
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);
}

void test_sd_event_handler(void)
{
    m_start_ts();

    m_radio_session_close_expect = 1;
    m_return_code_session_close = NRF_SUCCESS;
    timeslot_sd_event_handler(NRF_EVT_RADIO_SESSION_IDLE);
    TEST_ASSERT_EQUAL(0, m_radio_session_close_expect);

    m_radio_session_close_expect = 1;
    m_return_code_session_close = 0x1234; /* Random non-success return code */
    TEST_NRF_MESH_ASSERT_EXPECT(timeslot_sd_event_handler(NRF_EVT_RADIO_SESSION_IDLE));
    TEST_ASSERT_EQUAL(0, m_radio_session_close_expect);

    /* Getting a canceled event should cause the module to request a new timeslot. */
    m_radio_request_expect = 1;
    m_return_code_request = NRF_SUCCESS;
    m_req.params.earliest.length_us = TIMESLOT_BASE_LENGTH_LONG_US;
    timeslot_sd_event_handler(NRF_EVT_RADIO_CANCELED);
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);

    m_radio_request_expect = 1;
    m_return_code_request = 0x1234; /* Random non-success return code */
    TEST_NRF_MESH_ASSERT_EXPECT(timeslot_sd_event_handler(NRF_EVT_RADIO_CANCELED));
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);

    /* Getting a blocked event should cause the module to request with the smallest possible amount. */
    m_radio_request_expect = 1;
    m_return_code_request = NRF_SUCCESS;
    m_req.params.earliest.length_us = TIMESLOT_BASE_LENGTH_SHORT_US;
    timeslot_sd_event_handler(NRF_EVT_RADIO_BLOCKED);
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);

    m_radio_request_expect = 1;
    m_return_code_request = 0x1234; /* Random non-success return code */
    TEST_NRF_MESH_ASSERT_EXPECT(timeslot_sd_event_handler(NRF_EVT_RADIO_BLOCKED));
    TEST_ASSERT_EQUAL(0, m_radio_request_expect);

    TEST_NRF_MESH_ASSERT_EXPECT(timeslot_sd_event_handler(NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN));
    timeslot_sd_event_handler(NRF_EVT_RADIO_SESSION_CLOSED); /* Shouldn't do antything */

}

void test_state_peek_functions(void)
{
    m_start_ts();
    TEST_ASSERT_FALSE(timeslot_is_in_ts());
    TEST_ASSERT_FALSE(timeslot_is_in_cb());
    TEST_ASSERT_EQUAL(0, timeslot_end_time_get());
    TEST_ASSERT_EQUAL(0, timeslot_start_time_get());
    TEST_ASSERT_EQUAL(0, timeslot_remaining_time_get());

    /* Start a MAX len timeslot */
    uint32_t ts_end_time = TIMESLOT_MAX_LENGTH_US - TIMESLOT_END_SAFETY_MARGIN_US - end_timer_drift_margin(TIMESLOT_MAX_LENGTH_US, 250);
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_START);
    timer_on_ts_begin_Expect(0);
    bearer_handler_on_ts_begin_Expect();

    expect_end_timer_order(TIMESLOT_MAX_LENGTH_US, 250);
    timer_order_cb_IgnoreArg_callback();
    m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED);

    TEST_ASSERT_TRUE(timeslot_is_in_ts());
    TEST_ASSERT_FALSE(timeslot_is_in_cb());
    TEST_ASSERT_EQUAL(ts_end_time, timeslot_end_time_get());
    TEST_ASSERT_EQUAL(0, timeslot_start_time_get());
    timer_now_ExpectAndReturn(1000);
    TEST_ASSERT_EQUAL(ts_end_time - 1000, timeslot_remaining_time_get());
}

void test_state_lock(void)
{
    NRF_RTC0->COUNTER = 0;
    m_start_ts_with_maxlen();

    nrf_radio_signal_callback_return_param_t* p_ret = NULL;

    /* Lock the state, and make sure any forced commands can't affect the timeslot. */
    timeslot_state_lock(true);

    TEST_ASSERT_FALSE(timeslot_end_is_pending());
    m_expect_ts_trigger();
    timeslot_restart();
    TEST_ASSERT_TRUE(timeslot_end_is_pending());

    /* Other IRQs during the lock should be handled, and the forced command should be ignored. */
    timer_event_handler_Expect();
    bearer_handler_timer_irq_handler_Expect();
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE, p_ret->callback_action);

    /* Unlocking the state causes the TS to trigger. */
    m_expect_ts_trigger();
    timeslot_state_lock(false);

    /* Should execute the forced command now that the state is unlocked. */
    bearer_handler_on_ts_end_Expect();
    timer_on_ts_end_Ignore();
    timer_now_ExpectAndReturn(1000);
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END, p_ret->callback_action);
    bearer_handler_mock_Verify();

    /* End is never pending when the state is locked */
    m_start_ts_with_maxlen();

    timeslot_state_lock(true);

    TEST_ASSERT_FALSE(timeslot_end_is_pending());
    m_expect_ts_trigger();
    timeslot_stop();
    nrf_mesh_cmsis_mock_mock_Verify();
    TEST_ASSERT_TRUE(timeslot_end_is_pending());

    m_expect_ts_trigger();
    timeslot_state_lock(false);

    /* Execute the stop. */
    bearer_handler_on_ts_end_Expect();
    timer_on_ts_end_Ignore();
    timer_now_ExpectAndReturn(2000);
    p_ret = m_signal_handler(NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0);
    TEST_ASSERT_EQUAL(NRF_RADIO_SIGNAL_CALLBACK_ACTION_END, p_ret->callback_action);
}
