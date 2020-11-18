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

#include "model_common.h"

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>

#include <unity.h>
#include <cmock.h>

#include "access.h"

#include "timer_scheduler_mock.h"
#include "timer_mock.h"
#include "app_timer_mock.h"

static access_message_rx_meta_t m_meta;
static uint8_t m_tid;
static uint32_t m_opcode;

static timer_event_t * mp_tid_expiry_timer;
static tid_tracker_t m_tid_item;
static timestamp_t m_exp_timestamp;
static uint16_t    m_exp_src;
static uint16_t    m_exp_dst;
static uint32_t    m_exp_opcode;
static uint8_t     m_exp_old_tid;

#define OPCODE_1             (0x11111111)
#define OPCODE_2             (0x22222222)
#define TIMER_NOW_1          (0x00000010ul)

#define TID_TIMESTAMP(tm) (tm + SEC_TO_US(6))

/* For model timer tests */
#define TIMEOUT_TICKS_1         (1000)
#define TIMER_CONTEXT_1         (0xAABBCCDDEE)

#define TIMEOUT_TICKS_2         (2000)
#define TIMER_CONTEXT_2         (0x5555555555)

#define MAX_RTC_COUNTER_VAL     (0x00FFFFFF)

#define APP_TIMER_MAX_TIMEOUT   (APP_TIMER_MAX_CNT_VAL - (APP_TIMER_MIN_TIMEOUT_TICKS * 2))

static app_timer_id_t m_app_timer_id;
static app_timer_mode_t m_app_timer_mode;
static app_timer_timeout_handler_t m_app_timer_cb;

static uint32_t m_app_timer_count = 0;

static model_timer_t m_model_timer;
static model_timer_t m_exp_model_timer;
static bool m_test_expected_active;


APP_TIMER_DEF(test_timer);


/* Test related static functions */
static void timer_sch_reschedule_mock(timer_event_t * p_timer, timestamp_t new_timestamp, int count)
{
    tid_tracker_t * p_tid_item = (tid_tracker_t *) p_timer->p_context;
    mp_tid_expiry_timer = p_timer;
    TEST_ASSERT_EQUAL(m_exp_timestamp, new_timestamp);
    TEST_ASSERT_EQUAL(0, p_timer->interval);
    TEST_ASSERT_NOT_NULL(p_timer->cb);
    TEST_ASSERT_EQUAL(m_exp_src, p_tid_item->src);
    TEST_ASSERT_EQUAL(m_exp_dst, p_tid_item->dst);
    TEST_ASSERT_EQUAL(m_exp_old_tid, p_tid_item->old_tid);
}

static void helper_tid_validate_expect(uint32_t timer_val, uint16_t src_addr, uint16_t dst_addr, uint32_t opcode, uint16_t tid)
{
    timer_now_ExpectAndReturn(timer_val);
    m_exp_timestamp = TID_TIMESTAMP(timer_val);
    m_exp_src = src_addr;
    m_exp_dst = dst_addr;
    m_exp_opcode = opcode;
    m_exp_old_tid = tid;
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_mock);
}

static void helper_timer_cb_trigger(void)
{
    TEST_ASSERT_NOT_NULL(mp_tid_expiry_timer->cb);
    mp_tid_expiry_timer->cb(TIMER_NOW_1, mp_tid_expiry_timer->p_context);
    TEST_ASSERT_NULL(mp_tid_expiry_timer->cb);
}

/******** Setup and Tear Down ********/
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

/******** Tests ********/
void test_model_tid_validate(void)
{
    /* Test: Initialize tid tracker item, this should always PASS */
    m_meta.src.value = 0x0123;
    m_meta.dst.value = 0x0001;
    m_tid = 0;
    m_opcode = OPCODE_1;
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Test: Validation fails, without timeout */
    TEST_ASSERT_FALSE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Trigger timer callback (6 seconds are over) */
    helper_timer_cb_trigger();

    /* Test: After callback triggers, same src, dst, tid result in True */
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Test: provide new values */
    m_meta.src.value = 0x0555;
    m_meta.dst.value = 0xCACA;
    m_tid++;
    m_opcode = OPCODE_2;
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Test: Validation fails, without timeout */
    TEST_ASSERT_FALSE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Trigger timer callback (6 seconds are over) */
    helper_timer_cb_trigger();

    /* Test: After callback triggers, same src, dst, tid result in True */
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Test: provide same values, but change opcode */
    m_opcode = OPCODE_1;
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Test: Validation fails, without timeout */
    TEST_ASSERT_FALSE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Trigger timer callback (6 seconds are over) */
    helper_timer_cb_trigger();

    /* Test: After callback triggers, same src, dst, tid result in True */
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));
}

void test_model_transaction_is_new(void)
{
    /* Test: Initialize tid tracker item, this should always PASS */
    m_meta.src.value = 0x0123;
    m_meta.dst.value = 0x0001;
    m_tid = 0;
    m_opcode = OPCODE_1;
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Transaction is new */
    TEST_ASSERT_TRUE(model_transaction_is_new(&m_tid_item));

    /* Test: Validation fails, without timeout */
    TEST_ASSERT_FALSE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Transaction is old */
    TEST_ASSERT_FALSE(model_transaction_is_new(&m_tid_item));

    /* Trigger timer callback (6 seconds are over) */
    helper_timer_cb_trigger();

    /* Test: After callback triggers, same src, dst, tid result in True */
    helper_tid_validate_expect(TIMER_NOW_1, m_meta.src.value, m_meta.dst.value, m_opcode, m_tid);
    TEST_ASSERT_TRUE(model_tid_validate(&m_tid_item, &m_meta, m_opcode, m_tid));

    /* Transaction is new */
    TEST_ASSERT_TRUE(model_transaction_is_new(&m_tid_item));
}

/**************************************************************************************************/
/* Model timer tests */

static void helper_model_timer_cb(void * p_context)
{
    model_timer_t * p_timer = (model_timer_t *) p_context;

    TEST_ASSERT_EQUAL(m_exp_model_timer.p_context, p_context);

    TEST_ASSERT_EQUAL(m_app_timer_count, p_timer->last_rtc_stamp);
    TEST_ASSERT_EQUAL(m_exp_model_timer.total_rtc_ticks, p_timer->total_rtc_ticks);
    m_exp_model_timer.last_rtc_stamp = m_app_timer_count;

    TEST_ASSERT_EQUAL(m_test_expected_active, model_timer_is_running(&m_model_timer));
}

static ret_code_t app_timer_create_mock(app_timer_id_t const * p_timer_id, app_timer_mode_t mode,
                                        app_timer_timeout_handler_t timeout_handler, int count)
{
    m_app_timer_id = *p_timer_id;
    m_app_timer_mode = mode;
    m_app_timer_cb = timeout_handler;

    TEST_ASSERT_EQUAL(mode, APP_TIMER_MODE_SINGLE_SHOT);
    TEST_ASSERT_NOT_NULL(timeout_handler);

    return NRF_SUCCESS;
}

static void helper_setup_model_timer(model_timer_mode_t mode, uint64_t timeout_rtc_ticks,
                                     void * p_context)
{
    m_model_timer.timeout_rtc_ticks = timeout_rtc_ticks;
    m_model_timer.p_context = p_context;
    m_model_timer.mode = mode;
    m_model_timer.cb = helper_model_timer_cb;
    m_model_timer.p_timer_id = &test_timer;
    m_model_timer.total_rtc_ticks = 0;
    m_model_timer.last_rtc_stamp = 0;

    m_exp_model_timer = m_model_timer;
    m_app_timer_count = 0;

    app_timer_stop_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    app_timer_cnt_get_ExpectAndReturn(m_app_timer_count);

    if (timeout_rtc_ticks > MODEL_TIMER_MAX_TIMEOUT_TICKS)
    {
        app_timer_start_ExpectAndReturn(*m_model_timer.p_timer_id,
                                         APP_TIMER_MAX_TIMEOUT,
                                         &m_model_timer, NRF_SUCCESS);

        m_app_timer_count = APP_TIMER_MAX_TIMEOUT;
    }
    else
    {
        app_timer_start_ExpectAndReturn(*m_model_timer.p_timer_id,
                                        timeout_rtc_ticks,
                                        &m_model_timer, NRF_SUCCESS);
        m_app_timer_count = timeout_rtc_ticks;
    }

}

static void helper_trigger_timer(void)
{
    app_timer_cnt_get_ExpectAndReturn(m_app_timer_count);
    app_timer_cnt_get_ExpectAndReturn(m_app_timer_count);
    app_timer_cnt_diff_compute_ExpectAnyArgsAndReturn(((m_app_timer_count - m_model_timer.last_rtc_stamp) & MAX_RTC_COUNTER_VAL));

    if (m_model_timer.remaining_ticks > 0 || m_model_timer.mode == MODEL_TIMER_MODE_REPEATED)
    {
        m_exp_model_timer.total_rtc_ticks = m_app_timer_count;
        app_timer_start_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    }
    else
    {
        m_exp_model_timer.total_rtc_ticks = m_app_timer_count;
    }

    TEST_ASSERT_NOT_NULL(m_app_timer_cb);
    m_app_timer_cb(&m_model_timer);

    /* Advance emulated time */
    if (m_model_timer.remaining_ticks > APP_TIMER_MAX_TIMEOUT)
    {
        m_app_timer_count += APP_TIMER_MAX_TIMEOUT;
    }
    else
    {
        m_app_timer_count += m_model_timer.remaining_ticks;
    }
}

void test_model_timer_create(void)
{
    /* test: Invalid inputs */
    memset(&m_model_timer, 0, sizeof(m_model_timer));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, model_timer_create(NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, model_timer_create(&m_model_timer));

    m_model_timer.timeout_rtc_ticks = TIMEOUT_TICKS_1;
    m_model_timer.p_context = (void *) TIMER_CONTEXT_1;
    m_model_timer.mode = MODEL_TIMER_MODE_SINGLE_SHOT;
    m_model_timer.cb = helper_model_timer_cb;
    m_model_timer.p_timer_id = &test_timer;
    m_test_expected_active = false;

    app_timer_create_StubWithCallback(app_timer_create_mock);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_create(&m_model_timer));
    TEST_ASSERT_EQUAL(false, model_timer_is_running(&m_model_timer));
}

void test_model_timer_schedule_single_shot()
{
    uint32_t req_timeout_ticks;

    /* test: Invalid inputs */
    memset(&m_model_timer, 0, sizeof(m_model_timer));

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, model_timer_schedule(NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, model_timer_schedule(&m_model_timer));

    m_model_timer.timeout_rtc_ticks = 1;
    m_model_timer.p_context = (void *) TIMER_CONTEXT_1;
    m_model_timer.mode = MODEL_TIMER_MODE_SINGLE_SHOT;
    m_model_timer.cb = helper_model_timer_cb;
    m_model_timer.p_timer_id = &test_timer;
    m_test_expected_active = false;

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, model_timer_schedule(&m_model_timer));

    memset(&m_model_timer, 0, sizeof(m_model_timer));
    helper_setup_model_timer(MODEL_TIMER_MODE_SINGLE_SHOT, TIMEOUT_TICKS_2, (void *)TIMER_CONTEXT_2);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_schedule(&m_model_timer));

    /* test: Schedule with valid params, with small delay, where timeout occurs immediately */
    req_timeout_ticks = 5000;
    helper_setup_model_timer(MODEL_TIMER_MODE_SINGLE_SHOT, req_timeout_ticks, &m_model_timer);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_schedule(&m_model_timer));
    TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));
    helper_trigger_timer();
    TEST_ASSERT_EQUAL(false, model_timer_is_running(&m_model_timer));

    /* test: Schedule with valid params, with a very long delay */
    req_timeout_ticks = APP_TIMER_MAX_TIMEOUT * 3 + 100 ;
    helper_setup_model_timer(MODEL_TIMER_MODE_SINGLE_SHOT, req_timeout_ticks, &m_model_timer);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_schedule(&m_model_timer));
    TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));
    helper_trigger_timer();

    for(uint32_t i = 0; i < (req_timeout_ticks/APP_TIMER_MAX_TIMEOUT); i++)
    {
        TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));
        helper_trigger_timer();
    }
    TEST_ASSERT_EQUAL(0, m_model_timer.remaining_ticks);
    TEST_ASSERT_EQUAL(false, model_timer_is_running(&m_model_timer));
}

void test_model_timer_schedule_repeat()
{
    uint32_t req_timeout_ticks;

    /* test: Invalid inputs */
    memset(&m_model_timer, 0, sizeof(m_model_timer));

    m_model_timer.timeout_rtc_ticks = 1;
    m_model_timer.p_context = (void *) TIMER_CONTEXT_1;
    m_model_timer.mode = MODEL_TIMER_MODE_REPEATED;
    m_model_timer.cb = helper_model_timer_cb;
    m_model_timer.p_timer_id = &test_timer;
    m_test_expected_active = true;

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, model_timer_schedule(&m_model_timer));
    TEST_ASSERT_EQUAL(false, model_timer_is_running(&m_model_timer));

    memset(&m_model_timer, 0, sizeof(m_model_timer));
    helper_setup_model_timer(MODEL_TIMER_MODE_REPEATED, TIMEOUT_TICKS_2, (void *)TIMER_CONTEXT_2);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_schedule(&m_model_timer));
    TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));

    /* test: Schedule with valid params, with small delay, where timeout occurs immediately */
    req_timeout_ticks = 5000;
    helper_setup_model_timer(MODEL_TIMER_MODE_REPEATED, req_timeout_ticks, &m_model_timer);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_schedule(&m_model_timer));
    TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));
    for(uint32_t i = 0; i < 10; i++)
    {
        helper_trigger_timer();
        TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));
    }
    TEST_ASSERT_EQUAL(0, m_model_timer.remaining_ticks);

    /* test: Schedule with valid params, with a very long delay */
    req_timeout_ticks = APP_TIMER_MAX_TIMEOUT * 3 + 100 ;
    helper_setup_model_timer(MODEL_TIMER_MODE_SINGLE_SHOT, req_timeout_ticks, &m_model_timer);
    m_test_expected_active = false;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_schedule(&m_model_timer));
    TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));

    for (uint32_t j = 0; j < 11; j++)
    {
        for(uint32_t i = 0; i < (req_timeout_ticks/APP_TIMER_MAX_TIMEOUT); i++)
        {
            helper_trigger_timer();
        }
        TEST_ASSERT_EQUAL(0, m_model_timer.remaining_ticks);
    }
    TEST_ASSERT_EQUAL(false, model_timer_is_running(&m_model_timer));
}

void test_model_timer_abort(void)
{
    memset(&m_model_timer, 0, sizeof(m_model_timer));

    helper_setup_model_timer(MODEL_TIMER_MODE_SINGLE_SHOT, TIMEOUT_TICKS_2, (void *)TIMER_CONTEXT_2);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, model_timer_schedule(&m_model_timer));
    TEST_ASSERT_EQUAL(true, model_timer_is_running(&m_model_timer));

    app_timer_stop_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    model_timer_abort(&m_model_timer);
    TEST_ASSERT_EQUAL(false, model_timer_is_running(&m_model_timer));

    TEST_ASSERT_EQUAL(0, m_model_timer.remaining_ticks);
    TEST_ASSERT_EQUAL(0, m_model_timer.timeout_rtc_ticks);
}

void test_model_timer_is_running(void)
{
    /* This is implicitly tested as a part of other functionality tests */
}
