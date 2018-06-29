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

#include <unity.h>
#include <cmock.h>
#include <string.h>

#include "nrf.h"
#include "test_assert.h"

#include "timeslot_mock.h"
#include "queue_mock.h"
#include "timer_mock.h"
#include "scanner_mock.h"
#include "nrf_mesh_cmsis_mock_mock.h"

#define DEFAULT_ACTION {action_start_cb, action_radio_irq_handler, NULL, 1000, NULL, {NULL, NULL}}

static void* mp_expected_args;
static timestamp_t m_time_now;
static uint32_t m_expected_start_calls;
static uint32_t m_expected_radio_irq_calls;
static bool m_end_action;
static NRF_RADIO_Type m_radio;
static NRF_TIMER_Type m_action_timer;
static NRF_TIMER_Type m_timeslot_timer;
static NRF_PPI_Type m_ppi;
NRF_RADIO_Type * NRF_RADIO = &m_radio;
NRF_TIMER_Type * NRF_TIMER2 = &m_action_timer;
NRF_TIMER_Type * NRF_TIMER0 = &m_timeslot_timer;
NRF_PPI_Type * NRF_PPI = &m_ppi;

void setUp(void)
{
    timeslot_mock_Init();
    queue_mock_Init();
    timer_mock_Init();
    scanner_mock_Init();
    nrf_mesh_cmsis_mock_mock_Init();

    m_expected_start_calls = 0;
    m_expected_radio_irq_calls = 0;
    mp_expected_args = NULL;
    m_end_action = false;

    NVIC_SetPriority_Expect(BEARER_ACTION_TIMER_IRQn, 0);
    queue_init_Expect(NULL);
    queue_init_IgnoreArg_p_queue();
    bearer_handler_init();
    timeslot_trigger_Expect();
    bearer_handler_start();
}

void tearDown(void)
{
    timeslot_mock_Verify();
    timeslot_mock_Destroy();
    queue_mock_Verify();
    queue_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    scanner_mock_Verify();
    scanner_mock_Destroy();
    nrf_mesh_cmsis_mock_mock_Verify();
    nrf_mesh_cmsis_mock_mock_Destroy();
}

static void action_start_cb(timestamp_t start_time, void* p_args)
{
    TEST_ASSERT_EQUAL(m_time_now, start_time);
    TEST_ASSERT_EQUAL(mp_expected_args, p_args);
    TEST_ASSERT_NOT_EQUAL(0, m_expected_start_calls);
    m_expected_start_calls--;
    if (m_end_action)
    {
        bearer_handler_action_end();
    }
}

static void action_radio_irq_handler(void* p_args)
{
    TEST_ASSERT_EQUAL(mp_expected_args, p_args);
    TEST_ASSERT_NOT_EQUAL(0, m_expected_radio_irq_calls);
    m_expected_radio_irq_calls--;
    if (m_end_action)
    {
        bearer_handler_action_end();
    }
}

static void m_queue_peek_fail(void)
{
    queue_peek_ExpectAndReturn(NULL, NULL);
    queue_peek_IgnoreArg_p_queue();
}

static void m_timer_setup_expect(uint32_t start_time)
{
    timeslot_start_time_get_ExpectAndReturn(start_time - 10);
    NRF_TIMER0->CC[2] = 10 + 1;
    NVIC_ClearPendingIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
}

static void m_radio_irq_clear_expect(void)
{
    NVIC_ClearPendingIRQ_Expect(RADIO_IRQn);
}

static void m_scanner_start_expect(uint32_t start_time)
{
    timeslot_end_is_pending_ExpectAndReturn(false);
    m_queue_peek_fail();
    timeslot_remaining_time_get_ExpectAndReturn(1000);
    m_radio_irq_clear_expect();
    m_timer_setup_expect(start_time);
    scanner_radio_start_Expect(start_time);
}

static void m_action_end_expect(void)
{
    timer_now_ExpectAndReturn(m_time_now);
    timeslot_state_lock_Expect(false);
}

static void m_setup_actions(bearer_action_t* p_actions, uint32_t count)
{
    bearer_action_t default_action = DEFAULT_ACTION;
    for (uint32_t i = 0; i < count; ++i)
    {
        memcpy(&p_actions[i], &default_action, sizeof(bearer_action_t));
        p_actions[i].p_args = (void*) &p_actions[i];
        p_actions[i].queue_elem.p_data = NULL;
    }
}

static void m_action_pop_Expect(bearer_action_t * p_action, uint32_t action_duration)
{
    p_action->queue_elem.p_data = p_action;
    p_action->duration_us = action_duration;
    queue_peek_ExpectAndReturn(NULL, &p_action->queue_elem);
    queue_peek_IgnoreArg_p_queue();
    queue_pop_ExpectAndReturn(NULL, &p_action->queue_elem);
    queue_pop_IgnoreArg_p_queue();
}

static void m_action_start_expect(bearer_action_t * p_action, uint32_t start_time, uint32_t duration)
{
    m_expected_start_calls = 1;
    mp_expected_args = p_action;
    timeslot_end_is_pending_ExpectAndReturn(false);
    m_action_pop_Expect(p_action, duration);
    timeslot_remaining_time_get_ExpectAndReturn(duration + 1000);
    m_radio_irq_clear_expect();
    m_timer_setup_expect(start_time);
    timeslot_state_lock_Expect(true);
}

/******** Tests ********/
void test_enqueue_action(void)
{
    bearer_action_t action[6] = {DEFAULT_ACTION, DEFAULT_ACTION, DEFAULT_ACTION, DEFAULT_ACTION, DEFAULT_ACTION, DEFAULT_ACTION};

    queue_push_Expect(NULL, &action[0].queue_elem);
    queue_push_IgnoreArg_p_queue();
    timeslot_trigger_Expect();
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, bearer_handler_action_enqueue(&action[0]));
    TEST_ASSERT_EQUAL_PTR(&action[0], action[0].queue_elem.p_data);

    /* Push a second action to ensure this doesn't break it */
    queue_push_Expect(NULL, &action[1].queue_elem);
    queue_push_IgnoreArg_p_queue();
    timeslot_trigger_Expect();
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, bearer_handler_action_enqueue(&action[1]));

    /* Push the already enqueued action, should detect that it's already enqueued. */
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_STATE, bearer_handler_action_enqueue(&action[1]));

    /* The longest duration call should be pushable */
    action[2].duration_us = BEARER_ACTION_DURATION_MAX_US;
    queue_push_Expect(NULL, &action[2].queue_elem);
    queue_push_IgnoreArg_p_queue();
    timeslot_trigger_Expect();
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, bearer_handler_action_enqueue(&action[2]));

    /* Test asserts: */
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_enqueue(NULL));
    action[3].start_cb = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_enqueue(&action[3]));
    action[4].duration_us = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_enqueue(&action[4]));
    action[5].duration_us = BEARER_ACTION_DURATION_MAX_US + 1;
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_enqueue(&action[5]));
}

void test_signal_handler(void)
{
    bearer_action_t action[2];
    m_setup_actions(action, 2);

    /** Wake up with nothing in the queue, the scanner should start. */
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_scanner_start_expect(100);

    bearer_handler_timer_irq_handler();

    /** Wake up with nothing in the queue, the scanner should not be called again. */
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_queue_peek_fail();
    timeslot_remaining_time_get_ExpectAndReturn(5000);
    timeslot_end_is_pending_ExpectAndReturn(false);

    bearer_handler_timer_irq_handler();
    scanner_mock_Verify();
    queue_mock_Verify();

    /** Wake up with event in queue, while the scanner is running */
    m_time_now = 1234;
    timeslot_is_in_cb_ExpectAndReturn(true);
    scanner_radio_stop_Expect();
    m_action_start_expect(&action[0], m_time_now, 1000);

    bearer_handler_timer_irq_handler();

    TEST_ASSERT_EQUAL(0, m_expected_start_calls);
    queue_mock_Verify();

    /* Enqueue a second event */
    queue_push_Expect(NULL, &action[1].queue_elem);
    queue_push_IgnoreArg_p_queue();
    /* Don't expect trigger this time, as an action is already running. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, bearer_handler_action_enqueue(&action[1]));

    /* Nothing happens on the signal handler when an action is already running */
    timeslot_is_in_cb_ExpectAndReturn(true);
    bearer_handler_timer_irq_handler();

    /* Test asserts */
    timeslot_is_in_cb_ExpectAndReturn(false);
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_timer_irq_handler());
}

void test_end_action(void)
{
    bearer_action_t action[4];
    m_setup_actions(action, 4);

    /* Trigger signal handler to start the current action. */
    m_time_now = 1234;
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_start_expect(&action[0], m_time_now, 1000);

    bearer_handler_timer_irq_handler();
    TEST_ASSERT_EQUAL(0, m_expected_start_calls);

    /* End the action, setup the next */
    m_time_now = 2100;
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_end_expect();
    m_action_start_expect(&action[1], m_time_now, 1000);

    bearer_handler_action_end();
    TEST_ASSERT_EQUAL(0, m_expected_start_calls);

    /* End the action, no more events, can fit scanner action */
    m_time_now = 3000;
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_end_expect();
    m_scanner_start_expect(m_time_now);

    bearer_handler_action_end();

    /* Trigger signal handler to start an action. */
    m_time_now = 1234;
    timeslot_is_in_cb_ExpectAndReturn(true);
    scanner_radio_stop_Expect();
    m_action_start_expect(&action[2], m_time_now, 1000);

    bearer_handler_timer_irq_handler();
    TEST_ASSERT_EQUAL(0, m_expected_start_calls);

    /* End the action, no more events, can NOT fit scanner action */
    m_time_now = 2000;
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_end_expect();
    m_queue_peek_fail();
    timeslot_remaining_time_get_ExpectAndReturn(100);
    timeslot_end_is_pending_ExpectAndReturn(false);

    bearer_handler_action_end();

    /* Trigger signal handler to start an action. Timer will roll over during the session. */
    m_time_now = (uint32_t) -500;/* slot will end at 1500 */
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_start_expect(&action[3], m_time_now, 2000);

    bearer_handler_timer_irq_handler();
    TEST_ASSERT_EQUAL(0, m_expected_start_calls);
    queue_mock_Verify();
    scanner_mock_Verify();

    /* End the action, no more events, can fit scanner action */
    m_time_now = 1000;
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_end_expect();
    m_scanner_start_expect(m_time_now);

    bearer_handler_action_end();

    /* End action from within the start callback */
    m_time_now = 1234;
    m_end_action = true;
    m_setup_actions(action, 1);

    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_start_expect(&action[0], m_time_now, 1000);
    scanner_radio_stop_Expect();

    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_end_expect();
    m_scanner_start_expect(m_time_now);

    bearer_handler_timer_irq_handler();
    TEST_ASSERT_EQUAL(0, m_expected_start_calls);

    /* setup new event */
    m_time_now = 1234;
    m_end_action = false;
    m_setup_actions(action, 1);

    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_start_expect(&action[0], m_time_now, 1000);
    scanner_radio_stop_Expect();

    bearer_handler_timer_irq_handler();

    /* End from within the radio irq handler */
    m_end_action = true;
    m_expected_radio_irq_calls = 1;

    timeslot_is_in_cb_ExpectAndReturn(true); /* One for the radio IRQ handler */
    timeslot_is_in_cb_ExpectAndReturn(true); /* One for the action end */
    m_action_end_expect();
    m_scanner_start_expect(m_time_now);

    bearer_handler_radio_irq_handler();

    /* Test asserts */

    /* No action is running, assert */
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_end());
}

void test_fire_action(void)
{
    bearer_action_t action[2];
    m_setup_actions(action, 2);

    /* Successful fire: */
    m_queue_peek_fail();
    timeslot_is_in_ts_ExpectAndReturn(true);
    timeslot_remaining_time_get_ExpectAndReturn(10000);
    queue_push_Expect(NULL, &action[0].queue_elem);
    queue_push_IgnoreArg_p_queue();
    timeslot_trigger_Expect();
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, bearer_handler_action_fire(&action[0]));
    TEST_ASSERT_EQUAL_PTR(&action[0], action[0].queue_elem.p_data);
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_STATE, bearer_handler_action_enqueue(&action[0]));

    /* Failed because an action is already enqueued: */
    queue_peek_ExpectAndReturn(NULL, &action[0].queue_elem);
    queue_peek_IgnoreArg_p_queue();
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_BUSY, bearer_handler_action_fire(&action[1]));


    /* Failed because we're not in a timeslot: */
    m_queue_peek_fail();
    timeslot_is_in_ts_ExpectAndReturn(false);
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_BUSY, bearer_handler_action_fire(&action[1]));

    /* Failed because we don't have enough time left in the timeslot: */
    m_queue_peek_fail();
    timeslot_is_in_ts_ExpectAndReturn(true);
    timeslot_remaining_time_get_ExpectAndReturn(action[1].duration_us - 1);
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_BUSY, bearer_handler_action_fire(&action[1]));

    /* Start an action */
    m_time_now = 1234;
    mp_expected_args = &action[0];
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_start_expect(&action[0], m_time_now, 1000);

    bearer_handler_timer_irq_handler();
    TEST_ASSERT_EQUAL(0, m_expected_start_calls);

    /* Failed because an event is already running: */
    /* doesn't matter if our other checkers are called */
    queue_peek_IgnoreAndReturn(&action[1].queue_elem);
    timeslot_is_in_ts_IgnoreAndReturn(true);
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_BUSY, bearer_handler_action_fire(&action[1]));

    /* Test asserts: */
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_fire(NULL));
    action[0].start_cb = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_fire(&action[0]));
    action[0].start_cb = action_start_cb;
    action[0].duration_us = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_fire(&action[0]));
    action[0].duration_us = BEARER_ACTION_DURATION_MAX_US + 1;
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_action_fire(&action[0]));
}

void test_ts_start_end(void)
{
    bearer_action_t action[2];
    m_setup_actions(action, 2);

    /* Test asserts */
    timeslot_is_in_cb_ExpectAndReturn(false);
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_on_ts_begin());

    /* Start scanner if action queue is empty */
    timeslot_is_in_cb_ExpectAndReturn(true);
    NVIC_EnableIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
    NVIC_EnableIRQ_Expect(RADIO_IRQn);
    m_scanner_start_expect(m_time_now);
    bearer_handler_on_ts_begin();

    /* Reset state */
    NVIC_SetPriority_Expect(BEARER_ACTION_TIMER_IRQn, 0);
    queue_init_Expect(NULL);
    queue_init_IgnoreArg_p_queue();
    bearer_handler_init();
    timeslot_trigger_Expect();
    bearer_handler_start();

    /* Start action if any in queue */
    m_time_now = 2000;
    timeslot_is_in_cb_ExpectAndReturn(true);
    NVIC_EnableIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
    NVIC_EnableIRQ_Expect(RADIO_IRQn);
    m_action_start_expect(&action[0], m_time_now, 1500);

    bearer_handler_on_ts_begin();
    TEST_ASSERT_EQUAL_INT(0, m_expected_start_calls);

    /* Assert if timeslot ends in the middle of an action */
    timeslot_is_in_cb_ExpectAndReturn(true);
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_on_ts_end());

    /* End the action */
    m_time_now = 3000;
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_action_end_expect();
    m_scanner_start_expect(m_time_now);

    bearer_handler_action_end();
    TEST_ASSERT_EQUAL_INT(0, m_expected_start_calls);

    /* Assert if called outside of signal callback */
    timeslot_is_in_cb_ExpectAndReturn(false);
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_on_ts_end());

    /* Actually end the timeslot */
    memset(NRF_TIMER2, 0, sizeof(*NRF_TIMER2));
    memset(NRF_RADIO, 0, sizeof(*NRF_RADIO));

    timeslot_is_in_cb_ExpectAndReturn(true);
    scanner_radio_stop_Expect();
    NVIC_ClearPendingIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
    m_radio_irq_clear_expect();
    NVIC_DisableIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
    bearer_handler_on_ts_end();

    // ensure that the state has actually been wiped
    TEST_ASSERT_EQUAL(0xFFFFFFFF, NRF_TIMER2->INTENCLR);
    TEST_ASSERT_EQUAL(1, NRF_TIMER2->TASKS_STOP);
    TEST_ASSERT_EQUAL(1, NRF_TIMER2->TASKS_CLEAR);
    TEST_ASSERT_EQUAL(0xFFFFFFFF, NRF_RADIO->INTENCLR);

    /* Call end again, should cause another clearing */
    timeslot_is_in_cb_ExpectAndReturn(true);
    NVIC_ClearPendingIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
    m_radio_irq_clear_expect();
    NVIC_DisableIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
    bearer_handler_on_ts_end();
}

void test_radio_irq(void)
{
    /* No action, no scanner */
    timeslot_is_in_cb_ExpectAndReturn(true);
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_radio_irq_handler());
    /* Not in cb */
    timeslot_is_in_cb_ExpectAndReturn(false);
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_radio_irq_handler());

    /* Start scanner */
    timeslot_is_in_cb_ExpectAndReturn(true);
    NVIC_EnableIRQ_Expect(BEARER_ACTION_TIMER_IRQn);
    NVIC_EnableIRQ_Expect(RADIO_IRQn);
    m_scanner_start_expect(0);

    bearer_handler_on_ts_begin();

    /* forward interrupt to scanner */
    timeslot_is_in_cb_ExpectAndReturn(true);
    scanner_radio_irq_handler_Expect();
    bearer_handler_radio_irq_handler();

    /* Start an action */
    bearer_action_t action[1];
    m_setup_actions(action, 1);
    m_time_now = 1234;

    timeslot_is_in_cb_ExpectAndReturn(true);
    scanner_radio_stop_Expect();
    m_action_start_expect(&action[0], m_time_now, 1000);

    bearer_handler_timer_irq_handler();
    TEST_ASSERT_EQUAL(0, m_expected_start_calls);

    /* forward interrupt to action */
    timeslot_is_in_cb_ExpectAndReturn(true);
    m_expected_radio_irq_calls = 1;
    bearer_handler_radio_irq_handler();
    TEST_ASSERT_EQUAL(0, m_expected_radio_irq_calls);

    /* Don't call NULL pointer IRQ handler in action */
    action[0].radio_irq_handler = NULL;
    timeslot_is_in_cb_ExpectAndReturn(true);
    TEST_NRF_MESH_ASSERT_EXPECT(bearer_handler_radio_irq_handler());
    TEST_ASSERT_EQUAL(0, m_expected_radio_irq_calls);
}
