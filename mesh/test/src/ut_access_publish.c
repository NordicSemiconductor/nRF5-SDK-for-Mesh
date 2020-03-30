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

#include <unity.h>
#include <cmock.h>
#include <string.h>

#include "bearer_event_mock.h"
#include "timer_scheduler_mock.h"

#include "utils.h"
#include "timer.h"

#include "access_config.h"
#include "access_publish.h"

/*******************************************************************************
 * Static Variables
 *******************************************************************************/

static uint32_t m_publish_timeout_cb_called;
static access_model_handle_t m_publish_timeout_cb_handle;

static timestamp_t m_current_timestamp;

static timer_event_t * mp_scheduled_event = NULL;
static uint32_t timer_sch_reschedule_mock_called;

/*******************************************************************************
 * Helper Functions // Mocks // Callbacks
 *******************************************************************************/

timestamp_t timer_now()
{
    return m_current_timestamp;
}

static void timer_sch_schedule_mock(timer_event_t * p_event, int num_calls)
{
    if (mp_scheduled_event != NULL)
    {
        TEST_FAIL_MESSAGE("trying to schedule a new timer scheduler event while one is already scheduled");
    }

    mp_scheduled_event = p_event;
}

static void timer_sch_reschedule_mock(timer_event_t * p_event, timestamp_t new_timestamp, int num_calls)
{
    ++timer_sch_reschedule_mock_called;

    if (mp_scheduled_event == NULL)
    {
        mp_scheduled_event = p_event;
    }

    TEST_ASSERT_EQUAL_PTR(mp_scheduled_event, p_event);
    mp_scheduled_event->timestamp = new_timestamp;
}

static void timer_sch_schedule_mock_trigger(void)
{
    if (!mp_scheduled_event)
    {
        TEST_FAIL_MESSAGE("cannot trigger timer scheduler event when none is scheduled");
    }

    timer_event_t * p_event = mp_scheduled_event;
    mp_scheduled_event = NULL;
    m_current_timestamp = p_event->timestamp;
    p_event->cb(p_event->timestamp, p_event->p_context);
}

static void publish_timeout_cb(access_model_handle_t handle, void * p_args)
{
    ++m_publish_timeout_cb_called;
    m_publish_timeout_cb_handle = handle;
}

uint32_t access_model_p_args_get(access_model_handle_t handle, void ** pp_args)
{
    return NRF_SUCCESS;
}

/*******************************************************************************
 * Test Setup
 *******************************************************************************/

void setUp(void)
{
    timer_scheduler_mock_Init();
    bearer_event_mock_Init();

    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    m_current_timestamp = 0;
    mp_scheduled_event = NULL;
    timer_sch_reschedule_mock_called = 0;

    access_publish_init();
}

void tearDown(void)
{
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
}


/*****************************************************************************
 * Tests
 *****************************************************************************/

void test_periodic_publishing_singlemodel(void)
{
    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock);

    access_model_publication_state_t test_pubstate;
    test_pubstate.publish_timeout_cb = publish_timeout_cb;

    const access_publish_resolution_t resolutions[] =
        { ACCESS_PUBLISH_RESOLUTION_100MS, ACCESS_PUBLISH_RESOLUTION_1S, ACCESS_PUBLISH_RESOLUTION_10S, ACCESS_PUBLISH_RESOLUTION_10MIN };

    for (uint8_t res_index = 0; res_index < ARRAY_SIZE(resolutions); ++res_index)
    {
        for (uint8_t steps = 1; steps <= 0x3f; ++steps)
        {
            /* Reset mocks for the iteration: */
            m_publish_timeout_cb_called = 0;
            mp_scheduled_event = NULL;
            m_current_timestamp = 0;

            /* Re-initialize the publication module to reset the internal counters and lists: */
            access_publish_init();

            /* Schedule the periodic publishing event: */
            access_publish_period_set(&test_pubstate, resolutions[res_index], steps);

            /* Trigger the timer scheduler interrupt for the required number of steps until the event fires: */
            for (uint8_t i = 1; i < steps; ++i)
            {
                timer_sch_schedule_mock_trigger();
                TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);

                /* Ensure that the publishing event is still scheduled: */
                TEST_ASSERT_NOT_NULL(mp_scheduled_event);
            }

            TEST_ASSERT_NOT_NULL(mp_scheduled_event);

            /* Trigger the publishing event: */
            m_publish_timeout_cb_called = 0;
            timer_sch_schedule_mock_trigger();
            TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
            TEST_ASSERT_NOT_NULL(mp_scheduled_event);
        }
    }
}

void test_periodic_publishing_multimodel(void)
{
    access_model_publication_state_t test_pubstate_1, test_pubstate_2;

    memset(&test_pubstate_1, 0, sizeof(test_pubstate_1));
    test_pubstate_1.publish_timeout_cb = publish_timeout_cb;

    memset(&test_pubstate_2, 0, sizeof(test_pubstate_2));
    test_pubstate_2.publish_timeout_cb = publish_timeout_cb;

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock);

    /* Test scheduling two models to publish at the same time: */
    const access_publish_resolution_t resolutions[] =
        { ACCESS_PUBLISH_RESOLUTION_100MS, ACCESS_PUBLISH_RESOLUTION_1S, ACCESS_PUBLISH_RESOLUTION_10S, ACCESS_PUBLISH_RESOLUTION_10MIN };
    for (uint8_t res_index = 0; res_index < ARRAY_SIZE(resolutions); ++res_index)
    {
        for (uint8_t steps = 1; steps <= 0x3f; ++steps)
        {
            /* Reset mocks and the publication module for the iteration: */
            m_publish_timeout_cb_called = 0;
            mp_scheduled_event = NULL;
            m_current_timestamp = 0;
            access_publish_init();

            /* Schedule the periodic publishing event: */
            access_publish_period_set(&test_pubstate_1, resolutions[res_index], steps);
            access_publish_period_set(&test_pubstate_2, resolutions[res_index], steps);

            /* Trigger the timer scheduler interrupt for the required number of steps until the event fires: */
            for (uint8_t i = 1; i < steps; ++i)
            {
                timer_sch_schedule_mock_trigger();
                TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);

                /* Ensure that the publishing event is still scheduled: */
                TEST_ASSERT_NOT_NULL(mp_scheduled_event);
            }

            TEST_ASSERT_NOT_NULL(mp_scheduled_event);

            /* Trigger the publishing event: */
            m_publish_timeout_cb_called = 0;
            timer_sch_schedule_mock_trigger();
            TEST_ASSERT_EQUAL(2, m_publish_timeout_cb_called);
            TEST_ASSERT_NOT_NULL(mp_scheduled_event);
        }
    }

    /* Reset mocks and the access layer for further testing: */
    mp_scheduled_event = NULL;
    m_current_timestamp = 0;
    access_publish_init();

    /* Set up the models for testing: */
    test_pubstate_1.model_handle = 1;
    test_pubstate_2.model_handle = 2;

    /* Test scheduling two models, the first will trigger first, the second will trigger second: */
    access_publish_period_set(&test_pubstate_1, ACCESS_PUBLISH_RESOLUTION_1S, 9);
    access_publish_period_set(&test_pubstate_2, ACCESS_PUBLISH_RESOLUTION_10S, 1);

    /* Wait until the first event is ready to trigger: */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    for (uint8_t i = 0; i < 8; ++i)
    {
        timer_sch_schedule_mock_trigger();
        TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
        TEST_ASSERT_NOT_NULL(mp_scheduled_event);
    }

    /* Trigger the first event */
    m_publish_timeout_cb_called = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_handle);

    /* Trigger the second event a second later: */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(2, m_publish_timeout_cb_handle);

    /* Check that they are both rescheduled correctly by doing another cycle: */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    for (uint8_t i = 0; i < 7; ++i)
    {
        timer_sch_schedule_mock_trigger();
        TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
        TEST_ASSERT_NOT_NULL(mp_scheduled_event);
    }

    /* Trigger the first event: */
    m_publish_timeout_cb_called = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_handle);

    /* One tick remains before the second event will trigger: */
    m_publish_timeout_cb_called = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);

    /* Trigger the second event: */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(2, m_publish_timeout_cb_handle);
}

void test_periodic_publishing_rescheduling(void)
{
    access_model_publication_state_t test_pubstate[3];

    for (int i = 0; i < 3; ++i)
    {
        memset(&test_pubstate[i], 0, sizeof(access_model_publication_state_t));
        test_pubstate[i].publish_timeout_cb = publish_timeout_cb;
        test_pubstate[i].model_handle = i;
    }

    /* Reset mocks: */
    m_publish_timeout_cb_called = 0;
    mp_scheduled_event = NULL;
    m_current_timestamp = 0;
    access_publish_init();

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock);
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_mock);

    /* Schedule the three models to publish after one another: */
    access_publish_period_set(&test_pubstate[0], ACCESS_PUBLISH_RESOLUTION_100MS, 4);
    access_publish_period_set(&test_pubstate[1], ACCESS_PUBLISH_RESOLUTION_100MS, 5);
    access_publish_period_set(&test_pubstate[2], ACCESS_PUBLISH_RESOLUTION_100MS, 6);
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);

    /* Do two timer ticks: */
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    /* Reschedule test_pubstate1] to be the first to trigger: */
    timer_sch_reschedule_mock_called = 0;
    m_publish_timeout_cb_handle = 0;
    access_publish_period_set(&test_pubstate[1], ACCESS_PUBLISH_RESOLUTION_100MS, 1);
    TEST_ASSERT_EQUAL(1, timer_sch_reschedule_mock_called);

    /* Trigger the timer and see what happens: */
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_handle);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    /* Now both test model 0 and 1 should trigger at the next timer tick: */
    m_publish_timeout_cb_called = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(2, m_publish_timeout_cb_called);

    /* Reschedule the first event in the list: */
    access_publish_period_set(&test_pubstate[1], ACCESS_PUBLISH_RESOLUTION_100MS, 6);

    /* The next event to fire now should be test_pubstate[2], at the 2nd tick from now. */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);

    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(2, m_publish_timeout_cb_handle);

    /* Then test_pubstate[0] will fire on the 2nd tick from now: */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);

    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_handle);

    /* And then test_pubstate[1] will fire on the 2nd tick from now: */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);

    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_handle);

    /* At this point, test_pubstate[1] is the last element in the list. Reschedule it to be first: */
    access_publish_period_set(&test_pubstate[1], ACCESS_PUBLISH_RESOLUTION_100MS, 1);

    /* The next event to fire now should be test_pubstate[1], scheduled for the next 100 ms tick: */
    m_publish_timeout_cb_called = 0;
    m_publish_timeout_cb_handle = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_handle);
}

void test_periodic_publishing_add_with_reschedule(void)
{
    access_model_publication_state_t test_pubstate_1, test_pubstate_2;

    memset(&test_pubstate_1, 0, sizeof(test_pubstate_1));
    test_pubstate_1.publish_timeout_cb = publish_timeout_cb;

    memset(&test_pubstate_2, 0, sizeof(test_pubstate_2));
    test_pubstate_2.publish_timeout_cb = publish_timeout_cb;

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock);
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_mock);

    /* Reset mocks and the publication module: */
    mp_scheduled_event = NULL;
    m_current_timestamp = 0;
    access_publish_init();

    /* Set up the models for testing: */
    test_pubstate_1.model_handle = 1;
    test_pubstate_2.model_handle = 2;

    /* Check that the timer has been rescheduled after adding the second publicatoin state: */
    timer_sch_reschedule_mock_called = 0;

    /* Test scheduling two publications, the second will trigger first, the first will trigger second: */
    access_publish_period_set(&test_pubstate_1, ACCESS_PUBLISH_RESOLUTION_1S, 2);
    access_publish_period_set(&test_pubstate_2, ACCESS_PUBLISH_RESOLUTION_1S, 1);
    TEST_ASSERT_EQUAL(1, timer_sch_reschedule_mock_called);

    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_EQUAL(2, m_publish_timeout_cb_handle);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    m_publish_timeout_cb_called = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(2, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);
}

void test_cancelling_publication(void)
{
    access_model_publication_state_t test_pubstate_1, test_pubstate_2;

    memset(&test_pubstate_1, 0, sizeof(test_pubstate_1));
    test_pubstate_1.publish_timeout_cb = publish_timeout_cb;

    memset(&test_pubstate_2, 0, sizeof(test_pubstate_2));
    test_pubstate_2.publish_timeout_cb = publish_timeout_cb;

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock);
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_mock);

    /* Reset mocks and the publication module: */
    mp_scheduled_event = NULL;
    m_current_timestamp = 0;
    timer_sch_reschedule_mock_called = 0;
    access_publish_init();

    /* Set up the models for testing: */
    test_pubstate_1.model_handle = 1;
    test_pubstate_2.model_handle = 2;

    /* Test scheduling two publications, the first will trigger first, the second will trigger second: */
    access_publish_period_set(&test_pubstate_1, ACCESS_PUBLISH_RESOLUTION_1S, 2);
    access_publish_period_set(&test_pubstate_2, ACCESS_PUBLISH_RESOLUTION_1S, 5);

    /* Give the timer one tick, the first event is then scheduled to trigger at the next timer tick: */
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    /* Disable the first event: */
    access_publish_period_set(&test_pubstate_1, ACCESS_PUBLISH_RESOLUTION_1S, 0);
    TEST_ASSERT_EQUAL(1, timer_sch_reschedule_mock_called);

    /* Ensure the event does not trigger: */
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    /* Re-add the first event at the start of the list: */
    timer_sch_reschedule_mock_called = 0;
    access_publish_period_set(&test_pubstate_1, ACCESS_PUBLISH_RESOLUTION_1S, 2);
    TEST_ASSERT_EQUAL(1, timer_sch_reschedule_mock_called);

    /* Give the timer one tick, the event added above will then trigger on the next tick: */
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    /* Disable the second publication event: */
    timer_sch_reschedule_mock_called = 0;
    access_publish_period_set(&test_pubstate_2, ACCESS_PUBLISH_RESOLUTION_1S, 0);
    TEST_ASSERT_EQUAL(0, timer_sch_reschedule_mock_called);

    /* Trigger the remaining event: */
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(1, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    /* Give the timer another tick: */
    m_publish_timeout_cb_called = 0;
    timer_sch_schedule_mock_trigger();
    TEST_ASSERT_EQUAL(0, m_publish_timeout_cb_called);
    TEST_ASSERT_NOT_NULL(mp_scheduled_event);

    /* Disable the remaining publication event as well: */
    timer_sch_abort_Expect(mp_scheduled_event);
    access_publish_period_set(&test_pubstate_1, ACCESS_PUBLISH_RESOLUTION_1S, 0);
    timer_scheduler_mock_Verify();
}

void test_publish_period_get(void)
{
    access_model_publication_state_t test_state;
    test_state.period.step_res = ACCESS_PUBLISH_RESOLUTION_10MIN;
    test_state.period.step_num = 42;

    access_publish_resolution_t resolution = ACCESS_PUBLISH_RESOLUTION_1S;
    uint8_t step_number = 0;

    access_publish_period_get(&test_state, &resolution, &step_number);
    TEST_ASSERT_EQUAL(ACCESS_PUBLISH_RESOLUTION_10MIN, resolution);
    TEST_ASSERT_EQUAL(42, step_number);
}

void test_access_publish_clear(void)
{
    access_model_publication_state_t test_state;
    test_state.period.step_res = ACCESS_PUBLISH_RESOLUTION_10MIN;
    test_state.period.step_num = 42;
    test_state.publish_timeout_cb = publish_timeout_cb;

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock);
    access_publish_period_set(&test_state, ACCESS_PUBLISH_RESOLUTION_1S, 5);

    timer_sch_abort_Expect(mp_scheduled_event);
    access_publish_clear();
}
