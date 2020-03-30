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
#include <stdlib.h>

#include "mesh_mem_mock.h"
#include "timer_scheduler_mock.h"
#include "access_internal_mock.h"

#include "manual_mock_queue.h"

#include "utils.h"
#include "test_assert.h"
#include "timer.h"

#include "access_publish_retransmission.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/

#define TX_MESSAGE_BUF "\x01\x02\x03\x04"
#define TX_MESSAGE_BUF_LEN (sizeof(TX_MESSAGE_BUF))
#define TX_MESSAGE_ACCESS_TOKEN 0x1234

#define ACCESS_MESSAGE_TX_INIT(msg) \
    do { \
        (msg).opcode.opcode = 0x0001; \
        (msg).opcode.company_id = 0x0001; \
        (msg).p_buffer = (uint8_t *) TX_MESSAGE_BUF; \
        (msg).length = TX_MESSAGE_BUF_LEN; \
        (msg).force_segmented = false; \
        (msg).transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT; \
        (msg).access_token = TX_MESSAGE_ACCESS_TOKEN; \
    } while (0)

#define GET_INTERVAL_US(step) (MS_TO_US(((step) + 1) * 50))
#define GET_NEXT_TIMEOUT_US(step, retry) ((retry + 1) * GET_INTERVAL_US(step))

#define EXPECT(name, times) \
    do { \
        for (size_t i = 0; i < times; i++) \
        { \
            uint32_t value = 0; \
            name##_Expect(&value); \
        } \
    } while(0)

#define CONSUME(name) \
    do { \
        uint32_t value; \
        name##_Consume(&value); \
    } while(0)

/*******************************************************************************
 * Static Variables
 *******************************************************************************/

typedef struct
{
    access_message_tx_t tx_message;
    uint8_t *p_access_payload;
    uint16_t access_payload_length;
    bool is_expected;
} sched_msg;

static timestamp_t m_current_timestamp;
static sched_msg m_expected_msgs_to_be_published[ACCESS_MODEL_COUNT];
static timestamp_t m_expected_next_timeout_us;
static timer_event_t *m_p_timer_evt;

//UTEST_FUNC_OBSERVE(timer_sch_reschedule_mock);
MOCK_QUEUE_DEF(timer_sch_reschedule_mock, uint32_t, NULL);
//UTEST_FUNC_OBSERVE(mesh_mem_free_mock);
MOCK_QUEUE_DEF(mesh_mem_free_mock, uint32_t, NULL);

/*******************************************************************************
 * Helper Functions // Mocks // Callbacks
 *******************************************************************************/

timestamp_t timer_now()
{
    return m_current_timestamp;
}

static uint32_t access_packet_tx_mock(access_model_handle_t handle,
                                      const access_message_tx_t * p_tx_message,
                                      const uint8_t *p_access_payload,
                                      uint16_t access_payload_length,
                                      int num_calls)
{
    UNUSED_VARIABLE(num_calls);

    TEST_ASSERT(m_expected_msgs_to_be_published[handle].is_expected);

    /* Reset the flag */
    m_expected_msgs_to_be_published[handle].is_expected = false;

    TEST_ASSERT_EQUAL_MEMORY(&m_expected_msgs_to_be_published[handle].tx_message,
                             p_tx_message,
                             sizeof(access_message_tx_t));

    TEST_ASSERT_EQUAL(m_expected_msgs_to_be_published[handle].access_payload_length,
                      access_payload_length);

    TEST_ASSERT_EQUAL(m_expected_msgs_to_be_published[handle].p_access_payload,
                      p_access_payload);

    return NRF_SUCCESS;
}

static void mesh_mem_free_mock(void * ptr, int num_calls)
{
    UNUSED_VARIABLE(ptr);
    UNUSED_VARIABLE(num_calls);

    CONSUME(mesh_mem_free_mock);
}

static void timer_sch_reschedule_mock(timer_event_t* p_timer_evt,
                                      timestamp_t new_timestamp,
                                      int num_calls)
{
    UNUSED_VARIABLE(num_calls);

    TEST_ASSERT_EQUAL(m_expected_next_timeout_us, new_timestamp);

    CONSUME(timer_sch_reschedule_mock);

    m_p_timer_evt = p_timer_evt;
}

static timestamp_t add_message(access_model_handle_t model_handle,
                               const access_publish_retransmit_t *p_publish_retransmit,
                               access_message_tx_t *p_tx_message,
                               bool expect_reschedule)
{
    timestamp_t next_timeout_us = GET_NEXT_TIMEOUT_US(p_publish_retransmit->interval_steps, 0);

    if (expect_reschedule)
    {
        EXPECT(timer_sch_reschedule_mock, 1);
    }

    access_publish_retransmission_message_add(model_handle, p_publish_retransmit,
                                              p_tx_message,
                                              (uint8_t *) p_tx_message->p_buffer,
                                              p_tx_message->length);

    if (expect_reschedule)
    {
        timer_sch_reschedule_mock_Verify();
    }

    /* Store allocated pointer in the tx_message */
    m_expected_msgs_to_be_published[model_handle].tx_message = *p_tx_message;
    m_expected_msgs_to_be_published[model_handle].p_access_payload = (uint8_t*) p_tx_message->p_buffer;
    m_expected_msgs_to_be_published[model_handle].access_payload_length = p_tx_message->length;
    m_expected_msgs_to_be_published[model_handle].is_expected = true;

    return next_timeout_us;
}

/*******************************************************************************
 * Test Setup
 *******************************************************************************/

void setUp(void)
{
    mesh_mem_mock_Init();
    timer_scheduler_mock_Init();

    timer_sch_reschedule_mock_Init();
    mesh_mem_free_mock_Init();

    m_current_timestamp = 0;
    m_expected_next_timeout_us = 0;

    memset(&m_expected_msgs_to_be_published, 0, sizeof (access_message_tx_t));

    access_publish_retransmission_init();
}

void tearDown(void)
{
    timer_sch_reschedule_mock_Verify();
    timer_sch_reschedule_mock_Destroy();
    mesh_mem_free_mock_Verify();
    mesh_mem_free_mock_Destroy();

    mesh_mem_mock_Verify();
    mesh_mem_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
}

static void prepareCallbacks(void)
{
    mesh_mem_free_StubWithCallback(mesh_mem_free_mock);
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_mock);
    access_packet_tx_StubWithCallback(access_packet_tx_mock);
}

/*****************************************************************************
 * Tests
 *****************************************************************************/

void test_fail_on_add_message(void)
{
    access_publish_retransmit_t publish_retransmit;
    access_message_tx_t tx_message;
    ACCESS_MESSAGE_TX_INIT(tx_message);

    prepareCallbacks();
    mesh_mem_alloc_StubWithCallback(NULL);

    publish_retransmit.count = 1;
    publish_retransmit.interval_steps = 10;

    /**************************************************************************/

    TEST_NRF_MESH_ASSERT_EXPECT(access_publish_retransmission_message_add(
                                    ACCESS_MODEL_COUNT, &publish_retransmit,
                                    &tx_message, (uint8_t*) tx_message.p_buffer,
                                    tx_message.length));
    TEST_NRF_MESH_ASSERT_EXPECT(access_publish_retransmission_message_add(
                                    0, NULL, &tx_message,
                                    (uint8_t*) tx_message.p_buffer,
                                    tx_message.length));
    TEST_NRF_MESH_ASSERT_EXPECT(access_publish_retransmission_message_add(
                                    0, &publish_retransmit, NULL,
                                    (uint8_t*) tx_message.p_buffer,
                                    tx_message.length));
    TEST_NRF_MESH_ASSERT_EXPECT(access_publish_retransmission_message_add(
                                    0, &publish_retransmit, &tx_message,
                                    NULL,
                                    tx_message.length));
    TEST_NRF_MESH_ASSERT_EXPECT(access_publish_retransmission_message_add(
                                    0, &publish_retransmit, &tx_message,
                                    (uint8_t*) tx_message.p_buffer,
                                    0));

    publish_retransmit.count = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(access_publish_retransmission_message_add(
                                    0, &publish_retransmit, &tx_message,
                                    (uint8_t*) tx_message.p_buffer,
                                    tx_message.length));
}

void test_1_msg_1_retransmit(void)
{
    access_publish_retransmit_t publish_retransmit;
    timestamp_t next_timeout_us;
    access_message_tx_t tx_message;
    ACCESS_MESSAGE_TX_INIT(tx_message);

    prepareCallbacks();

    publish_retransmit.count = 1;
    publish_retransmit.interval_steps = 10;

    /* Schedule 1 message now */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 0);
    next_timeout_us = add_message(0, &publish_retransmit, &tx_message, true);

    /* Schedule timeout */
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_current_timestamp = next_timeout_us;
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
}

void test_1_model_2_msgs(void)
{
    access_publish_retransmit_t publish_retransmit;
    timestamp_t next_timeout_us;
    access_message_tx_t tx_message[2];
    ACCESS_MESSAGE_TX_INIT(tx_message[0]);
    ACCESS_MESSAGE_TX_INIT(tx_message[1]);

    /* Make the messages differ */
    tx_message[1].opcode.opcode += 1;

    prepareCallbacks();

    publish_retransmit.count = 1;
    publish_retransmit.interval_steps = 10;

    /* Schedule a new message for the same model twice */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 0);
    next_timeout_us = add_message(0, &publish_retransmit, &tx_message[0], true);

    /* The old message shall be removed */
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 0);
    next_timeout_us = add_message(0, &publish_retransmit, &tx_message[1], true);

    /* Schedule timeout */
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_current_timestamp = next_timeout_us;
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
}

void test_1_model_1_msg_3_retransmits(void)
{
    access_publish_retransmit_t publish_retransmit;
    timestamp_t next_timeout_us;
    access_message_tx_t tx_message;
    ACCESS_MESSAGE_TX_INIT(tx_message);

    prepareCallbacks();

    publish_retransmit.count = 3;
    publish_retransmit.interval_steps = 5;

    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 0);
    next_timeout_us = add_message(0, &publish_retransmit, &tx_message, true);

    /* 1st retransmit. */
    m_expected_msgs_to_be_published[0].is_expected = true;
    m_current_timestamp = next_timeout_us;
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 1);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* 2nd retransmit. */
    m_expected_msgs_to_be_published[0].is_expected = true;
    m_current_timestamp = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 2);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* 3rd retransmit. Message shall be removed. */
    m_expected_msgs_to_be_published[0].is_expected = true;
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_current_timestamp = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 3);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
}

void test_2_models_2_msgs_3_retransmits_different_interval(void)
{
    access_publish_retransmit_t publish_retransmit[2];
    access_message_tx_t tx_message[2];
    timestamp_t next_timeout_us[2];
    ACCESS_MESSAGE_TX_INIT(tx_message[0]);
    ACCESS_MESSAGE_TX_INIT(tx_message[1]);

    /* Make the messages differ */
    tx_message[1].opcode.opcode += 1;

    prepareCallbacks();

    publish_retransmit[0].count = 3;
    publish_retransmit[0].interval_steps = 5;
    publish_retransmit[1].count = 3;
    publish_retransmit[1].interval_steps = 12;

    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit[0].interval_steps, 0);
    next_timeout_us[0] = add_message(0, &publish_retransmit[0], &tx_message[0], true);
    m_expected_msgs_to_be_published[0].is_expected = false;

    next_timeout_us[1] = add_message(1, &publish_retransmit[1], &tx_message[1], false);
    m_expected_msgs_to_be_published[1].is_expected = false;

    size_t active_msgs = 2;
    size_t cur_msg_id;
    size_t next_msg_id;
    for (size_t i = 0; i < 2 * 3; i++)
    {
        /* Which message shall be triggered now? */
        if (active_msgs == 2)
        {
            cur_msg_id = next_timeout_us[0] <= next_timeout_us[1] ? 0 : 1;
        }
        else
        {
            cur_msg_id = publish_retransmit[0].count != 0 ? 0 : 1;
        }

        /* Shift clock */
        m_current_timestamp = next_timeout_us[cur_msg_id];

        /* Shall the message be removed now? */
        publish_retransmit[cur_msg_id].count--;
        if (publish_retransmit[cur_msg_id].count == 0)
        {
//            UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
            EXPECT(mesh_mem_free_mock, 1);
            active_msgs--;
        }

        /* Update next timeout for that message according to new clock */
        next_timeout_us[cur_msg_id] = GET_NEXT_TIMEOUT_US(publish_retransmit[cur_msg_id].interval_steps,
                                                          3 - publish_retransmit[cur_msg_id].count);

        /* Which message shall be triggered next? */
        if (active_msgs == 2)
        {
            next_msg_id = next_timeout_us[0] <= next_timeout_us[1] ? 0 : 1;
        }
        else
        {
            next_msg_id = publish_retransmit[0].count != 0 ? 0 : 1;
        }

        /* Expect next schedule for that message */
        m_expected_next_timeout_us = next_timeout_us[next_msg_id];

        /* Trigger callback */
        m_expected_msgs_to_be_published[cur_msg_id].is_expected = true;
        m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
    }
}

void test_2_msgs_5_retransmits_same_interval(void)
{
    access_publish_retransmit_t publish_retransmit;
    access_message_tx_t tx_message[2];
    timestamp_t next_timeout_us;
    ACCESS_MESSAGE_TX_INIT(tx_message[0]);
    ACCESS_MESSAGE_TX_INIT(tx_message[1]);

    /* Make the messages differ */
    tx_message[1].opcode.opcode += 1;

    prepareCallbacks();

    publish_retransmit.count = 5;
    publish_retransmit.interval_steps = 10;

    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 0);
    next_timeout_us = add_message(0, &publish_retransmit, &tx_message[0], true);
    m_expected_msgs_to_be_published[0].is_expected = false;

    /* Next timeout is the same as for the first message */
    (void) add_message(1, &publish_retransmit, &tx_message[1], false);

    m_expected_msgs_to_be_published[1].is_expected = false;

    /* Do all steps in the loop except the last one */
    for (size_t i = 0; i < 4; i++)
    {
        /* Shift clock */
        m_current_timestamp = next_timeout_us;

        /* Update next timeout for that message according to new clock */
        next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps,
                                              i + 1);

        /* Expect next schedule for that message */
        m_expected_next_timeout_us = next_timeout_us;

        /* Trigger callback */
        m_expected_msgs_to_be_published[0].is_expected = true;
        m_expected_msgs_to_be_published[1].is_expected = true;
        m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
    }

    /* Last step */
    m_current_timestamp = next_timeout_us;
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 2);
    EXPECT(mesh_mem_free_mock, 2);
    m_expected_msgs_to_be_published[0].is_expected = true;
    m_expected_msgs_to_be_published[1].is_expected = true;
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
}

void test_error_on_publishing(void)
{
    access_publish_retransmit_t publish_retransmit;
    access_message_tx_t tx_message;
    ACCESS_MESSAGE_TX_INIT(tx_message);

    prepareCallbacks();

    publish_retransmit.count = 6;
    publish_retransmit.interval_steps = 10;

    access_packet_tx_StubWithCallback(NULL);

    /* Schedule the message */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 0);
    (void) add_message(0, &publish_retransmit, &tx_message, true);

    /* NRF_ERROR_NO_MEM */
    access_packet_tx_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    m_current_timestamp = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 1);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* NRF_ERROR_FORBIDDEN */
    access_packet_tx_ExpectAnyArgsAndReturn(NRF_ERROR_FORBIDDEN);
    m_current_timestamp = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 2);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* NRF_ERROR_INVALID_STATE */
    access_packet_tx_ExpectAnyArgsAndReturn(NRF_ERROR_INVALID_STATE);
    m_current_timestamp = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 3);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* The message shall be removed before count is expired by
     * NRF_ERROR_NOT_FOUND */
    access_packet_tx_ExpectAnyArgsAndReturn(NRF_ERROR_NOT_FOUND);
    m_current_timestamp = GET_NEXT_TIMEOUT_US(publish_retransmit.interval_steps, 4);
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
}

void test_3_models_3_msgs(void)
{
    access_publish_retransmit_t publish_retransmit[4];
    access_message_tx_t tx_message[4];
    timestamp_t next_timeout_us[4];
    ACCESS_MESSAGE_TX_INIT(tx_message[0]);
    ACCESS_MESSAGE_TX_INIT(tx_message[1]);
    ACCESS_MESSAGE_TX_INIT(tx_message[2]);
    ACCESS_MESSAGE_TX_INIT(tx_message[3]);

    /* Make the messages differ */
    tx_message[1].opcode.opcode += 1;
    tx_message[2].opcode.opcode += 2;
    tx_message[3].opcode.opcode += 3;

    prepareCallbacks();

    publish_retransmit[0].count = 1;
    publish_retransmit[0].interval_steps = 5;
    publish_retransmit[1].count = 1;
    publish_retransmit[1].interval_steps = 12;
    publish_retransmit[2].count = 1;
    publish_retransmit[2].interval_steps = 7;
    publish_retransmit[3].count = 1;
    publish_retransmit[3].interval_steps = 6;

    /* Adding message the 1st model */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit[0].interval_steps, 0);
    next_timeout_us[0] = add_message(0, &publish_retransmit[0], &tx_message[0], true);

    /* Adding message to the 2nd model */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit[1].interval_steps, 0);
    next_timeout_us[1] = add_message(1, &publish_retransmit[1], &tx_message[1], false);

    /* Adding message to the 3rd model */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit[2].interval_steps, 0);
    next_timeout_us[2] = add_message(2, &publish_retransmit[2], &tx_message[2], false);

    /* Replace message for the 1st model. The timer shall be rescheduled. */
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit[3].interval_steps, 0);
    next_timeout_us[3] = add_message(0, &publish_retransmit[3], &tx_message[3], true);

    /* Schedule the 4th message */
    m_current_timestamp = next_timeout_us[3];
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_expected_msgs_to_be_published[0].is_expected = true;
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* Schedule the 3rd message */
    m_current_timestamp = next_timeout_us[2];
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_expected_msgs_to_be_published[2].is_expected = true;
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* Schedule the 2nd message */
    m_current_timestamp = next_timeout_us[1];
    m_expected_msgs_to_be_published[1].is_expected = true;
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
}

void test_2_models_2_msgs_new_has_shorter_interval(void)
{
    access_publish_retransmit_t publish_retransmit[2];
    access_message_tx_t tx_message[2];
    timestamp_t next_timeout_us[2];
    ACCESS_MESSAGE_TX_INIT(tx_message[0]);
    ACCESS_MESSAGE_TX_INIT(tx_message[1]);

    /* Make the messages differ */
    tx_message[1].opcode.opcode += 1;

    prepareCallbacks();

    publish_retransmit[0].count = 1;
    publish_retransmit[0].interval_steps = 12;
    publish_retransmit[1].count = 1;
    publish_retransmit[1].interval_steps = 5;

    /* Adding message the 1st model */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit[0].interval_steps, 0);
    next_timeout_us[0] = add_message(0, &publish_retransmit[0], &tx_message[0], true);

    /* Adding message to the 2nd model */
    m_expected_next_timeout_us = GET_NEXT_TIMEOUT_US(publish_retransmit[1].interval_steps, 0);
    next_timeout_us[1] = add_message(1, &publish_retransmit[1], &tx_message[1], true);

    /* Schedule the 2nd message */
    m_current_timestamp = next_timeout_us[1];
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_expected_msgs_to_be_published[1].is_expected = true;
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);

    /* Schedule the 1st message */
    m_current_timestamp = next_timeout_us[0];
//    UTEST_FUNC_EXPECT(mesh_mem_free_mock, 1);
    EXPECT(mesh_mem_free_mock, 1);
    m_expected_msgs_to_be_published[0].is_expected = true;
    m_p_timer_evt->cb(m_current_timestamp, m_p_timer_evt->p_context);
}
