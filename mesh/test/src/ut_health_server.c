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

#include "log.h"
#include "access_mock.h"
#include "access_config_mock.h"
#include "access_reliable_mock.h"
#include "bearer_event_mock.h"
#include "timer_scheduler_mock.h"
#include "nrf_mesh_mock.h"

#include "health_messages.h"
#include "health_opcodes.h"
#include "health_server.h"
#include "mesh_config_mock.h"
#include "mesh_opt_mock.h"

#include "utils.h"

#define TEST_MODEL_HANDLE           14
#define TEST_ELEMENT_INDEX          0
#define TEST_COMPANY_ID             0x1234
#define TEST_MAX_HEALTH_SERVERS     (ACCESS_ELEMENT_COUNT)

#define ACCESS_MESSAGE_RX(req, op) \
    { \
        .opcode = ACCESS_OPCODE_SIG(op), \
        .p_data = (const uint8_t *) &req, \
        .length = sizeof(req) \
    }

#define ACCESS_MESSAGE_RX_EMPTY(op) \
    { \
        .opcode = ACCESS_OPCODE_SIG(op), \
        .p_data = NULL, \
        .length = 0, \
    }

/********** Static variables and mock functions **********/

static const access_opcode_handler_t * mp_opcode_handlers;
static uint16_t m_num_opcodes;

static access_model_handle_t m_model_handle;

static uint32_t access_model_subscription_list_alloc_mock(access_model_handle_t handle, int count)
{
    return NRF_SUCCESS;
}

static uint32_t access_model_add_mock(const access_model_add_params_t * p_init_params,
        access_model_handle_t * p_model_handle, int count)
{
    m_model_handle = TEST_MODEL_HANDLE;
    *p_model_handle = m_model_handle;

    mp_opcode_handlers = p_init_params->p_opcode_handlers;
    m_num_opcodes = p_init_params->opcode_count;

    TEST_ASSERT_EQUAL(HEALTH_SERVER_MODEL_ID, p_init_params->model_id.model_id);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_init_params->model_id.company_id);

    return NRF_SUCCESS;
}

static timestamp_t m_current_time;
timestamp_t timer_now(void)
{
    return m_current_time;
}

#define EXPECT_TIMER_SCH_SCHEDULE() \
    TEST_ASSERT_FALSE(m_timer_sch_schedule_expected); \
    m_timer_sch_schedule_expected = true

#define TIMER_SCH_TRIGGER() \
    do { \
        if(mp_timer_sch_event->interval != 0) \
        { \
            mp_timer_sch_event->timestamp += mp_timer_sch_event->interval; \
            m_current_time += mp_timer_sch_event->interval; \
        } \
        else \
        { \
            m_current_time = mp_timer_sch_event->timestamp; \
        } \
        mp_timer_sch_event->cb(m_current_time, mp_timer_sch_event->p_context); \
    } while(0);

static bool m_timer_sch_schedule_expected = false;
static timer_event_t * mp_timer_sch_event;

void timer_sch_schedule_mock(timer_event_t * p_event, int count)
{
    TEST_ASSERT_TRUE(m_timer_sch_schedule_expected);
    m_timer_sch_schedule_expected = false;

    mp_timer_sch_event = p_event;
}

#define EXPECT_PUBLISH_PERIOD_GET(x_handle, x_retval_res, x_retval_steps) \
    do { \
        TEST_ASSERT_FALSE(m_publish_period_get_expected); \
        m_publish_period_get_expected = true; \
        m_publish_period_get_expected_handle = x_handle; \
        m_publish_period_get_retval_res = x_retval_res; \
        m_publish_period_get_retval_steps = x_retval_steps; \
    } while(0)
static bool m_publish_period_get_expected = false;
static access_model_handle_t m_publish_period_get_expected_handle;
static access_publish_resolution_t m_publish_period_get_retval_res;
static uint8_t m_publish_period_get_retval_steps;

static uint32_t access_model_publish_period_get_mock(access_model_handle_t handle,
        access_publish_resolution_t * p_res, uint8_t * p_steps, int count)
{
    TEST_ASSERT_NOT_NULL(p_res);
    TEST_ASSERT_NOT_NULL(p_steps);

    TEST_ASSERT_TRUE(m_publish_period_get_expected);
    m_publish_period_get_expected = false;

    TEST_ASSERT_EQUAL(m_publish_period_get_expected_handle, handle);

    *p_res = m_publish_period_get_retval_res;
    *p_steps = m_publish_period_get_retval_steps;

    return NRF_SUCCESS;
}

#define EXPECT_REPLY(x_handle, x_opcode, xp_expected_data, x_expected_data_length) \
    do { \
        TEST_ASSERT_FALSE(m_reply_expected); \
        m_reply_expected = true; \
        m_reply_expected_handle = x_handle; \
        m_reply_expected_opcode = x_opcode; \
        mp_reply_expected_data = xp_expected_data; \
        m_reply_expected_data_length = x_expected_data_length; \
    } while(0)

static bool m_reply_expected = false;
static access_model_handle_t m_reply_expected_handle;
static uint16_t m_reply_expected_opcode;
static void * mp_reply_expected_data;
static uint16_t m_reply_expected_data_length;

static uint32_t access_model_reply_mock(access_model_handle_t handle, const access_message_rx_t * p_message,
        const access_message_tx_t * p_reply, int count)
{
    TEST_ASSERT_NOT_NULL(p_message);
    TEST_ASSERT_NOT_NULL(p_reply);

    TEST_ASSERT_TRUE(m_reply_expected);
    m_reply_expected = false;

    TEST_ASSERT_EQUAL(m_reply_expected_handle, handle);
    TEST_ASSERT_EQUAL(m_reply_expected_opcode, p_reply->opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reply->opcode.company_id);
    TEST_ASSERT_EQUAL(m_reply_expected_data_length, p_reply->length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t *) mp_reply_expected_data, p_reply->p_buffer, p_reply->length);

    return NRF_SUCCESS;
}

#define EXPECT_TX(x_handle, x_opcode, xp_expected_data, x_expected_data_length) \
    do { \
        TEST_ASSERT_FALSE(m_publish_expected); \
        m_publish_expected = true; \
        m_publish_expected_handle = x_handle; \
        m_publish_expected_opcode = x_opcode; \
        mp_publish_expected_data = xp_expected_data; \
        m_publish_expected_data_length = x_expected_data_length; \
    } while(0)

static bool m_publish_expected = false;
static access_model_handle_t m_publish_expected_handle;
static uint16_t m_publish_expected_opcode;
static void * mp_publish_expected_data;
static uint16_t m_publish_expected_data_length;

static uint32_t access_model_publish_mock(access_model_handle_t handle, const access_message_tx_t * p_message, int count)
{
    TEST_ASSERT_NOT_NULL(p_message);

    TEST_ASSERT_TRUE(m_publish_expected);
    m_publish_expected = false;

    TEST_ASSERT_EQUAL(m_publish_expected_handle, handle);
    TEST_ASSERT_EQUAL(m_publish_expected_opcode, p_message->opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_message->opcode.company_id);
    TEST_ASSERT_EQUAL(m_publish_expected_data_length, p_message->length);
    TEST_ASSERT_EQUAL(false, p_message->force_segmented);
    TEST_ASSERT_EQUAL(NRF_MESH_TRANSMIC_SIZE_DEFAULT, p_message->transmic_size);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t *) mp_publish_expected_data, p_message->p_buffer, p_message->length);

    return NRF_SUCCESS;
}

static void call_opcode_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    bool handler_found = false;
    for(uint16_t i = 0; i < m_num_opcodes; ++i)
    {
        if (memcmp(&mp_opcode_handlers[i].opcode, &p_message->opcode, sizeof(access_opcode_t)) == 0)
        {
            mp_opcode_handlers[i].handler(handle, p_message, p_args);
            handler_found = true;
            break;
        }
    }

    TEST_ASSERT_TRUE(handler_found);
}

extern const mesh_config_entry_params_t m_mesh_opt_health_params;
static const mesh_config_entry_params_t * entry_params_get(mesh_config_entry_id_t entry_id)
{
    if (IS_IN_RANGE(entry_id.record, MESH_OPT_HEALTH_ID_START, MESH_OPT_HEALTH_ID_END))
    {
        return &m_mesh_opt_health_params;
    }

    TEST_FAIL_MESSAGE("Unknown entry id");
    return NULL;
}

uint32_t mesh_config_entry_set(mesh_config_entry_id_t id, const void * p_entry)
{
    return entry_params_get(id)->callbacks.setter(id, p_entry);
}

uint32_t mesh_config_entry_get(mesh_config_entry_id_t id, void * p_entry)
{
    entry_params_get(id)->callbacks.getter(id, p_entry);
    return NRF_SUCCESS;
}

uint32_t mesh_config_entry_delete(mesh_config_entry_id_t id)
{
    return NRF_SUCCESS;
}

/********** Self-test helper functions **********/

static const uint8_t * mp_next_faults;
static uint8_t m_next_faults_count;

static void selftest_test_function(health_server_t * p_server, uint16_t company_id, uint8_t test_id)
{
    for(uint8_t i = 0; i < m_next_faults_count; ++i)
    {
        health_server_fault_register(p_server, mp_next_faults[i]);
    }
}

/********** Attention timer helper functions **********/

#define EXPECT_ATTENTION_CALLBACK(xp_server, x_attention_state) \
    do { \
        TEST_ASSERT_FALSE(m_attention_callback_expected); \
        m_attention_callback_expected = true; \
        mp_attention_callback_expected_server = xp_server; \
        m_attention_callback_expected_state = x_attention_state; \
    } while(0);
static bool m_attention_callback_expected = false;
static const health_server_t * mp_attention_callback_expected_server;
static bool m_attention_callback_expected_state;

static void attention_callback(const health_server_t * p_server, bool attention_state)
{
    TEST_ASSERT_TRUE(m_attention_callback_expected);
    m_attention_callback_expected = false;

    TEST_ASSERT_EQUAL_PTR(mp_attention_callback_expected_server, p_server);
    TEST_ASSERT_EQUAL(m_attention_callback_expected_state, attention_state);
}

static void verify_attention_timer(health_server_t * p_server, uint8_t expected_attention)
{
    access_message_rx_t request_message = ACCESS_MESSAGE_RX_EMPTY(HEALTH_OPCODE_ATTENTION_GET);
    health_msg_attention_status_t expected_reply = { .attention = expected_attention };
    EXPECT_REPLY(p_server->model_handle, HEALTH_OPCODE_ATTENTION_STATUS, &expected_reply, sizeof(expected_reply));
    call_opcode_handler(p_server->model_handle, &request_message, p_server);
}

static void set_attention_timer(health_server_t * p_server, uint8_t attention_value, bool acked)
{
    const health_msg_attention_set_t request = { .attention = attention_value };
    access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, acked ? HEALTH_OPCODE_ATTENTION_SET : HEALTH_OPCODE_ATTENTION_SET_UNACKED);

    health_msg_attention_status_t expected_reply = { .attention = attention_value }; /* Note: this variable must be declared outside the if() */
    if(acked)
    {
        EXPECT_REPLY(p_server->model_handle, HEALTH_OPCODE_ATTENTION_STATUS, &expected_reply, sizeof(expected_reply));
    }

    call_opcode_handler(p_server->model_handle, &request_message, p_server);
}

/********** Test initialization and finalization **********/

void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    access_model_add_StubWithCallback(access_model_add_mock);
    access_model_subscription_list_alloc_StubWithCallback(access_model_subscription_list_alloc_mock);
    access_model_reply_StubWithCallback(access_model_reply_mock);
    access_model_publish_StubWithCallback(access_model_publish_mock);
    access_model_publish_period_get_StubWithCallback(access_model_publish_period_get_mock);

    access_reliable_mock_Init();
    bearer_event_mock_Init();

    timer_scheduler_mock_Init();
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock);

    nrf_mesh_mock_Init();
    nrf_mesh_unique_token_get_IgnoreAndReturn((nrf_mesh_tx_token_t)0x55AA55AAul);

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();

    mesh_config_mock_Init();
    //mesh_config_entry_mock_Init();

    m_publish_period_get_expected = false;
    TEST_ASSERT_TRUE(TEST_MAX_HEALTH_SERVERS > 1);
}

void tearDown(void)
{
    TEST_ASSERT_FALSE(m_reply_expected);
    TEST_ASSERT_FALSE(m_timer_sch_schedule_expected);
    TEST_ASSERT_FALSE(m_attention_callback_expected);
    TEST_ASSERT_FALSE(m_publish_expected);
    TEST_ASSERT_FALSE(m_publish_period_get_expected);

    access_mock_Verify();
    access_mock_Destroy();

    access_config_mock_Verify();
    access_config_mock_Destroy();

    access_reliable_mock_Verify();
    access_reliable_mock_Destroy();

    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();

    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();

    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();

    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();

    //mesh_config_entry_mock_Verify();
    //mesh_config_entry_mock_Destroy();
}

/********** Test cases **********/

void test_faultarray(void)
{
    health_server_t server;
    health_server_selftest_t test_array[] = {{ .test_id = 0x01, .selftest_function = selftest_test_function }};
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_server_init(&server, TEST_ELEMENT_INDEX, TEST_COMPANY_ID, NULL, test_array, ARRAY_SIZE(test_array)));

    TEST_ASSERT_EQUAL(0, health_server_fault_count_get(&server));

    access_model_publish_period_divisor_set_ExpectAndReturn(server.model_handle, 1 << server.fast_period_divisor, NRF_SUCCESS);

    health_server_fault_register(&server, 0xf1);
    TEST_ASSERT_EQUAL(1, health_server_fault_count_get(&server));
    TEST_ASSERT_TRUE(health_server_fault_is_set(&server, 0xf1));

    health_server_fault_register(&server, 0x22);
    TEST_ASSERT_EQUAL(2, health_server_fault_count_get(&server));
    TEST_ASSERT_TRUE(health_server_fault_is_set(&server, 0x22));

    TEST_ASSERT_FALSE(health_server_fault_is_set(&server, 0x00));
    TEST_ASSERT_FALSE(health_server_fault_is_set(&server, 0x88));
    TEST_ASSERT_FALSE(health_server_fault_is_set(&server, 0xff));

    health_server_fault_clear(&server, 0xf1);
    TEST_ASSERT_EQUAL(1, health_server_fault_count_get(&server));
    TEST_ASSERT_FALSE(health_server_fault_is_set(&server, 0xf1));
    TEST_ASSERT_TRUE(health_server_fault_is_set(&server, 0x22));

    access_model_publish_period_divisor_set_ExpectAndReturn(server.model_handle, 1, NRF_SUCCESS);
    health_server_fault_clear(&server, 0x22);
    TEST_ASSERT_EQUAL(0, health_server_fault_count_get(&server));
    TEST_ASSERT_FALSE(health_server_fault_is_set(&server, 0xf1));
    TEST_ASSERT_FALSE(health_server_fault_is_set(&server, 0x22));
}

void test_fast_period_divisor(void)
{
    health_server_t server;
    health_server_selftest_t test_array[] = {{ .test_id = 0x01, .selftest_function = selftest_test_function }};
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_server_init(&server, TEST_ELEMENT_INDEX, TEST_COMPANY_ID, NULL, test_array, ARRAY_SIZE(test_array)));

    TEST_ASSERT_EQUAL(0, health_server_fault_count_get(&server));

    /* Cheat, and set the fast period divisor manually to 2 (giving an actual divisor of 2^2): */
    server.fast_period_divisor = 2;

    /* Fast period divisor value will be used to calculate actual divisor and will be passed to access module */
    access_model_publish_period_divisor_set_ExpectAndReturn(server.model_handle, 1 << server.fast_period_divisor, NRF_SUCCESS);
    health_server_fault_register(&server, 0xf1);

    access_model_publish_period_divisor_set_ExpectAndReturn(server.model_handle, 1, NRF_SUCCESS);
    health_server_fault_clear(&server, 0xf1);
}

void test_selftest(void)
{
    health_server_t server;
    health_server_selftest_t test_array[] =
    {
        { .test_id = 0x34, .selftest_function = selftest_test_function },
        { .test_id = 0xf1, .selftest_function = selftest_test_function },
    };
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_server_init(&server, TEST_ELEMENT_INDEX, TEST_COMPANY_ID, NULL, test_array, ARRAY_SIZE(test_array)));

    /* Check that no faults are returned after initialization: */
    {
        const health_msg_fault_get_t request = { .company_id = TEST_COMPANY_ID };
        access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_FAULT_GET);

        health_msg_fault_status_t expected_reply = { .test_id = 0, .company_id = TEST_COMPANY_ID };
        EXPECT_REPLY(server.model_handle, HEALTH_OPCODE_FAULT_STATUS, &expected_reply, sizeof(expected_reply));
        call_opcode_handler(server.model_handle, &request_message, &server);
    }

    /* Run the selftest function with ID 0xf1: */
    {
        const health_msg_fault_test_t request = { .test_id = 0xf1, .company_id = TEST_COMPANY_ID };
        access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_FAULT_TEST);

        const uint8_t expected_faults[] = { 0x2f, 0x88 };
        mp_next_faults = expected_faults;
        m_next_faults_count = sizeof(expected_faults);

        uint8_t reply_buffer[sizeof(health_msg_fault_status_t) + 2];
        health_msg_fault_status_t * p_expected_reply = (health_msg_fault_status_t *) reply_buffer;
        p_expected_reply->test_id = 0xf1;
        p_expected_reply->company_id = TEST_COMPANY_ID;
        memcpy(p_expected_reply->fault_array, expected_faults, sizeof(expected_faults));

        access_model_publish_period_divisor_set_ExpectAndReturn(server.model_handle, 1 << server.fast_period_divisor, NRF_SUCCESS);
        EXPECT_REPLY(server.model_handle, HEALTH_OPCODE_FAULT_STATUS, reply_buffer, sizeof(reply_buffer));
        call_opcode_handler(server.model_handle, &request_message, &server);
    }

    /* Clear the fault array using an unacked message: */
    {
        const health_msg_fault_clear_t request = { .company_id = TEST_COMPANY_ID };
        access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_FAULT_CLEAR_UNACKED);
        call_opcode_handler(server.model_handle, &request_message, &server);
    }

    /* Verify that the fault array was cleared: */
    {
        const health_msg_fault_get_t request = { .company_id = TEST_COMPANY_ID };
        access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_FAULT_GET);

        health_msg_fault_status_t expected_reply = { .test_id = 0xf1, .company_id = TEST_COMPANY_ID };
        EXPECT_REPLY(server.model_handle, HEALTH_OPCODE_FAULT_STATUS, &expected_reply, sizeof(expected_reply));
        call_opcode_handler(server.model_handle, &request_message, &server);
    }

    /* Manually set a fault in the current fault array and verify that it is carried through to the registered fault array: */
    health_server_fault_register(&server, 0x25);

    /* Verify that the fault is set: */
    bool verified_after_clear = false;
_verify_fault_set:
    {
        const health_msg_fault_get_t request = { .company_id = TEST_COMPANY_ID };
        access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_FAULT_GET);

        uint8_t reply_buffer[sizeof(health_msg_fault_status_t) + 1];
        health_msg_fault_status_t * p_expected_reply = (health_msg_fault_status_t *) reply_buffer;
        p_expected_reply->test_id = 0xf1;
        p_expected_reply->company_id = TEST_COMPANY_ID;
        p_expected_reply->fault_array[0] = 0x25;
        EXPECT_REPLY(server.model_handle, HEALTH_OPCODE_FAULT_STATUS, reply_buffer, sizeof(reply_buffer));
        call_opcode_handler(server.model_handle, &request_message, &server);
    }

    /* Clear the fault and check that it is still set in the registered fault array: */
    health_server_fault_clear(&server, 0x25);
    if(!verified_after_clear)
    {
        verified_after_clear = true;
        goto _verify_fault_set;
    }
}

void test_attention(void)
{
    health_server_t server;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_server_init(&server, TEST_ELEMENT_INDEX, TEST_COMPANY_ID, attention_callback, NULL, 0));

    /* Get the attention timer value to verify that it is 0 after initialization: */
    verify_attention_timer(&server, 0);

    /* Set the attention timer to 5 seconds: */
    EXPECT_TIMER_SCH_SCHEDULE();
    EXPECT_ATTENTION_CALLBACK(&server, true);
    set_attention_timer(&server, 5, true);

    /* Run the timer scheduler until the attention timer expires: */
    for(int i = 0; i < 4; ++i)
    {
        TIMER_SCH_TRIGGER();
        verify_attention_timer(&server, 5 - (i + 1));
    }

    EXPECT_ATTENTION_CALLBACK(&server, false);
    timer_sch_abort_ExpectAnyArgs(); /* The timer should be cancelled because all attention timers have expired. */
    TIMER_SCH_TRIGGER();

    /* The attention timer should be 0 at this point: */
    verify_attention_timer(&server, 0);

    /* Set the attention timer to 3 seconds with a unacked message: */
    EXPECT_TIMER_SCH_SCHEDULE();
    EXPECT_ATTENTION_CALLBACK(&server, true);
    set_attention_timer(&server, 3, false);

    /* Verify the attention timer value: */
    verify_attention_timer(&server, 3);

    /* Reduce the attention timer to 1: */
    TIMER_SCH_TRIGGER();
    TIMER_SCH_TRIGGER();

    /* Verify the attention timer value: */
    verify_attention_timer(&server, 1);

    /* Set the attention timer to 11: */
    set_attention_timer(&server, 11, true);

    /* Run the attention timer for a little while: */
    for(int i = 0; i < 5; ++i)
    {
        TIMER_SCH_TRIGGER();
        verify_attention_timer(&server, 11 - (i + 1));
    }

    /* Cancel the attention timer: */
    set_attention_timer(&server, 0, true);

    /* The timer will then be cancelled at the next timer tick: */
    EXPECT_ATTENTION_CALLBACK(&server, false);
    timer_sch_abort_ExpectAnyArgs();
    TIMER_SCH_TRIGGER();

    /* Verify that the attention timer has the correct value: */
    verify_attention_timer(&server, 0);
}

void test_multimodel_attention(void)
{
    health_server_t servers[TEST_MAX_HEALTH_SERVERS];
    const uint8_t attention_values[TEST_MAX_HEALTH_SERVERS] = { 4, 5, 1, 11, 10 };
    const uint8_t max_attention_value = 11; /* max(attention_values) */

    EXPECT_TIMER_SCH_SCHEDULE();
    for(int i = 0; i < TEST_MAX_HEALTH_SERVERS; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_server_init(&servers[i], TEST_ELEMENT_INDEX + i, TEST_COMPANY_ID, attention_callback, NULL, 0));

        EXPECT_ATTENTION_CALLBACK(&servers[i], true);
        set_attention_timer(&servers[i], attention_values[i], true);
    }

    /* Run the timer scheduler and ensure all timers expire when they should: */
    for(unsigned int i = 0; i < max_attention_value; ++i)
    {
        for(unsigned int d = 0; d < sizeof(attention_values); ++d)
        {
            verify_attention_timer(&servers[d],
                    attention_values[d] - i <= attention_values[d] ? attention_values[d] - i : 0);

            if(i + 1 == attention_values[d])
            {
                EXPECT_ATTENTION_CALLBACK(&servers[d], false);
            }
        }

        if(i == max_attention_value - 1u)
        {
            timer_sch_abort_ExpectAnyArgs();
        }

        //TIMER_SCH_TRIGGER();
        do {
        if(mp_timer_sch_event->interval != 0)
        {
            mp_timer_sch_event->timestamp += mp_timer_sch_event->interval;
            m_current_time += mp_timer_sch_event->interval;
        }
        else
        {
            m_current_time = mp_timer_sch_event->timestamp;
        }
        mp_timer_sch_event->cb(m_current_time, mp_timer_sch_event->p_context);
        } while(0);
    }
}

void test_status_period(void)
{
    health_server_t server[TEST_MAX_HEALTH_SERVERS];
    health_server_selftest_t test_array[] = {{ .test_id = 0x01, .selftest_function = selftest_test_function }};

    for (int i = 0; i < TEST_MAX_HEALTH_SERVERS; i++)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_server_init(&server[i], TEST_ELEMENT_INDEX, TEST_COMPANY_ID, NULL, test_array, ARRAY_SIZE(test_array)));

        /* Check that the default value of the fast period divisor is 0: */
        {
            access_message_rx_t request_message = ACCESS_MESSAGE_RX_EMPTY(HEALTH_OPCODE_PERIOD_GET);
            health_msg_period_status_t expected_reply = { .fast_period_divisor = 0 };
            EXPECT_REPLY(server[i].model_handle, HEALTH_OPCODE_PERIOD_STATUS, &expected_reply, sizeof(expected_reply));
            call_opcode_handler(server[i].model_handle, &request_message, &server[i]);
        }

        /* Set the fast period divisor to 5: */
        {
            const health_msg_period_set_t request = { .fast_period_divisor = 5 };
            access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_PERIOD_SET);

            health_msg_period_status_t expected_reply = { .fast_period_divisor = 5};
            EXPECT_REPLY(server[i].model_handle, HEALTH_OPCODE_PERIOD_STATUS, &expected_reply, sizeof(expected_reply));
            call_opcode_handler(server[i].model_handle, &request_message, &server[i]);
        }

        /* Set the fast period divisor to 8: */
        {
            const health_msg_period_set_t request = { .fast_period_divisor = 8 };
            access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_PERIOD_SET_UNACKED);
            call_opcode_handler(server[i].model_handle, &request_message, &server[i]);
        }

        /* Check that the default value of the fast period divisor is 8: */
        {
            access_message_rx_t request_message = ACCESS_MESSAGE_RX_EMPTY(HEALTH_OPCODE_PERIOD_GET);
            health_msg_period_status_t expected_reply = { .fast_period_divisor = 8 };
            EXPECT_REPLY(server[i].model_handle, HEALTH_OPCODE_PERIOD_STATUS, &expected_reply, sizeof(expected_reply));
            call_opcode_handler(server[i].model_handle, &request_message, &server[i]);
        }

        /* Try setting the divisor to an invalid value: */
        {
            const health_msg_period_set_t request = { .fast_period_divisor = 18 }; /* 15 is the maximum according to the spec */
            access_message_rx_t request_message = ACCESS_MESSAGE_RX(request, HEALTH_OPCODE_PERIOD_SET);

            health_msg_period_status_t expected_reply = { .fast_period_divisor = 8}; /* The invalid value should have been ignored */
            EXPECT_REPLY(server[i].model_handle, HEALTH_OPCODE_PERIOD_STATUS, &expected_reply, sizeof(expected_reply));
            call_opcode_handler(server[i].model_handle, &request_message, &server[i]);
        }
    }
}

