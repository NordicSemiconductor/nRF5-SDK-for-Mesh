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

#include "access_reliable.h"
#include "packet_mesh.h"
#include "test_assert.h"
#include "access_utils.h"

#include "access_mock.h"
#include "access_config_mock.h"
#include "bearer_event_mock.h"
#include "timer_scheduler_mock.h"
#include "timer_mock.h"

/* ******************* Various definitions ******************* */

#define SOME_ERROR_CODE (0x12345431)
#define TEST_ARGS_PTR ((void *) 0xDEADBEEF)
#define TEST_HANDLE (1)
#define REPEATS (8)
#define TIME_SPACING MS_TO_US(2)
/* ******************* Type definitions ******************* */

typedef enum
{
    TIMER_STATE_STOPPED = 0,
    TIMER_STATE_RUNNING = 1
} timer_state_t;

/* ******************* Static variables ******************* */

static struct
{
    timer_event_t * p_evt;
    timer_state_t current_state;
    timestamp_t expected_timeout;
    timer_state_t expected_state;
    uint32_t sch_calls;
    uint32_t abort_calls;
} m_timer;


typedef struct
{
    access_model_handle_t handle;
    void * p_args;
    access_reliable_status_t status;
} expected_cb_data_t;

static struct
{
    expected_cb_data_t expected_data[ACCESS_RELIABLE_TRANSFER_COUNT];
    uint32_t num_calls;
} m_status_cb;

static access_reliable_t m_reliables[ACCESS_RELIABLE_TRANSFER_COUNT];

/* ******************* Callback functions ******************* */

static void timer_sch_reschedule_cb(timer_event_t * p_evt, timestamp_t next_timeout, int num_calls)
{
    TEST_ASSERT_MESSAGE(m_timer.sch_calls > 0, "Timer (re)schedule called more times than expected");
    m_timer.sch_calls--;
    TEST_ASSERT_EQUAL(m_timer.expected_state, m_timer.current_state);
    TEST_ASSERT_EQUAL(m_timer.expected_timeout, next_timeout);
    if (NULL == m_timer.p_evt)
    {
        m_timer.p_evt = p_evt;
    }
    else
    {
        TEST_ASSERT_NOT_NULL(m_timer.p_evt);
    }
    m_timer.current_state = TIMER_STATE_RUNNING;
}

static void timer_sch_abort_cb(timer_event_t * p_evt, int num_calls)
{
    TEST_ASSERT_MESSAGE(m_timer.abort_calls > 0, "Timer abort called more times than expected");
    m_timer.abort_calls--;
    TEST_ASSERT_EQUAL(TIMER_STATE_RUNNING, m_timer.current_state);
    m_timer.current_state = TIMER_STATE_STOPPED;
}

static void status_cb(access_model_handle_t handle, void * p_args, access_reliable_status_t status)
{
    TEST_ASSERT_MESSAGE(m_status_cb.num_calls > 0, "Success callback called more times than expected");
    m_status_cb.num_calls--;
    if (status == ACCESS_RELIABLE_TRANSFER_SUCCESS)
        printf("Success: %u: %p!\n", handle, p_args);
    else
        printf("Timeout: %u: %p!\n", handle, p_args);
    TEST_ASSERT_EQUAL(m_status_cb.expected_data[m_status_cb.num_calls].handle, handle);
    TEST_ASSERT_EQUAL(m_status_cb.expected_data[m_status_cb.num_calls].p_args, p_args);
    TEST_ASSERT_EQUAL(m_status_cb.expected_data[m_status_cb.num_calls].status, status);
}

/* ******************* Utility functions ******************* */

static void fire_timeout(timestamp_t timestamp, void * p_args)
{

    uint32_t expected_calls = m_timer.sch_calls;
    m_timer.p_evt->cb(timestamp, p_args);

    if (m_timer.p_evt->interval > 0)
    {
        /* Fake the reschedule */
        m_timer.expected_timeout = timestamp + m_timer.p_evt->interval;
        timer_sch_reschedule_cb(m_timer.p_evt, timestamp + m_timer.p_evt->interval, 0);
    }

    bool rescheduled = (expected_calls > 0) && (m_timer.sch_calls == 0);
    if (!rescheduled)
    {
        /* If no reschedule has been called, "stop" the timer. */
        m_timer.current_state = TIMER_STATE_STOPPED;
    }
}

static void status_cb_Expect(access_model_handle_t handle, void * p_args, access_reliable_status_t status)
{
    m_status_cb.expected_data[m_status_cb.num_calls].handle = handle;
    m_status_cb.expected_data[m_status_cb.num_calls].p_args = p_args;
    m_status_cb.expected_data[m_status_cb.num_calls].status = status;
    m_status_cb.num_calls++;
}

static void timer_reschedule_ExpectAndReturn(timer_state_t state, timestamp_t timeout)
{
    m_timer.sch_calls++;
    m_timer.expected_state = state;
    m_timer.expected_timeout = timeout;
}

static void __timer_abort_ExpectAndReturn(timer_state_t state)
{
    m_timer.abort_calls++;
    m_timer.expected_state = state;
}

static void initialize_contexts(void)
{
    const uint8_t data[] = "Hi";
    for (int i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        m_reliables[i].model_handle = TEST_HANDLE + i;
        m_reliables[i].message.length = sizeof(data);
        m_reliables[i].message.p_buffer = &data[0];
        m_reliables[i].message.opcode.opcode = 0x01 + i;
        m_reliables[i].message.opcode.company_id = ACCESS_COMPANY_ID_NONE;
        m_reliables[i].timeout = ACCESS_RELIABLE_TIMEOUT_MIN;
        m_reliables[i].status_cb = status_cb;
        m_reliables[i].reply_opcode.opcode = 0x01 + i;
        m_reliables[i].reply_opcode.company_id = i%2 > 0 ? ACCESS_COMPANY_ID_NONE : (0x01 + 1);
        timer_now_IgnoreAndReturn(i * TIME_SPACING);
        /* Expecting it twice -> first search for duplicate, second actually inserting. */
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();

        if (i == 0)
        {
            timer_reschedule_ExpectAndReturn(TIMER_STATE_STOPPED, ACCESS_RELIABLE_INTERVAL_DEFAULT);
        }

        uint8_t ttl = 0;

        if ((i & 1) == 0)
        {
            ttl = ACCESS_TTL_USE_DEFAULT;
            access_default_ttl_get_ExpectAndReturn(0);
        }

        access_model_publish_ttl_get_ExpectAndReturn(m_reliables[i].model_handle, NULL, NRF_SUCCESS);
        access_model_publish_ttl_get_IgnoreArg_p_ttl();
        access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&ttl);
        access_model_publish_ExpectAndReturn(m_reliables[i].model_handle, &m_reliables[i].message, NRF_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_publish(&m_reliables[i]));

        if (i == 0)
        {
            TEST_ASSERT_NOT_NULL(m_timer.p_evt);
            TEST_ASSERT_NOT_NULL(m_timer.p_evt->cb);
        }
    }
}

static void reliable_message_readd_cb(access_model_handle_t handle, void * p_args, access_reliable_status_t status)
{
    static bool trigger = false;

    printf("%s\n", (char *)p_args);
    TEST_ASSERT_EQUAL(ACCESS_RELIABLE_TRANSFER_TIMEOUT, status);

    if (!trigger)
    {
        uint8_t ttl = 0;
        trigger = true;
        access_model_publish_ttl_get_ExpectAndReturn(m_reliables[0].model_handle, NULL, NRF_SUCCESS);
        access_model_publish_ttl_get_IgnoreArg_p_ttl();
        access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&ttl);
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        timer_now_ExpectAndReturn(0);
        access_model_publish_ExpectAndReturn(m_reliables[0].model_handle, &m_reliables[0].message, NRF_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_publish(&m_reliables[0]));
    }
}

static void verify_callbacks(void)
{
    char error_message[512];
    sprintf(error_message, "Status callback called less times than expected %u.", m_status_cb.num_calls);
    TEST_ASSERT_MESSAGE(m_status_cb.num_calls == 0, error_message);
    sprintf(error_message, "Timer (re)schedule called less times than expected %u.", m_timer.sch_calls);
    TEST_ASSERT_MESSAGE(m_timer.sch_calls == 0, error_message);
    sprintf(error_message, "Timer abort called less times than expected %u.", m_timer.abort_calls);
    TEST_ASSERT_MESSAGE(m_timer.abort_calls == 0, error_message);
}

void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
    bearer_event_mock_Init();
    memset(&m_timer, 0, sizeof(m_timer));
    memset(&m_status_cb, 0, sizeof(m_status_cb));
    access_reliable_init();
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_cb);
    timer_sch_abort_StubWithCallback(timer_sch_abort_cb);
}

void tearDown(void)
{
    verify_callbacks();
    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
}

/* ******************* Test functions ******************* */

void __test_timeout_fire_round(uint32_t time_now)
{
    uint32_t i;
    for (i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT - 1; ++i)
    {
        timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING, time_now + (i + 1)*TIME_SPACING);
        access_model_publish_ExpectAndReturn(m_reliables[i].model_handle, &m_reliables[i].message, NRF_SUCCESS);
        fire_timeout(time_now + i * TIME_SPACING, NULL);
    }

    access_model_publish_ExpectAndReturn(m_reliables[i].model_handle, &m_reliables[i].message, NRF_SUCCESS);
    if (TIMER_OLDER_THAN(m_reliables[i].timeout, time_now * ACCESS_RELIABLE_BACK_OFF_FACTOR))
    {
        timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING,
                                         m_reliables[i].timeout);
    }
    else
    {
        timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING,
                                         time_now * ACCESS_RELIABLE_BACK_OFF_FACTOR);
    }
    fire_timeout(time_now + i * TIME_SPACING, NULL);

}

void test_reliable_publish(void)
{
    printf("\n----------------Start Test---------------\n");
    /* Initialize all contexts */
    initialize_contexts();
    /* Should be ACCESS_RELIABLE_TRANSFER_COUNT spaced TIME_SPACING in time
     *
     * |---------|----------...------------------------|------------------------>
     * 0   TIME_SPACING*1        TIME_SPACING*ACCESS_RELIABLE_TRANSFER_COUNT   t
     */

    printf("---> Fire all except last timeout.\n");
    uint32_t time = ACCESS_RELIABLE_INTERVAL_DEFAULT;
    for (; time < ACCESS_RELIABLE_TIMEOUT_MIN; time *= ACCESS_RELIABLE_BACK_OFF_FACTOR)
    {
        __test_timeout_fire_round(time);
    }
    verify_callbacks();

    printf("---> Let the first, last and middle transfer succeed.\n");
    time = ACCESS_RELIABLE_TIMEOUT_MIN;
    /* . */
    uint32_t remove_bitfield = (1 << 0) | (1 << (ACCESS_RELIABLE_TRANSFER_COUNT-1)) | (1 << (ACCESS_RELIABLE_TRANSFER_COUNT / 2));
    access_message_rx_t rx_message = {{0}};
    for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (((1 << i) & remove_bitfield) == 0)
        {
            continue;
        }

        access_reliable_t * p_ctx = &m_reliables[i];
        rx_message.opcode.opcode = p_ctx->reply_opcode.opcode;
        rx_message.opcode.company_id = p_ctx->reply_opcode.company_id;
        void * p_args = (uint8_t *) TEST_ARGS_PTR + i;
        if (i == 0)
        {
            /* Expect a reschedule when clearing HEAD */
            timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING, time + TIME_SPACING);
        }

        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        status_cb_Expect(p_ctx->model_handle, p_args, ACCESS_RELIABLE_TRANSFER_SUCCESS);
        access_reliable_message_rx_cb(p_ctx->model_handle, &rx_message, p_args);
    }
    verify_callbacks();

    printf("---> Timeout the remaining contexts.\n");
    void * p_args;
    /* Timeout the remaining contexts */
    uint32_t remaining_bitfield = (~remove_bitfield) & (~(1 << ACCESS_RELIABLE_TRANSFER_COUNT));
    for (int i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if ((remaining_bitfield & (1 << i)) > 0)
        {
            if ((remaining_bitfield & (1 << (i+1))) > 0)
            {
                /* The next is not removed */
                timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING,
                                                 ACCESS_RELIABLE_TIMEOUT_MIN + TIME_SPACING*(i + 1));
            }
            else if ((remaining_bitfield & (1 << (i+2))) > 0)
            {
                /* The one after the next is not removed */
                timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING,
                                                 ACCESS_RELIABLE_TIMEOUT_MIN + TIME_SPACING*(i + 2));
            }

            p_args = (uint8_t *) TEST_ARGS_PTR + i;
            printf("p_args: %p\n", p_args);
            access_model_p_args_get_ExpectAndReturn(m_reliables[i].model_handle, NULL, NRF_SUCCESS);
            access_model_p_args_get_IgnoreArg_pp_args();
            access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
            status_cb_Expect(m_reliables[i].model_handle, p_args, ACCESS_RELIABLE_TRANSFER_TIMEOUT);
            fire_timeout(time + i*TIME_SPACING, NULL);
        }
    }
}

void test_multiple_cancels(void)
{
    /* Initialize all contexts */
    initialize_contexts();
    /* Remove again */
    for (int i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (i < ACCESS_RELIABLE_TRANSFER_COUNT-1)
        {
            timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING, ACCESS_RELIABLE_INTERVAL_DEFAULT + (i + 1)*TIME_SPACING);
        }
        else
        {
            __timer_abort_ExpectAndReturn(TIMER_STATE_RUNNING);
        }
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();

        void * p_args = (uint8_t *) TEST_ARGS_PTR + i;
        access_model_p_args_get_ExpectAndReturn(m_reliables[i].model_handle, NULL, NRF_SUCCESS);
        access_model_p_args_get_IgnoreArg_pp_args();
        access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
        status_cb_Expect(m_reliables[i].model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);

        (void) access_model_reliable_cancel(m_reliables[i].model_handle);
    }

    initialize_contexts();
    /*  Reverse order. */
    int i = ACCESS_RELIABLE_TRANSFER_COUNT-1;
    for (; i >= 0; --i)
    {
        if (i == 0)
        {
            __timer_abort_ExpectAndReturn(TIMER_STATE_RUNNING);
        }
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();

        void * p_args = (uint8_t *) TEST_ARGS_PTR + i;
        access_model_p_args_get_ExpectAndReturn(m_reliables[i].model_handle, NULL, NRF_SUCCESS);
        access_model_p_args_get_IgnoreArg_pp_args();
        access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
        status_cb_Expect(m_reliables[i].model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);

        (void) access_model_reliable_cancel(m_reliables[i].model_handle);
    }

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    i = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_reliable_cancel(m_reliables[i].model_handle));
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_reliable_cancel(ACCESS_MODEL_COUNT+1));
}


void test_error_conditions(void)
{
    access_reliable_t * p_reliable = &m_reliables[0];

    p_reliable->status_cb = NULL;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_reliable_publish(NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, access_model_reliable_publish(p_reliable));

    p_reliable->status_cb = status_cb;
    p_reliable->model_handle = ACCESS_MODEL_COUNT;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, access_model_reliable_publish(p_reliable));
    p_reliable->model_handle = ACCESS_MODEL_COUNT-1;

    p_reliable->timeout = ACCESS_RELIABLE_TIMEOUT_MIN - 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_reliable_publish(p_reliable));
    p_reliable->timeout = ACCESS_RELIABLE_TIMEOUT_MAX + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, access_model_reliable_publish(p_reliable));
    p_reliable->timeout = ACCESS_RELIABLE_TIMEOUT_MAX;

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    uint32_t error_code = 0xCAFEBABE;
    access_model_publish_ExpectAndReturn(p_reliable->model_handle, &p_reliable->message, error_code);
    TEST_ASSERT_EQUAL(error_code, access_model_reliable_publish(p_reliable));

    /* Access should retry silently if it gets NRF_ERROR_NO_MEM */
    error_code = NRF_ERROR_NO_MEM;
    timer_now_IgnoreAndReturn(0);
    timer_reschedule_ExpectAndReturn(TIMER_STATE_STOPPED, ACCESS_RELIABLE_INTERVAL_DEFAULT);
    /* Expecting it twice -> first search for duplicate, second actually inserting. */
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    uint8_t ttl = 0;
    access_model_publish_ttl_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_publish_ttl_get_IgnoreArg_p_ttl();
    access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&ttl);
    access_model_publish_ExpectAndReturn(p_reliable->model_handle, &p_reliable->message, error_code);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_publish(p_reliable));

    __timer_abort_ExpectAndReturn(TIMER_STATE_RUNNING);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    void * p_args = (uint8_t *) TEST_ARGS_PTR;
    access_model_p_args_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_p_args_get_IgnoreArg_pp_args();
    access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
    status_cb_Expect(p_reliable->model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_cancel(p_reliable->model_handle));
    initialize_contexts();

    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, access_model_reliable_publish(p_reliable));
}

void test_no_mem_retry(void)
{
    initialize_contexts();

    timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING, ACCESS_RELIABLE_INTERVAL_DEFAULT + ACCESS_RELIABLE_RETRY_DELAY);
    access_model_publish_ExpectAndReturn(m_reliables[0].model_handle, &m_reliables[0].message, NRF_ERROR_NO_MEM);
    fire_timeout(ACCESS_RELIABLE_INTERVAL_DEFAULT, NULL);

    access_model_publish_ExpectAndReturn(m_reliables[0].model_handle, &m_reliables[0].message, NRF_ERROR_NULL);
    TEST_NRF_MESH_ASSERT_EXPECT(fire_timeout(ACCESS_RELIABLE_INTERVAL_DEFAULT + ACCESS_RELIABLE_RETRY_DELAY, NULL));

}

void test_sar_packet(void)
{
    const access_opcode_t OPCODE_1BYTE = {0x0001, ACCESS_COMPANY_ID_NONE};
    const access_opcode_t OPCODE_2BYTE = {0x8001, ACCESS_COMPANY_ID_NONE};
    const access_opcode_t OPCODE_3BYTE = {0x0001, 0x1337};
    const uint8_t data[128] = {0};
    uint8_t ttl = 0;
    access_reliable_t * p_reliable = &m_reliables[0];
    void * p_args;
    p_reliable->model_handle = 0;
    p_reliable->reply_opcode.opcode = 0x02;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->status_cb = status_cb;
    p_reliable->timeout = ACCESS_RELIABLE_TIMEOUT_MIN;

    /* 1 byte opcode should push it just over the limit to SAR */
    memcpy(&p_reliable->message.opcode, &OPCODE_1BYTE, sizeof(access_opcode_t));
    p_reliable->message.p_buffer = data;
    p_reliable->message.length   = NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX;

    uint32_t expected_timeout = (ACCESS_RELIABLE_INTERVAL_DEFAULT
                                 + ACCESS_RELIABLE_SEGMENT_COUNT_PENALTY);

    timer_reschedule_ExpectAndReturn(TIMER_STATE_STOPPED, expected_timeout);
    access_model_publish_ttl_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_publish_ttl_get_IgnoreArg_p_ttl();
    access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&ttl);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    timer_now_ExpectAndReturn(0);
    access_model_publish_ExpectAndReturn(p_reliable->model_handle, &p_reliable->message, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_publish(p_reliable));

    __timer_abort_ExpectAndReturn(TIMER_STATE_RUNNING);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    p_args = (uint8_t *) TEST_ARGS_PTR;
    access_model_p_args_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_p_args_get_IgnoreArg_pp_args();
    access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
    status_cb_Expect(p_reliable->model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_cancel(p_reliable->model_handle));

    /* 2 byte opcode should push it just over the limit to SAR */
    uint32_t expected_segments = 3;
    memcpy(&p_reliable->message.opcode, &OPCODE_2BYTE, sizeof(access_opcode_t));
    p_reliable->message.p_buffer = data;
    p_reliable->message.length   = PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE * (expected_segments - 1) - 1;

    expected_timeout = (ACCESS_RELIABLE_INTERVAL_DEFAULT
                        + ACCESS_RELIABLE_SEGMENT_COUNT_PENALTY * expected_segments);
    timer_reschedule_ExpectAndReturn(TIMER_STATE_STOPPED, expected_timeout);
    access_model_publish_ttl_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_publish_ttl_get_IgnoreArg_p_ttl();
    access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&ttl);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    timer_now_ExpectAndReturn(0);
    access_model_publish_ExpectAndReturn(p_reliable->model_handle, &p_reliable->message, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_publish(p_reliable));

    __timer_abort_ExpectAndReturn(TIMER_STATE_RUNNING);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    p_args = (uint8_t *) TEST_ARGS_PTR;
    access_model_p_args_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_p_args_get_IgnoreArg_pp_args();
    access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
    status_cb_Expect(p_reliable->model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_cancel(p_reliable->model_handle));

    /* 3 byte opcode should push it just over the limit to SAR */
    expected_segments = 5;
    memcpy(&p_reliable->message.opcode, &OPCODE_3BYTE, sizeof(access_opcode_t));
    p_reliable->message.p_buffer = data;
    p_reliable->message.length   = PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE * (expected_segments - 1) - 2;

    expected_timeout = (ACCESS_RELIABLE_INTERVAL_DEFAULT
                        + ACCESS_RELIABLE_SEGMENT_COUNT_PENALTY * expected_segments);
    timer_reschedule_ExpectAndReturn(TIMER_STATE_STOPPED, expected_timeout);
    access_model_publish_ttl_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_publish_ttl_get_IgnoreArg_p_ttl();
    access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&ttl);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    timer_now_ExpectAndReturn(0);
    access_model_publish_ExpectAndReturn(p_reliable->model_handle, &p_reliable->message, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_publish(p_reliable));

    __timer_abort_ExpectAndReturn(TIMER_STATE_RUNNING);
    bearer_event_critical_section_begin_Expect();
    bearer_event_critical_section_end_Expect();
    p_args = (uint8_t *) TEST_ARGS_PTR;
    access_model_p_args_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
    access_model_p_args_get_IgnoreArg_pp_args();
    access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
    status_cb_Expect(p_reliable->model_handle, p_args, ACCESS_RELIABLE_TRANSFER_CANCELLED);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_cancel(p_reliable->model_handle));
}

void test_rx_wrong_opcode(void)
{
    const uint16_t WRONG_OPCODE = 0x8765;
    /* Initialize all contexts */
    initialize_contexts();

    access_message_rx_t rx_message = {{0}};
    uint32_t time = ACCESS_RELIABLE_INTERVAL_DEFAULT;

    for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        access_reliable_t * p_ctx = &m_reliables[i];
        void * p_args = (uint8_t *) TEST_ARGS_PTR + i;

        rx_message.opcode.opcode = WRONG_OPCODE;
        rx_message.opcode.company_id = ACCESS_COMPANY_ID_NONE;
        /* Fail first */
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        access_reliable_message_rx_cb(p_ctx->model_handle, &rx_message, p_args);


        if (i < ACCESS_RELIABLE_TRANSFER_COUNT - 1)
        {
            /* Expect a reschedule when clearing HEAD */
            timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING, time + TIME_SPACING);
        }
        else
        {
            /* Last one should abort the timer */
            __timer_abort_ExpectAndReturn(TIMER_STATE_RUNNING);
        }

        /* Succeed */
        rx_message.opcode.opcode = p_ctx->reply_opcode.opcode;
        rx_message.opcode.company_id = p_ctx->reply_opcode.company_id;

        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        status_cb_Expect(p_ctx->model_handle, p_args, ACCESS_RELIABLE_TRANSFER_SUCCESS);
        access_reliable_message_rx_cb(p_ctx->model_handle, &rx_message, p_args);

        time += TIME_SPACING;

    }
    verify_callbacks();
}

void test_access_reliable_model_is_free(void)
{
    printf("\n----------------Start Test---------------\n");
    /* Initialize all contexts */
    initialize_contexts();
    /* Should be ACCESS_RELIABLE_TRANSFER_COUNT spaced TIME_SPACING in time
     *
     * |---------|----------...------------------------|------------------------>
     * 0   TIME_SPACING*1        TIME_SPACING*ACCESS_RELIABLE_TRANSFER_COUNT   t
     */

    printf("---> Fire all except last timeout.\n");
    uint32_t time = ACCESS_RELIABLE_INTERVAL_DEFAULT;
    for (; time < ACCESS_RELIABLE_TIMEOUT_MIN; time *= ACCESS_RELIABLE_BACK_OFF_FACTOR)
    {
        __test_timeout_fire_round(time);
    }
    verify_callbacks();

    /* Test if the model context are free */
    for (uint32_t i=0; i<ACCESS_RELIABLE_TRANSFER_COUNT; i++)
    {
        TEST_ASSERT_EQUAL(false, access_reliable_model_is_free(m_reliables[i].model_handle));
    }

    printf("---> Let the first, last and middle transfer succeed.\n");
    time = ACCESS_RELIABLE_TIMEOUT_MIN;
    /* . */
    uint32_t remove_bitfield = (1 << 0) | (1 << (ACCESS_RELIABLE_TRANSFER_COUNT-1)) | (1 << (ACCESS_RELIABLE_TRANSFER_COUNT / 2));
    access_message_rx_t rx_message = {{0}};
    for (uint32_t i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if (((1 << i) & remove_bitfield) == 0)
        {
            continue;
        }

        access_reliable_t * p_ctx = &m_reliables[i];
        rx_message.opcode.opcode = p_ctx->reply_opcode.opcode;
        rx_message.opcode.company_id = p_ctx->reply_opcode.company_id;
        void * p_args = (uint8_t *) TEST_ARGS_PTR + i;
        if (i == 0)
        {
            /* Expect a reschedule when clearing HEAD */
            timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING, time + TIME_SPACING);
        }

        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        status_cb_Expect(p_ctx->model_handle, p_args, ACCESS_RELIABLE_TRANSFER_SUCCESS);
        access_reliable_message_rx_cb(p_ctx->model_handle, &rx_message, p_args);
    }
    verify_callbacks();

    /* Test if the model context are free for the first last and middle contexts, and others are busy */
    TEST_ASSERT_EQUAL(true, access_reliable_model_is_free(m_reliables[0].model_handle));
    TEST_ASSERT_EQUAL(true, access_reliable_model_is_free(m_reliables[(ACCESS_RELIABLE_TRANSFER_COUNT-1)].model_handle));
    TEST_ASSERT_EQUAL(true, access_reliable_model_is_free(m_reliables[(ACCESS_RELIABLE_TRANSFER_COUNT/2)].model_handle));
    for (uint32_t i=0; i<ACCESS_RELIABLE_TRANSFER_COUNT; i++)
    {
        if (i == 0 || i == (ACCESS_RELIABLE_TRANSFER_COUNT-1) || i == (ACCESS_RELIABLE_TRANSFER_COUNT/2))
        {
            continue;
        }
        TEST_ASSERT_EQUAL(false, access_reliable_model_is_free(m_reliables[i].model_handle));
    }

    printf("---> Timeout the remaining contexts.\n");
    void * p_args;
    /* Timeout the remaining contexts */
    uint32_t remaining_bitfield = (~remove_bitfield) & (~(1 << ACCESS_RELIABLE_TRANSFER_COUNT));
    for (int i = 0; i < ACCESS_RELIABLE_TRANSFER_COUNT; ++i)
    {
        if ((remaining_bitfield & (1 << i)) > 0)
        {
            if ((remaining_bitfield & (1 << (i+1))) > 0)
            {
                /* The next is not removed */
                timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING,
                                                 ACCESS_RELIABLE_TIMEOUT_MIN + TIME_SPACING*(i + 1));
            }
            else if ((remaining_bitfield & (1 << (i+2))) > 0)
            {
                /* The one after the next is not removed */
                timer_reschedule_ExpectAndReturn(TIMER_STATE_RUNNING,
                                                 ACCESS_RELIABLE_TIMEOUT_MIN + TIME_SPACING*(i + 2));
            }

            p_args = (uint8_t *) TEST_ARGS_PTR + i;
            printf("p_args: %p\n", p_args);
            access_model_p_args_get_ExpectAndReturn(m_reliables[i].model_handle, NULL, NRF_SUCCESS);
            access_model_p_args_get_IgnoreArg_pp_args();
            access_model_p_args_get_ReturnThruPtr_pp_args(&p_args);
            status_cb_Expect(m_reliables[i].model_handle, p_args, ACCESS_RELIABLE_TRANSFER_TIMEOUT);
            fire_timeout(time + i*TIME_SPACING, NULL);
        }
    }

    /* Test if the model context are free */
    for (uint32_t i=0; i<ACCESS_RELIABLE_TRANSFER_COUNT; i++)
    {
        TEST_ASSERT_EQUAL(true, access_reliable_model_is_free(m_reliables[i].model_handle));
    }
}

void test_readding_reliable_transac_in_timer_cb(void)
{
    const access_opcode_t opcode_3bytes[2] = {{0x0001, 0x1337}, {0x0002, 0x1338}};
    const uint8_t data[2][NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX] = {{0}};
    access_reliable_t * p_reliable;
    uint8_t ttl = 0;
    uint32_t expected_timeout = ACCESS_RELIABLE_INTERVAL_DEFAULT;
    timer_state_t timer_state = 0;

    for (uint8_t i = 0; i < 2; i++)
    {
        p_reliable = &m_reliables[i];

        p_reliable->model_handle = i;
        p_reliable->status_cb = reliable_message_readd_cb;
        p_reliable->timeout = ACCESS_RELIABLE_TIMEOUT_MIN + 1;

        /* 1 byte opcode should push it just over the limit to SAR */
        memcpy(&p_reliable->message.opcode, &opcode_3bytes[i], sizeof(access_opcode_t));
        p_reliable->message.p_buffer = &data[i][0];
        p_reliable->message.length   = NRF_MESH_UNSEG_PAYLOAD_SIZE_MAX -
                access_utils_opcode_size_get(p_reliable->message.opcode);

        timer_reschedule_ExpectAndReturn(timer_state++, expected_timeout);
        access_model_publish_ttl_get_ExpectAndReturn(p_reliable->model_handle, NULL, NRF_SUCCESS);
        access_model_publish_ttl_get_IgnoreArg_p_ttl();
        access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&ttl);
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        bearer_event_critical_section_begin_Expect();
        bearer_event_critical_section_end_Expect();
        timer_now_ExpectAndReturn(0);
        access_model_publish_ExpectAndReturn(p_reliable->model_handle, &p_reliable->message, NRF_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, access_model_reliable_publish(p_reliable));
    }

    const char * message[] = {"The first reliable message fired", "The second reliable message fired"};
    for (uint8_t i = 0; i < 2; i++)
    {
        access_model_p_args_get_ExpectAndReturn(m_reliables[i].model_handle, NULL, NRF_SUCCESS);
        access_model_p_args_get_IgnoreArg_pp_args();
        access_model_p_args_get_ReturnThruPtr_pp_args((void **)&message[i]);
    }
    m_timer.p_evt->cb(ACCESS_RELIABLE_TIMEOUT_MIN + 2, NULL);
    TEST_ASSERT_EQUAL(TIMER_DIFF(ACCESS_RELIABLE_INTERVAL_DEFAULT, ACCESS_RELIABLE_TIMEOUT_MIN + 2), m_timer.p_evt->interval);
}
