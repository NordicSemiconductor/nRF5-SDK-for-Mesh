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

#include "mesh_flash.h"
#include "nrf_flash_mock.h"
#include "bearer_event.h"
#include "bearer_handler_mock.h"
#include "timeslot_timer_mock.h"
#include "bl_if.h"
#include "utils.h"
#include "test_assert.h"

#define START_TIME                      (123)
#define FLASH_PROCESS_TIME_OVERHEAD     (500)
#define FLASH_TIME_TO_ERASE_PAGE_US     (20000)
#define FLASH_TIME_TO_WRITE_ONE_WORD_US (50)
#define BEARER_ACTION_WRITE_MAX_WORDS   ((BEARER_ACTION_DURATION_MAX_US - FLASH_PROCESS_TIME_OVERHEAD) / FLASH_TIME_TO_WRITE_ONE_WORD_US)
#define BEARER_ACTION_ERASE_MAX_PAGES   ((BEARER_ACTION_DURATION_MAX_US - FLASH_PROCESS_TIME_OVERHEAD) / FLASH_TIME_TO_ERASE_PAGE_US)
#define TEST_WRITE_MAX_WORDS            (3 * BEARER_ACTION_WRITE_MAX_WORDS + 1)
#define TEST_ERASE_MAX_PAGES            (3 * BEARER_ACTION_ERASE_MAX_PAGES + 1)
#define NEXT_BEARER_ACTION_DELAY        (2 * BEARER_ACTION_DURATION_MAX_US)

typedef struct
{
    mesh_flash_user_t user;
    uint32_t expected_cb_count;
    uint32_t cb_all_count;
    flash_operation_t expected_op;
    uint16_t expected_callback_token;
} flash_user_end_expect_t;

static flash_user_end_expect_t m_end_expect_users[2];
static bearer_event_flag_callback_t m_event_cb;
static bool m_delayed_flag_event;
static uint32_t m_bearer_handler_action_enqueue_callback_cnt;
static uint32_t m_bearer_handler_action_enqueue_callback_expected_cnt;
static bearer_action_t * mp_bearer_action[MESH_FLASH_USERS];
static mesh_flash_user_t m_bearer_handler_action_enqueue_callback_expected_user;


extern void mesh_flash_reset(void);

void setUp(void)
{
    m_event_cb = NULL;
    m_bearer_handler_action_enqueue_callback_cnt = 0;
    m_bearer_handler_action_enqueue_callback_expected_cnt = 0;
    m_bearer_handler_action_enqueue_callback_expected_user = MESH_FLASH_USERS;  /* I.e. invalid value */
    memset(m_end_expect_users, 0, sizeof(m_end_expect_users));
    memset(mp_bearer_action, 0, sizeof(mp_bearer_action));
    m_end_expect_users[0].user = MESH_FLASH_USER_TEST;
    m_end_expect_users[1].user = MESH_FLASH_USER_DFU;
    m_delayed_flag_event = false;
    nrf_flash_mock_Init();
    timeslot_timer_mock_Init();
    bearer_handler_mock_Init();
}

void tearDown(void)
{
    mesh_flash_reset();
    nrf_flash_mock_Verify();
    nrf_flash_mock_Destroy();
    timeslot_timer_mock_Verify();
    timeslot_timer_mock_Destroy();
    bearer_handler_mock_Verify();
    bearer_handler_mock_Destroy();
}

static void mesh_flash_op_cb(mesh_flash_user_t user, const flash_operation_t * p_op, uint16_t token)
{
    /* fetch the relevant expect-values for the incoming user */
    flash_user_end_expect_t * p_expect = NULL;
    for (uint32_t i = 0; i < 2; i++)
    {
        if (m_end_expect_users[i].user == user)
        {
            p_expect = &m_end_expect_users[i];
            break;
        }
    }
    TEST_ASSERT_NOT_NULL(p_expect); /* Unknown user */


    if (p_expect->expected_cb_count == 0) /** Always expect a final "all operations" call after the expected count. */
    {
        TEST_ASSERT_EQUAL(FLASH_OP_TYPE_ALL, p_op->type);
        TEST_ASSERT_EQUAL_PTR(0, p_op->params.write.p_start_addr);
        TEST_ASSERT_EQUAL_PTR(0, p_op->params.write.p_data);
        TEST_ASSERT_EQUAL_PTR(0, p_op->params.write.length);
        p_expect->cb_all_count++;
    }
    else
    {
        TEST_ASSERT_EQUAL(p_expect->expected_callback_token, token);
        p_expect->expected_callback_token++;
        TEST_ASSERT(p_expect->expected_cb_count > 0);
        p_expect->expected_cb_count--;
        if (p_expect->expected_op.type == FLASH_OP_TYPE_WRITE)
        {
            TEST_ASSERT_EQUAL_MEMORY(&p_expect->expected_op, p_op, offsetof(flash_operation_t, params) + sizeof(p_op->params.write));
        }
        else
        {
            TEST_ASSERT_EQUAL_MEMORY(&p_expect->expected_op, p_op, offsetof(flash_operation_t, params) + sizeof(p_op->params.erase));
        }
    }
}

bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t cb)
{
    TEST_ASSERT_NOT_NULL(cb);
    m_event_cb = cb;
    return 0x1234;
}

void bearer_event_flag_set(bearer_event_flag_t flag)
{
    TEST_ASSERT_NOT_NULL(m_event_cb);
    if (!m_delayed_flag_event)
    {
        m_event_cb();
    }
}

static uint32_t max_write_words_in_remaining_bearer_action(mesh_flash_user_t user, uint32_t start_time, uint32_t current_time)
{
    if (current_time - start_time + FLASH_PROCESS_TIME_OVERHEAD > mp_bearer_action[user]->duration_us)
    {
        return 0;
    }
    else
    {
        return (mp_bearer_action[user]->duration_us - current_time + start_time - FLASH_PROCESS_TIME_OVERHEAD) / FLASH_TIME_TO_WRITE_ONE_WORD_US;
    }
}

static uint32_t max_erase_pages_in_remaining_bearer_action(mesh_flash_user_t user, uint32_t start_time, uint32_t current_time)
{
    if (current_time - start_time + FLASH_PROCESS_TIME_OVERHEAD > mp_bearer_action[user]->duration_us)
    {
        return 0;
    }
    else
    {
        return (mp_bearer_action[user]->duration_us - current_time + start_time - FLASH_PROCESS_TIME_OVERHEAD) / FLASH_TIME_TO_ERASE_PAGE_US;
    }
}

static uint32_t bearer_handler_action_enqueue_callback(bearer_action_t* p_action, int cmock_num_calls)
{
    m_bearer_handler_action_enqueue_callback_cnt++;
    mp_bearer_action[m_bearer_handler_action_enqueue_callback_expected_user] = p_action;

    return NRF_SUCCESS;
}

static void bearer_handler_action_enqueue_expect(mesh_flash_user_t user)
{
    m_bearer_handler_action_enqueue_callback_expected_user = user;
    m_bearer_handler_action_enqueue_callback_expected_cnt++;
}

/******** Tests ********/
void test_op_push(void)
{
    flash_operation_t flash_op;
    uint8_t data[4];
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = (uint32_t *) 0xADD1ADD0;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    uint16_t token = 0xFFFF;

    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, NULL, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.type = FLASH_OP_TYPE_NONE;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.type = FLASH_OP_TYPE_ALL;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = (uint32_t *) 0xADD1ADD1;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.params.write.p_start_addr = (uint32_t *) 0xADD0ADD0;
    flash_op.params.write.length = 1;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.params.write.length = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    bearer_handler_action_enqueue_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    flash_op.params.write.length = 4;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0, token);

    token = 0xFFFF;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USERS, &flash_op, &token)); /* invalid user */
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.params.erase.p_start_addr = (uint32_t *) 0xADD1ADD1;
    flash_op.params.erase.length = 4;
    flash_op.type = FLASH_OP_TYPE_ERASE;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.params.erase.p_start_addr = (uint32_t *) 0xADD1ADD0; /* Only page aligned erases allowed (word aligned shouldn't) */
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.params.erase.p_start_addr = (uint32_t *) 0xADD1A000;
    flash_op.params.erase.length = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    flash_op.params.erase.length = 4; // must be page aligned too
    TEST_NRF_MESH_ASSERT_EXPECT(mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(0xFFFF, token);

    /* Reset module to flush queue. */
    mesh_flash_reset();

    /* test full queue */
    uint16_t expected_token = 0;
    flash_op.params.erase.length = 1024;
    bearer_handler_action_enqueue_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    while (mesh_flash_op_available_slots(MESH_FLASH_USER_TEST))
    {
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        TEST_ASSERT_EQUAL(expected_token, token);
        expected_token++;
    }
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_NO_MEM, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
}

void test_execute_write(void)
{
    static uint32_t dest[TEST_WRITE_MAX_WORDS] __attribute__((aligned(PAGE_SIZE)));
    static uint8_t data[TEST_WRITE_MAX_WORDS * 4];

    for (uint32_t i = 0; i < TEST_WRITE_MAX_WORDS; ++i)
    {
        dest[i] = i + 0xAB000000;
    }

    for (uint32_t i = 0; i < sizeof(data); i += 4)
    {
        data[i]   = 0xab;
        data[i+1] = 0xcd;
        data[i+2] = 0xef;
        data[i+3] = 0x01;
    }

    uint16_t token = 0xFFFF;
    uint16_t expected_token = 0;

    flash_operation_t flash_op;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;

    typedef struct
    {
        uint32_t length;
        uint32_t process_time_overhead;
        uint32_t time_per_word;
    } test_vector_t;

    const test_vector_t test_vector[] = {
        { 4,                                       0, 0 },
        { 8,                                       0, 0 },
        { BEARER_ACTION_WRITE_MAX_WORDS * 4,       0, 0 },
        { (BEARER_ACTION_WRITE_MAX_WORDS + 1) * 4, 0, 0 },
        { TEST_WRITE_MAX_WORDS * 4,                0, 0 },

        { 4,                                       FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_WRITE_ONE_WORD_US / 5 },
        { 8,                                       FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_WRITE_ONE_WORD_US / 5 },
        { BEARER_ACTION_WRITE_MAX_WORDS * 4,       FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_WRITE_ONE_WORD_US / 5 },
        { (BEARER_ACTION_WRITE_MAX_WORDS + 1) * 4, FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_WRITE_ONE_WORD_US / 5 },
        { TEST_WRITE_MAX_WORDS * 4,                FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_WRITE_ONE_WORD_US / 5 },

        { 4,                                       FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_WRITE_ONE_WORD_US },
        { 8,                                       FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_WRITE_ONE_WORD_US },
        { BEARER_ACTION_WRITE_MAX_WORDS * 4,       FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_WRITE_ONE_WORD_US },
        { (BEARER_ACTION_WRITE_MAX_WORDS + 1) * 4, FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_WRITE_ONE_WORD_US },
        { TEST_WRITE_MAX_WORDS * 4,                FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_WRITE_ONE_WORD_US }
    };

    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    /* Run test vectors */
    for (uint32_t i = 0; i < ARRAY_SIZE(test_vector); ++i)
    {
        m_bearer_handler_action_enqueue_callback_cnt = 0;
        m_bearer_handler_action_enqueue_callback_expected_cnt = 0;

        /* Push flash operation */
        flash_op.params.write.length = test_vector[i].length;
        bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        TEST_ASSERT_EQUAL(expected_token, token);
        expected_token++;

        uint32_t words_so_far = 0;
        uint32_t words_remaining = flash_op.params.write.length / 4;
        uint32_t current_time = START_TIME;

        /* Handle callback(s) from bearer handler for required number of bearer actions */
        do
        {
            uint32_t start_time = current_time;

            /* Handle callback(s) from bearer handler for single bearer action */
            for (uint32_t j = 0; words_remaining > 0; j++)
            {
                uint32_t words = max_write_words_in_remaining_bearer_action(MESH_FLASH_USER_TEST, start_time, current_time);
                if (words > words_remaining)
                {
                    words = words_remaining;
                }
                if (words == 0)
                {
                    break;
                }
                nrf_flash_write_ExpectAndReturn(&dest[words_so_far],
                                                (uint32_t *)data + words_so_far,
                                                words * 4,
                                                NRF_SUCCESS);
                words_so_far += words;
                words_remaining -= words;
                current_time += test_vector[i].process_time_overhead + (words * test_vector[i].time_per_word);
                if (words_remaining > 0)
                {
                    ts_timer_now_ExpectAndReturn(current_time);
                }
            }
            bearer_handler_action_end_Expect();
            if (words_remaining > 0)
            {
                bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
            }
            m_end_expect_users[0].cb_all_count = 0;
            m_end_expect_users[0].expected_cb_count = 1;
            memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
            mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(start_time, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);

            current_time += NEXT_BEARER_ACTION_DELAY;

        } while (words_remaining > 0);

        TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    }

    bearer_handler_action_enqueue_StubWithCallback(NULL);
}

void test_execute_erase(void)
{
    uint32_t dest[TEST_ERASE_MAX_PAGES * PAGE_SIZE / 4] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < TEST_ERASE_MAX_PAGES * PAGE_SIZE / 4; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint16_t token = 0xFFFF;
    uint16_t expected_token = 0;

    flash_operation_t flash_op;
    flash_op.type = FLASH_OP_TYPE_ERASE;
    flash_op.params.erase.p_start_addr = dest;

    typedef struct
    {
        uint32_t length;
        uint32_t process_time_overhead;
        uint32_t time_per_page;
    } test_vector_t;

    const test_vector_t test_vector[] = {
        { PAGE_SIZE,                                       0, 0 },
        { PAGE_SIZE * 2,                                   0, 0 },
        { BEARER_ACTION_ERASE_MAX_PAGES * PAGE_SIZE,       0, 0 },
        { (BEARER_ACTION_ERASE_MAX_PAGES + 1) * PAGE_SIZE, 0, 0 },
        { TEST_ERASE_MAX_PAGES * PAGE_SIZE,                0, 0 },

        { PAGE_SIZE,                                       FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_ERASE_PAGE_US / 5 },
        { PAGE_SIZE * 2,                                   FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_ERASE_PAGE_US / 5 },
        { BEARER_ACTION_ERASE_MAX_PAGES * PAGE_SIZE,       FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_ERASE_PAGE_US / 5 },
        { (BEARER_ACTION_ERASE_MAX_PAGES + 1) * PAGE_SIZE, FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_ERASE_PAGE_US / 5 },
        { TEST_ERASE_MAX_PAGES * PAGE_SIZE,                FLASH_PROCESS_TIME_OVERHEAD / 2, FLASH_TIME_TO_ERASE_PAGE_US / 5 },

        { PAGE_SIZE,                                       FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_ERASE_PAGE_US },
        { PAGE_SIZE * 2,                                   FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_ERASE_PAGE_US },
        { BEARER_ACTION_ERASE_MAX_PAGES * PAGE_SIZE,       FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_ERASE_PAGE_US },
        { (BEARER_ACTION_ERASE_MAX_PAGES + 1) * PAGE_SIZE, FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_ERASE_PAGE_US },
        { TEST_ERASE_MAX_PAGES * PAGE_SIZE,                FLASH_PROCESS_TIME_OVERHEAD, FLASH_TIME_TO_ERASE_PAGE_US }
    };

    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    /* Run test vectors */
    for (uint32_t i = 0; i < ARRAY_SIZE(test_vector); ++i)
    {
        m_bearer_handler_action_enqueue_callback_cnt = 0;
        m_bearer_handler_action_enqueue_callback_expected_cnt = 0;

        /* Push flash operation */
        flash_op.params.erase.length = test_vector[i].length;
        bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
        TEST_ASSERT_EQUAL(expected_token, token);
        expected_token++;

        uint32_t pages_so_far = 0;
        uint32_t pages_remaining = flash_op.params.erase.length / PAGE_SIZE;
        uint32_t current_time = START_TIME;

        /* Handle callback(s) from bearer handler for required number of bearer actions */
        do
        {
            uint32_t start_time = current_time;

            /* Handle callback(s) from bearer handler for single bearer action */
            for (uint32_t j = 0; pages_remaining > 0; j++)
            {
                uint32_t pages = max_erase_pages_in_remaining_bearer_action(MESH_FLASH_USER_TEST, start_time, current_time);
                if (pages > pages_remaining)
                {
                    pages = pages_remaining;
                }
                if (pages == 0)
                {
                    break;
                }
                nrf_flash_erase_ExpectAndReturn(&dest[pages_so_far * PAGE_SIZE / 4],
                                                pages * PAGE_SIZE,
                                                NRF_SUCCESS);
                pages_so_far += pages;
                pages_remaining -= pages;
                current_time += test_vector[i].process_time_overhead + (pages * test_vector[i].time_per_page);
                if (pages_remaining > 0)
                {
                    ts_timer_now_ExpectAndReturn(current_time);
                }
            }
            bearer_handler_action_end_Expect();
            if (pages_remaining > 0)
            {
                bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
            }
            m_end_expect_users[0].cb_all_count = 0;
            m_end_expect_users[0].expected_cb_count = 1;
            memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
            mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(start_time, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);

            current_time += NEXT_BEARER_ACTION_DELAY;

        } while (pages_remaining > 0);

        TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    }

    bearer_handler_action_enqueue_StubWithCallback(NULL);
}

void test_available_slots(void)
{
    flash_operation_t flash_op;
    uint32_t dest;
    uint8_t data[4];
    uint16_t token = 0xFFFF;

    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = &dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;

    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    /* Check initial state */
    TEST_ASSERT_EQUAL(0, mesh_flash_op_available_slots(MESH_FLASH_USERS)); /* out of bounds, so there are no slots. */
    uint32_t available_slots = mesh_flash_op_available_slots(MESH_FLASH_USER_TEST);
    TEST_ASSERT_NOT_EQUAL(0, available_slots);

    /* Push flash operation */
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(available_slots - 1, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST));
    TEST_ASSERT_EQUAL(available_slots - 1, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST)); /* shouldn't alter anything. */

    /* Push second flash operation */
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(available_slots - 2, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST));

    /* Execute first flash operation */
    nrf_flash_write_ExpectAndReturn(&dest, (uint32_t*)data, 4, NRF_SUCCESS);
    bearer_handler_action_end_Expect();
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    m_end_expect_users[0].expected_cb_count = 1;
    mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(available_slots - 1, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST)); /* Back to full */

    /* Execute second flash operation */
    nrf_flash_write_ExpectAndReturn(&dest, (uint32_t*)data, 4, NRF_SUCCESS);
    bearer_handler_action_end_Expect();
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    m_end_expect_users[0].expected_cb_count = 1;
    mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(available_slots, mesh_flash_op_available_slots(MESH_FLASH_USER_TEST)); /* Back to full */

    TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    bearer_handler_action_enqueue_StubWithCallback(NULL);
}

void test_multiple_users(void)
{
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[1024] = {0xab, 0xcd, 0xef, 0x01};
    uint16_t token = 0xFFFF;

    flash_operation_t flash_op;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;

    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    mesh_flash_user_callback_set(MESH_FLASH_USER_DFU, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    memcpy(&m_end_expect_users[1].expected_op, &flash_op, sizeof(m_end_expect_users[1].expected_op));

    /* Push the same operation to two users */
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_DFU);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_DFU, &flash_op, &token));

    /* Execute first operation (the one with the lowest user index) */
    nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
    bearer_handler_action_end_Expect();
    m_end_expect_users[1].expected_cb_count = 1;
    m_end_expect_users[1].cb_all_count = 0;
    mp_bearer_action[MESH_FLASH_USER_DFU]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_DFU]->p_args);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[1].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[1].expected_cb_count);

    /* Execute the second */
    nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
    bearer_handler_action_end_Expect();
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);

    TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    bearer_handler_action_enqueue_StubWithCallback(NULL);
}

/** The module keeps track of its queue with indexes. These will
 * eventually roll over. Verify that this doesn't break the queue.
 */
void test_queue_index_rollover(void)
{
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[1024] = {0xab, 0xcd, 0xef, 0x01};
    uint16_t token = 0xFFFF;

    flash_operation_t flash_op;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;

    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));

    /* Keep pushing and processing (rollover happens at 256) */
    for (uint32_t i = 0; i < 270; i++)
    {
        /* Push operation */
        bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));

        /* Execute event */
        nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
        bearer_handler_action_end_Expect();
        m_end_expect_users[0].expected_cb_count = 1;
        m_end_expect_users[0].cb_all_count = 0;
        mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
        TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
        TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    }

    TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    bearer_handler_action_enqueue_StubWithCallback(NULL);
}

/** Delay execution of the async flag event, to let the module collect several
 * operations to report at the same time. Ensure that it pushes all the events
 * at a single execution step.
 */
void test_multifire(void)
{
    uint32_t dest[1024] __attribute__((aligned(PAGE_SIZE)));
    for (uint32_t i = 0; i < 1024; ++i)
    {
        dest[i] = i + 0xAB000000;
    }
    uint8_t data[1024] = {0xab, 0xcd, 0xef, 0x01};
    uint16_t token = 0xFFFF;

    flash_operation_t flash_op;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;

    m_delayed_flag_event = true;
    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));

    for (uint32_t i = 0; i < 4; i++)
    {
        /* Push operation */
        bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
        TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));

        /* Execute operation */
        nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
        bearer_handler_action_end_Expect();
        mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
    }

    /* Keep one operation in the pipeline, but fire the callback. The reports
     * on individual events should come, but not the final ALL events callback.
     * */
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));

    m_end_expect_users[0].expected_cb_count = 4;
    m_end_expect_users[0].cb_all_count = 0;
    m_event_cb();
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].cb_all_count); /* still one event in the pipeline. */
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);

    /* Execute last operation */
    nrf_flash_write_ExpectAndReturn(dest, (uint32_t *) data, 4, NRF_SUCCESS);
    bearer_handler_action_end_Expect();
    mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);

    /* Execute callback */
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    m_event_cb();
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count); /* All events fired. */
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);

    TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    bearer_handler_action_enqueue_StubWithCallback(NULL);
}

void test_suspend(void)
{
    uint32_t dest;
    uint8_t data[4];
    uint16_t token = 0xFFFF;

    flash_operation_t flash_op;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = &dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;

    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    /* Push operation */
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));

    /* Suspend, then execute the operation. Don't expect any flash operation to happen */
    mesh_flash_set_suspended(true);
    bearer_handler_action_end_Expect();
    mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
    nrf_flash_mock_Verify();

    /* Cancel suspension */
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
    mesh_flash_set_suspended(false);

    /* Execute operation */
    nrf_flash_write_ExpectAndReturn(&dest, (uint32_t*)data, 4, NRF_SUCCESS);
    bearer_handler_action_end_Expect();
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);

    TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    bearer_handler_action_enqueue_StubWithCallback(NULL);
}

void test_in_progress(void)
{
    uint32_t dest;
    uint8_t data[4];
    uint16_t token = 0xFFFF;

    flash_operation_t flash_op;
    flash_op.type = FLASH_OP_TYPE_WRITE;
    flash_op.params.write.p_start_addr = &dest;
    flash_op.params.write.p_data = (uint32_t *) data;
    flash_op.params.write.length = 4;

    mesh_flash_init();
    mesh_flash_user_callback_set(MESH_FLASH_USER_TEST, mesh_flash_op_cb);
    bearer_handler_action_enqueue_StubWithCallback(bearer_handler_action_enqueue_callback);

    /* Check initial in_progress state */
    TEST_ASSERT_EQUAL(false, mesh_flash_in_progress());

    /* Push operation */
    bearer_handler_action_enqueue_expect(MESH_FLASH_USER_TEST);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, mesh_flash_op_push(MESH_FLASH_USER_TEST, &flash_op, &token));
    TEST_ASSERT_EQUAL(true, mesh_flash_in_progress());

    /* Execute operation */
    nrf_flash_write_ExpectAndReturn(&dest, (uint32_t*)data, 4, NRF_SUCCESS);
    bearer_handler_action_end_Expect();
    m_end_expect_users[0].expected_cb_count = 1;
    m_end_expect_users[0].cb_all_count = 0;
    memcpy(&m_end_expect_users[0].expected_op, &flash_op, sizeof(m_end_expect_users[0].expected_op));
    mp_bearer_action[MESH_FLASH_USER_TEST]->start_cb(0, mp_bearer_action[MESH_FLASH_USER_TEST]->p_args);
    TEST_ASSERT_EQUAL(1, m_end_expect_users[0].cb_all_count);
    TEST_ASSERT_EQUAL(0, m_end_expect_users[0].expected_cb_count);
    TEST_ASSERT_EQUAL(false, mesh_flash_in_progress());

    TEST_ASSERT_EQUAL_UINT32(m_bearer_handler_action_enqueue_callback_expected_cnt, m_bearer_handler_action_enqueue_callback_cnt);
    bearer_handler_action_enqueue_StubWithCallback(NULL);
}
