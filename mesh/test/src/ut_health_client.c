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

#include <string.h>

#include <unity.h>
#include <cmock.h>

#include "utils.h"

#include "access_mock.h"
#include "access_config_mock.h"
#include "access_reliable_mock.h"
#include "nrf_mesh_mock.h"

#include "health_client.h"
#include "health_messages.h"
#include "health_opcodes.h"

#define TEST_MODEL_HANDLE   21
#define TEST_ELEMENT_INDEX  33

/********** Static variables and mock functions **********/

static void * mp_model_args;
static access_model_handle_t m_model_handle;
static const access_opcode_handler_t * mp_opcode_handlers;
static uint16_t m_num_opcodes;

static uint32_t access_model_add_mock(const access_model_add_params_t * p_init_params,
        access_model_handle_t * p_model_handle, int count)
{
    *p_model_handle = TEST_MODEL_HANDLE;
    m_model_handle = *p_model_handle;

    mp_opcode_handlers = p_init_params->p_opcode_handlers;
    m_num_opcodes = p_init_params->opcode_count;
    mp_model_args = p_init_params->p_args;

    TEST_ASSERT_EQUAL(HEALTH_CLIENT_MODEL_ID, p_init_params->model_id.model_id);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_init_params->model_id.company_id);
    TEST_ASSERT_EQUAL(TEST_ELEMENT_INDEX, p_init_params->element_index);

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

#define EXPECT_RELIABLE_TX(x_handle, x_opcode, x_reply_opcode, xp_expected_data, x_expected_data_length, x_status) \
    do { \
        TEST_ASSERT_FALSE(m_reliable_publish_expected); \
        m_reliable_publish_expected = true; \
        m_reliable_publish_expected_handle = x_handle; \
        m_reliable_publish_expected_opcode = x_opcode; \
        m_reliable_publish_expected_reply_opcode = x_reply_opcode; \
        mp_reliable_publish_expected_data = xp_expected_data; \
        m_reliable_publish_expected_data_length = x_expected_data_length; \
        m_reliable_publish_transfer_status = x_status; \
    } while(0)

static bool m_reliable_publish_expected = false;
static access_model_handle_t m_reliable_publish_expected_handle;
static uint16_t m_reliable_publish_expected_opcode;
static uint16_t m_reliable_publish_expected_reply_opcode;
static void * mp_reliable_publish_expected_data;
static uint16_t m_reliable_publish_expected_data_length;
static access_reliable_status_t m_reliable_publish_transfer_status;

static const access_reliable_t * mp_reliable_publish_context = NULL;

static uint32_t access_model_reliable_publish_mock(const access_reliable_t * p_reliable, int count)
{
    TEST_ASSERT_NOT_NULL(p_reliable);

    TEST_ASSERT_TRUE(m_reliable_publish_expected);
    m_reliable_publish_expected = false;

    TEST_ASSERT_EQUAL(m_reliable_publish_expected_handle, p_reliable->model_handle);
    TEST_ASSERT_EQUAL(m_reliable_publish_expected_opcode, p_reliable->message.opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reliable->message.opcode.company_id);
    TEST_ASSERT_EQUAL(m_reliable_publish_expected_reply_opcode, p_reliable->reply_opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reliable->reply_opcode.company_id);

    TEST_ASSERT_EQUAL(m_reliable_publish_expected_data_length, p_reliable->message.length);
    if(p_reliable->message.length > 0)
    {
        TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t *) mp_reliable_publish_expected_data,
                p_reliable->message.p_buffer, p_reliable->message.length);
    }

    TEST_ASSERT_NULL_MESSAGE(mp_reliable_publish_context,
            "Remember to call access_model_reliable_complete_transaction() after a reliable transfer!");
    mp_reliable_publish_context = p_reliable;
    return NRF_SUCCESS;
}

static void access_model_reliable_complete_transaction(void)
{
    TEST_ASSERT_NOT_NULL(mp_reliable_publish_context);
    mp_reliable_publish_context->status_cb(m_model_handle, mp_model_args, m_reliable_publish_transfer_status);
    mp_reliable_publish_context = NULL;
}

/********** Test initialization and finalization **********/

void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    access_reliable_mock_Init();
    nrf_mesh_mock_Init();

    nrf_mesh_unique_token_get_IgnoreAndReturn((nrf_mesh_tx_token_t)0x55AA55AAul);

    access_model_add_StubWithCallback(access_model_add_mock);
    access_model_publish_StubWithCallback(access_model_publish_mock);
    access_model_reliable_publish_StubWithCallback(access_model_reliable_publish_mock);

    access_model_subscription_list_alloc_ExpectAnyArgsAndReturn(NRF_SUCCESS);
}

void tearDown(void)
{
    TEST_ASSERT_FALSE(m_reliable_publish_expected);

    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    access_reliable_mock_Verify();
    access_reliable_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
}

/********** Health client event handler **********/

#define EXPECT_EVT_FAULT_STATUS_COMMON(x_evt_type, x_test_id, x_company_id, xp_fault_array, x_fault_array_length) \
    do { \
        m_event_expected = true; \
        m_expected_evt_type = x_evt_type; \
        m_expected_evt_data.fault_status.test_id = x_test_id; \
        m_expected_evt_data.fault_status.company_id = x_company_id; \
        m_expected_evt_data.fault_status.p_fault_array = xp_fault_array; \
        m_expected_evt_data.fault_status.fault_array_length = x_fault_array_length; \
    } while(0)

#define EXPECT_EVT_FAULT_STATUS(x_test_id, x_company_id, xp_fault_array, x_fault_array_length) \
    EXPECT_EVT_FAULT_STATUS_COMMON(HEALTH_CLIENT_EVT_TYPE_FAULT_STATUS_RECEIVED, \
            x_test_id, x_company_id, xp_fault_array, x_fault_array_length)

#define EXPECT_EVT_CURRENT_STATUS(x_test_id, x_company_id, xp_fault_array, x_fault_array_length) \
    EXPECT_EVT_FAULT_STATUS_COMMON(HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED, \
            x_test_id, x_company_id, xp_fault_array, x_fault_array_length)

#define EXPECT_EVT_PERIOD_STATUS(x_divisor) \
    do { \
        m_event_expected = true; \
        m_expected_evt_type = HEALTH_CLIENT_EVT_TYPE_PERIOD_STATUS_RECEIVED; \
        m_expected_evt_data.period_status.fast_period_divisor = x_divisor; \
    } while(0)

#define EXPECT_EVT_ATTENTION_STATUS(x_attention) \
    do { \
        m_event_expected = true; \
        m_expected_evt_type = HEALTH_CLIENT_EVT_TYPE_ATTENTION_STATUS_RECEIVED; \
        m_expected_evt_data.attention_status.attention = x_attention; \
    } while(0)

#define EXPECT_EVT_TIMEOUT() \
    do { \
        m_event_expected = true; \
        m_expected_evt_type = HEALTH_CLIENT_EVT_TYPE_TIMEOUT; \
    } while(0)

#define EXPECT_EVT_CANCELLED() \
    do { \
        m_event_expected = true; \
        m_expected_evt_type = HEALTH_CLIENT_EVT_TYPE_CANCELLED; \
    } while(0)

static bool m_event_expected = false;
static health_client_evt_type_t m_expected_evt_type;
static union
{
    health_client_evt_fault_status_t     fault_status;
    health_client_evt_period_status_t    period_status;
    health_client_evt_attention_status_t attention_status;
} m_expected_evt_data;

static void event_handler(const health_client_t * p_client, const health_client_evt_t * p_event)
{
    TEST_ASSERT_TRUE(m_event_expected);
    m_event_expected = false;

    TEST_ASSERT_EQUAL(m_expected_evt_type, p_event->type);
    switch(p_event->type)
    {
        case HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED:
        case HEALTH_CLIENT_EVT_TYPE_FAULT_STATUS_RECEIVED:
            TEST_ASSERT_EQUAL(m_expected_evt_data.fault_status.test_id, p_event->data.fault_status.test_id);
            TEST_ASSERT_EQUAL(m_expected_evt_data.fault_status.company_id, p_event->data.fault_status.company_id);
            TEST_ASSERT_EQUAL(m_expected_evt_data.fault_status.fault_array_length, p_event->data.fault_status.fault_array_length);
            if(p_event->data.fault_status.fault_array_length > 0)
            {
                TEST_ASSERT_EQUAL_HEX8_ARRAY(m_expected_evt_data.fault_status.p_fault_array, p_event->data.fault_status.p_fault_array,
                        p_event->data.fault_status.fault_array_length);
            }
            else
            {
                TEST_ASSERT_NULL(m_expected_evt_data.fault_status.p_fault_array);
            }
            break;
        case HEALTH_CLIENT_EVT_TYPE_PERIOD_STATUS_RECEIVED:
            TEST_ASSERT_EQUAL(m_expected_evt_data.period_status.fast_period_divisor, p_event->data.period_status.fast_period_divisor);
            break;
        case HEALTH_CLIENT_EVT_TYPE_ATTENTION_STATUS_RECEIVED:
            TEST_ASSERT_EQUAL(m_expected_evt_data.attention_status.attention, p_event->data.attention_status.attention);
            break;
        case HEALTH_CLIENT_EVT_TYPE_TIMEOUT:
            break;
        case HEALTH_CLIENT_EVT_TYPE_CANCELLED:
            break;
        default:
            TEST_FAIL_MESSAGE("invalid event test detected in event_handler()");
    }
}

/********** Test cases **********/

void test_events(void)
{
    health_client_t client;

    /* Test what happens if no event handler is specified: */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, health_client_init(&client, TEST_ELEMENT_INDEX, NULL));

    /* Properly initialize the client: */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_init(&client, TEST_ELEMENT_INDEX, event_handler));

    /* The following opcodes generate events: */
    const uint16_t test_opcodes[] =
    {
        HEALTH_OPCODE_CURRENT_STATUS, HEALTH_OPCODE_FAULT_STATUS,
        HEALTH_OPCODE_PERIOD_STATUS, HEALTH_OPCODE_ATTENTION_STATUS,
    };

    TEST_ASSERT_EQUAL(ARRAY_SIZE(test_opcodes), m_num_opcodes);
    for(uint16_t i = 0; i < m_num_opcodes; ++i)
    {
        bool opcode_tested = false;
        for(uint16_t j = 0; j < m_num_opcodes; ++j)
        {
            access_message_rx_t message;

            if(mp_opcode_handlers[j].opcode.opcode == test_opcodes[i])
            {
                switch(test_opcodes[i])
                {
                    case HEALTH_OPCODE_FAULT_STATUS:
                    case HEALTH_OPCODE_CURRENT_STATUS:
                    {
                        uint8_t fault_array[3] = { 0x81, 0x12, 0x11 };
                        uint8_t data_buffer[sizeof(health_msg_fault_status_t) + sizeof(fault_array)];
                        health_msg_fault_status_t * p_test_data = (health_msg_fault_status_t *) data_buffer;

                        p_test_data->test_id = 4;
                        p_test_data->company_id = 0x4321;
                        memcpy(p_test_data->fault_array, fault_array, sizeof(fault_array));

                        message.opcode = mp_opcode_handlers[j].opcode;
                        message.p_data = data_buffer;
                        message.length = sizeof(data_buffer);

                        if(test_opcodes[i] == HEALTH_OPCODE_CURRENT_STATUS)
                        {
                            EXPECT_EVT_CURRENT_STATUS(4, 0x4321, fault_array, sizeof(fault_array));
                        }
                        else
                        {
                            EXPECT_EVT_FAULT_STATUS(4, 0x4321, fault_array, sizeof(fault_array));
                        }
                        mp_opcode_handlers[j].handler(TEST_MODEL_HANDLE, &message, &client);

                        /* Try without the fault array: */
                        p_test_data->test_id = 3;
                        p_test_data->company_id = 0x1234;

                        message.opcode = mp_opcode_handlers[j].opcode;
                        message.p_data = data_buffer;
                        message.length = sizeof(health_msg_fault_status_t);

                        if(test_opcodes[i] == HEALTH_OPCODE_CURRENT_STATUS)
                        {
                            EXPECT_EVT_CURRENT_STATUS(3, 0x1234, NULL, 0);
                        }
                        else
                        {
                            EXPECT_EVT_FAULT_STATUS(3, 0x1234, NULL, 0);
                        }
                        mp_opcode_handlers[j].handler(TEST_MODEL_HANDLE, &message, &client);
                        break;
                    }
                    case HEALTH_OPCODE_PERIOD_STATUS:
                    {
                        const health_msg_period_status_t test_data = { .fast_period_divisor = 12 };
                        message.opcode = mp_opcode_handlers[j].opcode;
                        message.p_data = (const uint8_t *) &test_data;
                        message.length = sizeof(health_msg_period_status_t);
                        EXPECT_EVT_PERIOD_STATUS(12);
                        mp_opcode_handlers[j].handler(TEST_MODEL_HANDLE, &message, &client);
                        break;
                    }
                    case HEALTH_OPCODE_ATTENTION_STATUS:
                    {
                        const health_msg_attention_status_t test_data = { .attention = 44 };
                        message.opcode = mp_opcode_handlers[j].opcode;
                        message.p_data = (const uint8_t *) &test_data;
                        message.length = sizeof(health_msg_attention_status_t);
                        EXPECT_EVT_ATTENTION_STATUS(44);
                        mp_opcode_handlers[j].handler(TEST_MODEL_HANDLE, &message, &client);
                        break;
                    }
                    default:
                        TEST_FAIL_MESSAGE("unrecognized opcode in test_events()");
                }

                opcode_tested = true;
                break;
            }
        }
        TEST_ASSERT_TRUE(opcode_tested);
    }
}

void test_messages(void)
{
    health_client_t client;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_init(&client, TEST_ELEMENT_INDEX, event_handler));

    /* Test the Attention Get message: */
    EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_ATTENTION_GET, HEALTH_OPCODE_ATTENTION_STATUS, NULL, 0, ACCESS_RELIABLE_TRANSFER_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_attention_get(&client));
    access_model_reliable_complete_transaction();

    /* Test the Attention Set messages: */
    {
        health_msg_attention_set_t expected_data = { .attention = 22 };
        EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_ATTENTION_SET, HEALTH_OPCODE_ATTENTION_STATUS, &expected_data, sizeof(expected_data),
                ACCESS_RELIABLE_TRANSFER_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_attention_set(&client, 22, true));
        access_model_reliable_complete_transaction();

        EXPECT_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_ATTENTION_SET_UNACKED, &expected_data, sizeof(expected_data));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_attention_set(&client, 22, false));
    }

    /* Test the Period Get message: */
    EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_PERIOD_GET, HEALTH_OPCODE_PERIOD_STATUS, NULL, 0, ACCESS_RELIABLE_TRANSFER_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_period_get(&client));
    access_model_reliable_complete_transaction();

    /* Test the Period Set messages: */
    {
        health_msg_period_set_t expected_data = { .fast_period_divisor = 41 };
        EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_PERIOD_SET, HEALTH_OPCODE_PERIOD_STATUS, &expected_data, sizeof(expected_data),
                ACCESS_RELIABLE_TRANSFER_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_period_set(&client, 41, true));
        access_model_reliable_complete_transaction();

        EXPECT_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_PERIOD_SET_UNACKED, &expected_data, sizeof(expected_data));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_period_set(&client, 41, false));
    }

    /* Test the Fault Get message: */
    {
        health_msg_fault_get_t expected_data = { .company_id = 0x1234 };
        EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_FAULT_GET, HEALTH_OPCODE_FAULT_STATUS, &expected_data, sizeof(expected_data),
                ACCESS_RELIABLE_TRANSFER_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_fault_get(&client, 0x1234));
        access_model_reliable_complete_transaction();
    }

    /* Test the Fault Clear messages: */
    {
        health_msg_fault_clear_t expected_data = { .company_id = 0x4321 };
        EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_FAULT_CLEAR, HEALTH_OPCODE_FAULT_STATUS, &expected_data, sizeof(expected_data),
                ACCESS_RELIABLE_TRANSFER_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_fault_clear(&client, 0x4321, true));
        access_model_reliable_complete_transaction();

        EXPECT_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_FAULT_CLEAR_UNACKED, &expected_data, sizeof(expected_data));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_fault_clear(&client, 0x4321, false));
    }

    /* Test the Fault Test messages: */
    {
        health_msg_fault_test_t expected_data = { .test_id = 10, .company_id = 0x1324 };
        EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_FAULT_TEST, HEALTH_OPCODE_FAULT_STATUS, &expected_data, sizeof(expected_data),
                ACCESS_RELIABLE_TRANSFER_SUCCESS);
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_fault_test(&client, 0x1324, 10, true));
        access_model_reliable_complete_transaction();

        EXPECT_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_FAULT_TEST_UNACKED, &expected_data, sizeof(expected_data));
        TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_fault_test(&client, 0x1324, 10, false));
    }
}

void test_timeout(void)
{
    health_client_t client;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_init(&client, TEST_ELEMENT_INDEX, event_handler));

    health_msg_fault_test_t expected_data = { .test_id = 10, .company_id = 0x1234 };
    EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_FAULT_TEST, HEALTH_OPCODE_FAULT_STATUS, &expected_data, sizeof(expected_data),
            ACCESS_RELIABLE_TRANSFER_TIMEOUT);
    EXPECT_EVT_TIMEOUT();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_fault_test(&client, 0x1234, 10, true));
    access_model_reliable_complete_transaction();
}

void test_cancelled(void)
{
    health_client_t client;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_init(&client, TEST_ELEMENT_INDEX, event_handler));

    health_msg_fault_test_t expected_data = { .test_id = 10, .company_id = 0x1234 };
    EXPECT_RELIABLE_TX(TEST_MODEL_HANDLE, HEALTH_OPCODE_FAULT_TEST, HEALTH_OPCODE_FAULT_STATUS, &expected_data, sizeof(expected_data),
            ACCESS_RELIABLE_TRANSFER_CANCELLED);
    EXPECT_EVT_CANCELLED();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_fault_test(&client, 0x1234, 10, true));
    access_model_reliable_complete_transaction();
}

void test_pending_msg_cancel(void)
{
    health_client_t client;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, health_client_init(&client, TEST_ELEMENT_INDEX, event_handler));

    access_model_reliable_cancel_ExpectAndReturn(m_model_handle, NRF_SUCCESS);
    health_client_pending_msg_cancel(&client);
}
