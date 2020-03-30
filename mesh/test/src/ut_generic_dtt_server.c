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

#include "generic_dtt_server.h"
#include "generic_dtt_messages.h"

#include <stdint.h>
#include <string.h>

#include <unity.h>
#include <cmock.h>

#include "model_common.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "access_mock.h"
#include "access_config_mock.h"


#define TEST_ELEMENT_INDEX    (3)
#define TEST_MODEL_ID         (GENERIC_DTT_SERVER_MODEL_ID)
#define TEST_MODEL_HANDLE     (5)


static generic_dtt_server_t  m_server;
static generic_dtt_server_callbacks_t m_server_cbs;
static generic_dtt_set_params_t m_expected_p_in;

static void dtt_server_set_cb(const generic_dtt_server_t * p_self,
                                const access_message_rx_meta_t * p_meta,
                                const generic_dtt_set_params_t * p_in,
                                generic_dtt_status_params_t * p_out);

static void dtt_server_get_cb(const generic_dtt_server_t * p_self,
                                const access_message_rx_meta_t * p_meta,
                                generic_dtt_status_params_t * p_out);

static generic_dtt_status_params_t m_test_status;

/***** Test related functions *****/
#define ACCESS_MESSAGE_RX(var, data, op, len) \
    do { \
        var.opcode.opcode = op; \
        var.opcode.company_id = ACCESS_COMPANY_ID_NONE; \
        var.p_data = (const uint8_t *) &data; \
        var.length = len; \
    } while(0)

#define ACCESS_MESSAGE_RX_EMPTY(var, op) \
    do { \
        var.opcode.opcode = op; \
        var.opcode.company_id = ACCESS_COMPANY_ID_NONE; \
        var.p_data = NULL; \
        var.length = 0; \
    } while(0)

static void * mp_model_args;
static access_model_handle_t m_model_handle;
static access_model_add_params_t m_model_init_params;
static const access_opcode_handler_t * mp_opcode_handlers;
static uint16_t m_num_opcodes;

static uint32_t access_model_add_mock(const access_model_add_params_t * p_init_params,
        access_model_handle_t * p_model_handle, int count)
{
    m_model_init_params = *p_init_params;
    *p_model_handle = TEST_MODEL_HANDLE;

    m_model_handle = *p_model_handle;
    mp_opcode_handlers = p_init_params->p_opcode_handlers;
    m_num_opcodes = p_init_params->opcode_count;
    mp_model_args = p_init_params->p_args;

    TEST_ASSERT_EQUAL(TEST_MODEL_ID, p_init_params->model_id.model_id);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_init_params->model_id.company_id);
    TEST_ASSERT_EQUAL(TEST_ELEMENT_INDEX, p_init_params->element_index);

    return NRF_SUCCESS;
}

#define EXPECT_DTT_STATUS_PUBLISH_TX(x_handle, x_expected_params) \
    do { \
        TEST_ASSERT_FALSE(m_publish_expected); \
        m_publish_expected = true; \
        access_model_publish_StubWithCallback(access_model_publish_mock);\
        m_publish_expected_handle = x_handle; \
        m_publish_expected_opcode = GENERIC_DTT_OPCODE_STATUS; \
        static generic_dtt_status_msg_pkt_t expected_data; \
        expected_data.transition_time = model_transition_time_encode(x_expected_params.transition_time_ms);\
        mp_publish_expected_data = (void *) &expected_data; \
        m_publish_expected_data_length = sizeof(generic_dtt_status_msg_pkt_t); \
    } while(0)

static bool m_publish_expected;
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
    TEST_ASSERT_EQUAL(NRF_MESH_TRANSMIC_SIZE_SMALL, p_message->transmic_size);

    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t *) mp_publish_expected_data, p_message->p_buffer, p_message->length);

    return NRF_SUCCESS;
}

#define EXPECT_REPLY(x_handle, x_opcode, x_expected_data, x_expected_data_length) \
    do { \
        TEST_ASSERT_FALSE(m_reply_expected); \
        m_reply_expected = true; \
        m_reply_expected_handle = x_handle; \
        m_reply_expected_opcode = x_opcode; \
        static generic_dtt_status_msg_pkt_t expected_data; \
        expected_data.transition_time = model_transition_time_encode(x_expected_data.transition_time_ms);\
        mp_reply_expected_data = (void *) &expected_data; \
        m_reply_expected_data_length = x_expected_data_length; \
        access_model_reply_StubWithCallback(access_model_reply_mock); \
    } while(0)

static bool m_reply_expected;
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

/***** Helper functions *****/
static void helper_call_opcode_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
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

static void helper_expect_for_model_init_success(void)
{
    access_model_add_StubWithCallback(access_model_add_mock);
    access_model_subscription_list_alloc_ExpectAnyArgsAndReturn(NRF_SUCCESS);
}

static void helper_init_model_context_and_expectations(void)
{
    m_server_cbs.dtt_cbs.set_cb = dtt_server_set_cb;
    m_server_cbs.dtt_cbs.get_cb = dtt_server_get_cb;
    m_server.settings.p_callbacks = &m_server_cbs;
    m_server.settings.force_segmented = false;
    m_server.settings.transmic_size = NRF_MESH_TRANSMIC_SIZE_SMALL;
    helper_expect_for_model_init_success();
}

/******** Model Callbacks ********/
static void dtt_server_set_cb(const generic_dtt_server_t * p_self,
                         const access_message_rx_meta_t * p_meta,
                         const generic_dtt_set_params_t * p_in,
                         generic_dtt_status_params_t * p_out)
{
    TEST_ASSERT_NOT_NULL(p_meta);
    TEST_ASSERT_EQUAL(m_expected_p_in.transition_time_ms, p_in->transition_time_ms);
    if (p_out != NULL)
    {
        *p_out = m_test_status;
    }
}

static void dtt_server_get_cb(const generic_dtt_server_t * p_self,
                         const access_message_rx_meta_t * p_meta,
                         generic_dtt_status_params_t * p_out)
{
    if (m_publish_expected)
    {
        TEST_ASSERT_EQUAL(NULL, p_meta);
    }
    else
    {
        TEST_ASSERT_NOT_NULL(p_meta);
    }

    *p_out = m_test_status;
}

/******** Setup and Tear Down ********/
void setUp(void)
{
    access_mock_Init();
    timer_scheduler_mock_Init();
    timer_mock_Init();
}

void tearDown(void)
{
    access_mock_Verify();
    access_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    timer_mock_Verify();
}

/******** Tests ********/
void test_generic_dtt_server_init(void)
{
    /* Null checks */
    memset(&m_server, 0, sizeof(generic_dtt_server_t));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_dtt_server_init(NULL, TEST_ELEMENT_INDEX));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));
    m_server.settings.p_callbacks = &m_server_cbs;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));
    m_server_cbs.dtt_cbs.set_cb = dtt_server_set_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));
    m_server_cbs.dtt_cbs.get_cb = dtt_server_get_cb;

    /* All mandatory API inputs are correct */
    helper_expect_for_model_init_success();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));
}

void test_generic_dtt_server_status_publish(void)
{
    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));

    /* Null checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_dtt_server_status_publish(NULL, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_dtt_server_status_publish(&m_server, NULL));

    generic_dtt_status_params_t status;

    /* Invalid param check - There are no invalid parameters*/
    /* Valid param check - full length msg */
    status.transition_time_ms = 100;
    EXPECT_DTT_STATUS_PUBLISH_TX(TEST_MODEL_HANDLE, status);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_dtt_server_status_publish(&m_server, &status));

    /* Valid param check - no optional params */
    status.transition_time_ms = 0xFFFFFFFF;
    EXPECT_DTT_STATUS_PUBLISH_TX(TEST_MODEL_HANDLE, status);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_dtt_server_status_publish(&m_server, &status));
}

void test_periodic_publish_cb(void)
{
    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));

    /* Trigger periodic publish cb */
    m_test_status.transition_time_ms = 500;
    EXPECT_DTT_STATUS_PUBLISH_TX(m_server.model_handle, m_test_status);
    m_model_init_params.publish_timeout_cb(TEST_MODEL_HANDLE, &m_server);
}

void test_handle_set(void)
{
    uint32_t dtt_value = 0;
    access_message_rx_t request_msg;
    generic_dtt_set_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));

    /* SET */
    /* case 1: There is only one parameter */
    dtt_value = 100;
    pkt.transition_time = model_transition_time_encode(dtt_value);
    ACCESS_MESSAGE_RX(request_msg, pkt, GENERIC_DTT_OPCODE_SET, sizeof(generic_dtt_set_msg_pkt_t));
    m_expected_p_in.transition_time_ms = dtt_value;

    m_test_status.transition_time_ms = dtt_value;
    EXPECT_REPLY(m_server.model_handle, GENERIC_DTT_OPCODE_STATUS, m_test_status, sizeof(generic_dtt_status_msg_pkt_t));
    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);

    /* case 2: Invalid length message */
    dtt_value = 200;
    pkt.transition_time = model_transition_time_encode(dtt_value);
    ACCESS_MESSAGE_RX(request_msg, pkt, GENERIC_DTT_OPCODE_SET, sizeof(generic_dtt_set_msg_pkt_t) + 1);
    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);

    /* case 3: Invalid transition time */
    pkt.transition_time = 0x3F;
    ACCESS_MESSAGE_RX(request_msg, pkt, GENERIC_DTT_OPCODE_SET, sizeof(generic_dtt_set_msg_pkt_t));
    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);

    /* SET - Unacknowledged*/
    /* case 1: There is only one parameter */
    dtt_value = 300;
    pkt.transition_time = model_transition_time_encode(dtt_value);
    ACCESS_MESSAGE_RX(request_msg, pkt, GENERIC_DTT_OPCODE_SET_UNACKNOWLEDGED, sizeof(generic_dtt_set_msg_pkt_t));
    m_expected_p_in.transition_time_ms = dtt_value;

    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);

    /* case 2: Invalid length message */
    dtt_value = 400;
    pkt.transition_time = model_transition_time_encode(dtt_value);
    ACCESS_MESSAGE_RX(request_msg, pkt, GENERIC_DTT_OPCODE_SET_UNACKNOWLEDGED, sizeof(generic_dtt_set_msg_pkt_t) + 1);
    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);

    /* case 3: Invalid transition time */
    pkt.transition_time = 0x3F;
    ACCESS_MESSAGE_RX(request_msg, pkt, GENERIC_DTT_OPCODE_SET_UNACKNOWLEDGED, sizeof(generic_dtt_set_msg_pkt_t));
    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);
}

void test_handle_get(void)
{
    uint32_t dtt_value = 500;
    access_message_rx_t request_msg;
    uint8_t * p_pkt = NULL;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_dtt_server_init(&m_server, TEST_ELEMENT_INDEX));

    /* GET */
    /* case 1: Status is sent */
    ACCESS_MESSAGE_RX(request_msg, p_pkt, GENERIC_DTT_OPCODE_GET, 0);

    m_test_status.transition_time_ms = dtt_value;
    EXPECT_REPLY(m_server.model_handle, GENERIC_DTT_OPCODE_STATUS, m_test_status, sizeof(generic_dtt_status_msg_pkt_t));
    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);

    /* case 2: Invalid length */
    dtt_value = 1000;
    m_test_status.transition_time_ms = dtt_value;
    ACCESS_MESSAGE_RX(request_msg, p_pkt, GENERIC_DTT_OPCODE_GET, 1);
    helper_call_opcode_handler(m_server.model_handle, &request_msg, &m_server);
}