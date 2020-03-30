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

#include "generic_ponoff_client.h"
#include "generic_ponoff_messages.h"

#include <stdint.h>
#include <string.h>

#include <unity.h>
#include <cmock.h>

#include "model_common.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "access_mock.h"
#include "access_config_mock.h"
#include "access_reliable_mock.h"
#include "generic_onoff_client_mock.h"
#include "nrf_mesh_mock.h"


#define TEST_ELEMENT_INDEX          (7)
#define TEST_MODEL_ID               (GENERIC_PONOFF_CLIENT_MODEL_ID)
#define TEST_MODEL_HANDLE           (10)
#define TEST_MODEL_TIMEOUT          (10000)
#define TEST_ACCESS_TOKEN           (121)
#define TEST_MODEL_UNACK_REPEATS    (3)

static generic_ponoff_client_t  m_client;
static generic_ponoff_client_callbacks_t m_client_cbs;
static generic_ponoff_status_params_t m_expected_status;

static void ponoff_client_status_cb(const generic_ponoff_client_t * p_self,
                                const access_message_rx_meta_t * p_meta,
                                const generic_ponoff_status_params_t * p_in);


static void ponoff_client_ack_transaction_cb(access_model_handle_t model_handle, void * p_args,
                                            access_reliable_status_t status);


static void ponoff_client_publish_timeout_cb(access_model_handle_t handle, void * p_args);



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

#define EXPECT_ACCESS_MESSAGE_PUBLISH(x_handle, x_opcode, xp_expected_data, x_expected_data_length, x_repeats) \
    do { \
        TEST_ASSERT_FALSE(m_publish_expected); \
        m_publish_expected = true; \
        nrf_mesh_unique_token_get_ExpectAndReturn(TEST_ACCESS_TOKEN); \
        access_model_publish_StubWithCallback(access_model_publish_mock); \
        m_publish_repeats = x_repeats; \
        m_publish_expected_handle = x_handle; \
        m_publish_expected_opcode = x_opcode; \
        mp_publish_expected_data = (void *) xp_expected_data; \
        m_publish_expected_data_length = x_expected_data_length; \
    } while(0)

#define EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(x_handle, x_opcode, xp_expected_data, x_expected_data_length) \
    do { \
        TEST_ASSERT_FALSE(m_publish_expected); \
        m_publish_expected = true; \
        access_reliable_model_is_free_ExpectAndReturn(m_client.model_handle, true);\
        nrf_mesh_unique_token_get_ExpectAndReturn(TEST_ACCESS_TOKEN); \
        access_model_reliable_publish_StubWithCallback(access_model_reliable_publish_mock);\
        m_publish_expected_handle = x_handle; \
        m_publish_expected_opcode = x_opcode; \
        mp_publish_expected_data = (void *) xp_expected_data; \
        m_publish_expected_data_length = x_expected_data_length; \
    } while(0)

static bool m_publish_expected;
static access_model_handle_t m_publish_expected_handle;
static uint16_t m_publish_expected_opcode;
static void * mp_publish_expected_data;
static uint16_t m_publish_expected_data_length;
static uint8_t m_publish_repeats;

static uint32_t access_model_publish_mock(access_model_handle_t handle, const access_message_tx_t * p_message, int count)
{
    TEST_ASSERT_NOT_NULL(p_message);

    TEST_ASSERT_TRUE(m_publish_expected);
    if (m_publish_repeats == 0)
    {
        m_publish_expected = false;
    }
    else
    {
        m_publish_repeats--;
    }
    TEST_ASSERT_EQUAL(m_publish_expected_handle, handle);
    TEST_ASSERT_EQUAL(m_publish_expected_opcode, p_message->opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_message->opcode.company_id);
    TEST_ASSERT_EQUAL(m_publish_expected_data_length, p_message->length);
    TEST_ASSERT_EQUAL(false, p_message->force_segmented);
    TEST_ASSERT_EQUAL(NRF_MESH_TRANSMIC_SIZE_SMALL, p_message->transmic_size);
    TEST_ASSERT_EQUAL(TEST_ACCESS_TOKEN, p_message->access_token);
    if (mp_publish_expected_data != NULL)
    {
        TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t *) mp_publish_expected_data, p_message->p_buffer, p_message->length);
    }

    return NRF_SUCCESS;
}

static uint32_t access_model_reliable_publish_mock(const access_reliable_t * p_reliable, int count)
{
    TEST_ASSERT_NOT_NULL(p_reliable);

    TEST_ASSERT_TRUE(m_publish_expected);
    m_publish_expected = false;
    TEST_ASSERT_EQUAL(m_publish_expected_handle, p_reliable->model_handle);
    TEST_ASSERT_EQUAL(GENERIC_PONOFF_OPCODE_STATUS, p_reliable->reply_opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reliable->reply_opcode.company_id);
    TEST_ASSERT_EQUAL(TEST_MODEL_TIMEOUT, p_reliable->timeout);
    TEST_ASSERT_EQUAL(ponoff_client_ack_transaction_cb, p_reliable->status_cb);

    TEST_ASSERT_EQUAL(m_publish_expected_opcode, p_reliable->message.opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reliable->message.opcode.company_id);
    TEST_ASSERT_EQUAL(m_publish_expected_data_length, p_reliable->message.length);
    TEST_ASSERT_EQUAL(false, p_reliable->message.force_segmented);
    TEST_ASSERT_EQUAL(NRF_MESH_TRANSMIC_SIZE_SMALL, p_reliable->message.transmic_size);
    TEST_ASSERT_EQUAL(TEST_ACCESS_TOKEN, p_reliable->message.access_token);
    if (mp_publish_expected_data != NULL)
    {
        TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t *) mp_publish_expected_data, p_reliable->message.p_buffer, p_reliable->message.length);
    }

    return NRF_SUCCESS;
}

/***** Helper functions *****/
static void helper_call_opcode_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    bool handler_found = false;
    for (uint16_t i = 0; i < m_num_opcodes; ++i)
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
    m_client_cbs.ponoff_status_cb = ponoff_client_status_cb;
    m_client_cbs.ack_transaction_status_cb = ponoff_client_ack_transaction_cb;
    m_client_cbs.periodic_publish_cb = ponoff_client_publish_timeout_cb;
    m_client.settings.p_callbacks = &m_client_cbs;
    m_client.settings.force_segmented = false;
    m_client.settings.transmic_size = NRF_MESH_TRANSMIC_SIZE_SMALL;
    m_client.settings.timeout = TEST_MODEL_TIMEOUT;
    helper_expect_for_model_init_success();
}

/******** Model Callbacks ********/
static void ponoff_client_status_cb(const generic_ponoff_client_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const generic_ponoff_status_params_t * p_in)
{
    printf("ponoff_client_status_cb\n");
    TEST_ASSERT_NOT_NULL(p_meta);

    TEST_ASSERT_EQUAL(m_expected_status.on_powerup, p_in->on_powerup);
}

/* Dummy: unsued */
static void ponoff_client_ack_transaction_cb(access_model_handle_t model_handle, void * p_args,
                                            access_reliable_status_t status)
{
    return;
}

/* Dummy: unsued */
static void ponoff_client_publish_timeout_cb(access_model_handle_t handle, void * p_args)
{
    return;
}


/******** Setup and Tear Down ********/
void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    access_reliable_mock_Init();
    generic_onoff_client_mock_Init();
    nrf_mesh_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
}

void tearDown(void)
{
    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    access_reliable_mock_Verify();
    access_reliable_mock_Destroy();
    generic_onoff_client_mock_Verify();
    generic_onoff_client_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
}
/******** Tests ********/
void test_generic_ponoff_client_init(void)
{
    /* Null checks */
    memset(&m_client, 0, sizeof(generic_ponoff_client_t));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_init(NULL, TEST_ELEMENT_INDEX));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client.settings.p_callbacks = &m_client_cbs;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client_cbs.ack_transaction_status_cb = ponoff_client_ack_transaction_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client_cbs.periodic_publish_cb = ponoff_client_publish_timeout_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client_cbs.ponoff_status_cb = ponoff_client_status_cb;

    /* All mandatory API inputs are correct */
    helper_expect_for_model_init_success();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));
}


void test_generic_ponoff_client_set(void)
{
    generic_ponoff_set_params_t set_params = {0};
    generic_ponoff_set_msg_pkt_t pkt;
    uint16_t opcode;
    uint16_t len;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_set(NULL, &set_params));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_set(&m_client, NULL));

    /* case: Busy check fails */
    access_reliable_model_is_free_ExpectAndReturn(m_client.model_handle, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, generic_ponoff_client_set(&m_client, &set_params));

    /* case: Invalid on_powerup value */
    set_params.on_powerup = GENERIC_ON_POWERUP_MAX + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, generic_ponoff_client_set(&m_client, &set_params));

    /* case: Valid params, there are no optional params for this API */
    set_params.on_powerup = 1;
    pkt.on_powerup = set_params.on_powerup;

    opcode = GENERIC_PONOFF_OPCODE_SET;
    len = sizeof(pkt);
    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, opcode, &pkt, len);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_set(&m_client, &set_params));
}

void test_generic_ponoff_client_set_unack(void)
{
    generic_ponoff_set_params_t set_params = {0};
    generic_ponoff_set_msg_pkt_t pkt;
    uint16_t opcode;
    uint16_t len;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_set_unack(NULL, &set_params, TEST_MODEL_UNACK_REPEATS));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_set_unack(&m_client, NULL, TEST_MODEL_UNACK_REPEATS));

    /* case: Invalid on_powerup value */
    set_params.on_powerup = GENERIC_ON_POWERUP_MAX + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, generic_ponoff_client_set_unack(&m_client, &set_params, TEST_MODEL_UNACK_REPEATS));

    /* case: Valid params, there are no optional params for this API */
    set_params.on_powerup = 1;
    pkt.on_powerup = set_params.on_powerup;

    opcode = GENERIC_PONOFF_OPCODE_SET_UNACKNOWLEDGED;
    len = sizeof(pkt);
    EXPECT_ACCESS_MESSAGE_PUBLISH(m_client.model_handle, opcode, &pkt, len, TEST_MODEL_UNACK_REPEATS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_set_unack(&m_client, &set_params, TEST_MODEL_UNACK_REPEATS));
}

void test_generic_ponoff_client_get(void)
{
    uint16_t opcode;
    uint16_t len;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, generic_ponoff_client_get(NULL));

    /* case: Message does not have any params */
    opcode = GENERIC_PONOFF_OPCODE_GET;
    len = 0;
    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, opcode, NULL, len);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_get(&m_client));
}

void test_status_handle(void)
{
    access_message_rx_t request_msg;
    generic_ponoff_status_msg_pkt_t pkt;
    uint16_t opcode;
    uint16_t len;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, generic_ponoff_client_init(&m_client, TEST_ELEMENT_INDEX));

    opcode = GENERIC_PONOFF_OPCODE_STATUS;

    /* case: valid message is received */
    pkt.on_powerup = 1;
    len = sizeof(generic_ponoff_status_msg_pkt_t);

    m_expected_status.on_powerup = pkt.on_powerup;
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, len);
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);

    /* case: Message with invalid length was received, this should not trigger any TEST() */
    m_expected_status.on_powerup = 0xCA;
    len = sizeof(generic_ponoff_status_msg_pkt_t) + 1;
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, len);
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);
}
