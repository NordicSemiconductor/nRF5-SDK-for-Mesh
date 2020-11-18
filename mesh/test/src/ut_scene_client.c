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

#include "scene_client.h"
#include "scene_messages.h"

#include <stdint.h>
#include <string.h>

#include <unity.h>
#include <cmock.h>

#include "model_common.h"
#include "access_mock.h"
#include "access_config_mock.h"
#include "access_reliable_mock.h"
#include "nrf_mesh_mock.h"


#define TEST_ELEMENT_INDEX          (7)
#define TEST_MODEL_ID               (SCENE_CLIENT_MODEL_ID)
#define TEST_MODEL_HANDLE           (10)
#define TEST_MODEL_TIMEOUT          (10000)
#define TEST_ACCESS_TOKEN           (121)
#define TEST_MODEL_UNACK_REPEATS    (3)

static scene_client_t m_client;
static scene_client_callbacks_t m_client_cbs;
static scene_status_params_t m_expected_status;
static scene_register_status_params_t m_expected_register_status;

static void scene_client_status_cb(const scene_client_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const scene_status_params_t * p_in);

static void scene_client_register_status_cb(const scene_client_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const scene_register_status_params_t * p_in);

static void scene_client_ack_transaction_cb(access_model_handle_t model_handle, void * p_args,
                                            access_reliable_status_t status);

static void scene_client_publish_timeout_cb(access_model_handle_t handle, void * p_args);

#define ACCESS_MESSAGE_RX(var, data, op, len) \
    do { \
        var.opcode.opcode = op; \
        var.opcode.company_id = ACCESS_COMPANY_ID_NONE; \
        var.p_data = (const uint8_t *) &data; \
        var.length = len; \
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

static bool m_publish_expected;
static access_model_handle_t m_publish_expected_handle;
static uint16_t m_publish_expected_reply_opcode;
static uint16_t m_publish_expected_opcode;
static void * mp_publish_expected_data;
static uint16_t m_publish_expected_data_length;
static uint8_t m_publish_repeats;

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

#define EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(x_handle, x_opcode, xp_expected_data, x_expected_data_length, x_reply_opcode) \
    do { \
        TEST_ASSERT_FALSE(m_publish_expected); \
        m_publish_expected = true; \
        access_reliable_model_is_free_ExpectAndReturn(m_client.model_handle, true);\
        nrf_mesh_unique_token_get_ExpectAndReturn(TEST_ACCESS_TOKEN); \
        access_model_reliable_publish_StubWithCallback(access_model_reliable_publish_mock);\
        m_publish_expected_handle = x_handle; \
        m_publish_expected_reply_opcode = x_reply_opcode; \
        m_publish_expected_opcode = x_opcode; \
        mp_publish_expected_data = (void *) xp_expected_data; \
        m_publish_expected_data_length = x_expected_data_length; \
    } while(0)

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
    TEST_ASSERT_EQUAL(m_publish_expected_reply_opcode, p_reliable->reply_opcode.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reliable->reply_opcode.company_id);
    TEST_ASSERT_EQUAL(TEST_MODEL_TIMEOUT, p_reliable->timeout);
    TEST_ASSERT_EQUAL(scene_client_ack_transaction_cb, p_reliable->status_cb);

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
static void helper_expect_for_model_init_success(void)
{
    access_model_add_StubWithCallback(access_model_add_mock);
    access_model_subscription_list_alloc_ExpectAnyArgsAndReturn(NRF_SUCCESS);
}

static void helper_init_model_context_and_expectations(void)
{
    m_client_cbs.scene_status_cb = scene_client_status_cb;
    m_client_cbs.scene_register_status_cb = scene_client_register_status_cb;
    m_client_cbs.ack_transaction_status_cb = scene_client_ack_transaction_cb;
    m_client_cbs.periodic_publish_cb = scene_client_publish_timeout_cb;
    m_client.settings.p_callbacks = &m_client_cbs;
    m_client.settings.force_segmented = false;
    m_client.settings.transmic_size = NRF_MESH_TRANSMIC_SIZE_SMALL;
    m_client.settings.timeout = TEST_MODEL_TIMEOUT;
    helper_expect_for_model_init_success();
}

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

/******** Model Callbacks ********/
static void scene_client_status_cb(const scene_client_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   const scene_status_params_t * p_in)
{
    printf("scene_client_status_cb\n");
    TEST_ASSERT_NOT_NULL(p_meta);

    TEST_ASSERT_EQUAL(m_expected_status.status_code, p_in->status_code);
    TEST_ASSERT_EQUAL(m_expected_status.current_scene, p_in->current_scene);
    TEST_ASSERT_EQUAL(m_expected_status.target_scene, p_in->target_scene);
    TEST_ASSERT_EQUAL(m_expected_status.remaining_time_ms, p_in->remaining_time_ms);
}

static void scene_client_register_status_cb(const scene_client_t * p_self,
                                            const access_message_rx_meta_t * p_meta,
                                            const scene_register_status_params_t * p_in)
{
    printf("scene_client_register_status_cb\n");
    TEST_ASSERT_NOT_NULL(p_meta);

    TEST_ASSERT_EQUAL(m_expected_register_status.status_code, p_in->status_code);
    TEST_ASSERT_EQUAL(m_expected_register_status.current_scene, p_in->current_scene);
    TEST_ASSERT_EQUAL_HEX8_ARRAY((const uint8_t *) m_expected_register_status.scenes, p_in->scenes, SCENE_REGISTER_ARRAY_SIZE);
}

/* Dummy: unsued */
static void scene_client_ack_transaction_cb(access_model_handle_t model_handle, void * p_args,
                                            access_reliable_status_t status)
{
    return;
}

/* Dummy: unsued */
static void scene_client_publish_timeout_cb(access_model_handle_t handle, void * p_args)
{
    return;
}


/******** Setup and Tear Down ********/
void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    access_reliable_mock_Init();
    nrf_mesh_mock_Init();
}

void tearDown(void)
{
    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    access_reliable_mock_Verify();
    access_reliable_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
}

/******** Tests ********/
void test_scene_client_init(void)
{
    /* Null checks */
    memset(&m_client, 0, sizeof(scene_client_t));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_init(NULL, TEST_ELEMENT_INDEX));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client.settings.p_callbacks = &m_client_cbs;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client_cbs.scene_status_cb = scene_client_status_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client_cbs.scene_register_status_cb = scene_client_register_status_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client_cbs.periodic_publish_cb = scene_client_publish_timeout_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_init(&m_client, TEST_ELEMENT_INDEX));
    m_client_cbs.ack_transaction_status_cb = scene_client_ack_transaction_cb;

    /* Access model add/subscritpition failures. */
    access_model_add_ExpectAnyArgsAndReturn(NRF_ERROR_INVALID_LENGTH);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, scene_client_init(&m_client, TEST_ELEMENT_INDEX));
    access_model_add_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_subscription_list_alloc_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* All mandatory API inputs are correct */
    helper_expect_for_model_init_success();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));
}


void test_scene_client_store(void)
{
    scene_store_params_t set_params = {0};
    scene_store_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_store(NULL, &set_params));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_store(&m_client, NULL));

    /* case: Invalid scene_number value */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_store(&m_client, &set_params));

    /* Valid params, there are no optional params for this API */
    set_params.scene_number = 1;
    pkt.scene_number = set_params.scene_number;

    /* case: Busy check fails */
    access_reliable_model_is_free_ExpectAndReturn(m_client.model_handle, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, scene_client_store(&m_client, &set_params));

    /* case: Message ok */
    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, SCENE_OPCODE_STORE, &pkt, sizeof(pkt), SCENE_OPCODE_REGISTER_STATUS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_store(&m_client, &set_params));
}

void test_scene_client_store_unack(void)
{
    scene_store_params_t set_params = {0};
    scene_store_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_store_unack(NULL, &set_params, TEST_MODEL_UNACK_REPEATS));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_store_unack(&m_client, NULL, TEST_MODEL_UNACK_REPEATS));

    /* case: Invalid scene_number value */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_store_unack(&m_client, &set_params, TEST_MODEL_UNACK_REPEATS));

    /* Valid params, there are no optional params for this API */
    set_params.scene_number = 1;
    pkt.scene_number = set_params.scene_number;

    /* case: Message ok */
    EXPECT_ACCESS_MESSAGE_PUBLISH(m_client.model_handle, SCENE_OPCODE_STORE_UNACKNOWLEDGED, &pkt, sizeof(pkt), TEST_MODEL_UNACK_REPEATS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_store_unack(&m_client, &set_params, TEST_MODEL_UNACK_REPEATS));
}

void test_scene_client_delete(void)
{
    scene_delete_params_t set_params = {0};
    scene_delete_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_delete(NULL, &set_params));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_delete(&m_client, NULL));

    /* case: Invalid scene_number value */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_delete(&m_client, &set_params));

    /* Valid params, there are no optional params for this API */
    set_params.scene_number = 1;
    pkt.scene_number = set_params.scene_number;

    /* case: Busy check fails */
    access_reliable_model_is_free_ExpectAndReturn(m_client.model_handle, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, scene_client_delete(&m_client, &set_params));

    /* case: Message ok */
    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, SCENE_OPCODE_DELETE, &pkt, sizeof(pkt), SCENE_OPCODE_REGISTER_STATUS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_delete(&m_client, &set_params));
}

void test_scene_client_delete_unack(void)
{
    scene_delete_params_t set_params = {0};
    scene_delete_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_delete_unack(NULL, &set_params, TEST_MODEL_UNACK_REPEATS));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_delete_unack(&m_client, NULL, TEST_MODEL_UNACK_REPEATS));

    /* case: Invalid scene_number value */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_delete_unack(&m_client, &set_params, TEST_MODEL_UNACK_REPEATS));

    /* Valid params, there are no optional params for this API */
    set_params.scene_number = 1;
    pkt.scene_number = set_params.scene_number;

    /* case: Message ok */
    EXPECT_ACCESS_MESSAGE_PUBLISH(m_client.model_handle, SCENE_OPCODE_DELETE_UNACKNOWLEDGED, &pkt, sizeof(pkt), TEST_MODEL_UNACK_REPEATS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_delete_unack(&m_client, &set_params, TEST_MODEL_UNACK_REPEATS));
}

void test_scene_client_recall(void)
{
    scene_recall_params_t set_params = {0};
    model_transition_t transition_params = {0};
    scene_recall_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_recall(NULL, &set_params, &transition_params));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_recall(&m_client, NULL, &transition_params));

    /* case: Invalid scene_number value */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_recall(&m_client, &set_params, &transition_params));

    /* set valid params */
    set_params.scene_number = 1;

    /* case: Busy check fails */
    access_reliable_model_is_free_ExpectAndReturn(m_client.model_handle, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, scene_client_recall(&m_client, &set_params, &transition_params));

    /* case: Invalid transition time */
    transition_params.transition_time_ms = TRANSITION_TIME_MAX_MS + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_recall(&m_client, &set_params, &transition_params));
    transition_params.transition_time_ms = TRANSITION_TIME_MAX_MS;
    transition_params.delay_ms = DELAY_TIME_MAX_MS + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_recall(&m_client, &set_params, &transition_params));

    /* case: Message including optional params */
    set_params.scene_number = 1;
    set_params.tid = 0;
    transition_params.transition_time_ms = 1000;
    transition_params.delay_ms = 0;

    pkt.scene_number = set_params.scene_number;
    pkt.tid = set_params.tid;
    pkt.transition_time = model_transition_time_encode(transition_params.transition_time_ms);
    pkt.delay = model_delay_encode(transition_params.delay_ms);

    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, SCENE_OPCODE_RECALL, &pkt, SCENE_RECALL_MAXLEN, SCENE_OPCODE_STATUS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_recall(&m_client, &set_params, &transition_params));

    /* case: Message excluding optional params */
    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, SCENE_OPCODE_RECALL, &pkt, SCENE_RECALL_MINLEN, SCENE_OPCODE_STATUS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_recall(&m_client, &set_params, NULL));
}

void test_scene_client_recall_unack(void)
{
    scene_recall_params_t set_params = {0};
    model_transition_t transition_params = {0};
    scene_recall_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_recall_unack(NULL, &set_params, &transition_params, TEST_MODEL_UNACK_REPEATS));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_recall_unack(&m_client, NULL, &transition_params, TEST_MODEL_UNACK_REPEATS));

    /* case: Invalid scene_number value */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_recall_unack(&m_client, &set_params, &transition_params, TEST_MODEL_UNACK_REPEATS));

    /* case: Invalid transition time */
    set_params.scene_number = 1;
    transition_params.transition_time_ms = TRANSITION_TIME_MAX_MS + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_recall_unack(&m_client, &set_params, &transition_params, TEST_MODEL_UNACK_REPEATS));
    transition_params.transition_time_ms = TRANSITION_TIME_MAX_MS;
    transition_params.delay_ms = DELAY_TIME_MAX_MS + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_client_recall_unack(&m_client, &set_params, &transition_params, TEST_MODEL_UNACK_REPEATS));

    /* case: Message including optional params */
    set_params.scene_number = 1;
    set_params.tid = 0;
    transition_params.transition_time_ms = 1000;
    transition_params.delay_ms = 0;

    pkt.scene_number = set_params.scene_number;
    pkt.tid = set_params.tid;
    pkt.transition_time = model_transition_time_encode(transition_params.transition_time_ms);
    pkt.delay = model_delay_encode(transition_params.delay_ms);

    EXPECT_ACCESS_MESSAGE_PUBLISH(m_client.model_handle, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, &pkt, SCENE_RECALL_MAXLEN, TEST_MODEL_UNACK_REPEATS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_recall_unack(&m_client, &set_params, &transition_params, TEST_MODEL_UNACK_REPEATS));

    /* case: Message excluding optional params */
    EXPECT_ACCESS_MESSAGE_PUBLISH(m_client.model_handle, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, &pkt, SCENE_RECALL_MINLEN, TEST_MODEL_UNACK_REPEATS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_recall_unack(&m_client, &set_params, NULL, TEST_MODEL_UNACK_REPEATS));
}

void test_scene_client_get(void)
{
    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_get(NULL));

    /* case: Message ok */
    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, SCENE_OPCODE_GET, NULL, 0, SCENE_OPCODE_STATUS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_get(&m_client));
}

void test_scene_client_register_get(void)
{
    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_client_register_get(NULL));

    /* case: Message ok */
    EXPECT_ACCESS_MESSAGE_RELIABLE_PUBLISH(m_client.model_handle, SCENE_OPCODE_REGISTER_GET, NULL, 0, SCENE_OPCODE_REGISTER_STATUS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_register_get(&m_client));
}

void test_status_handle(void)
{
    access_message_rx_t request_msg;
    scene_status_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    uint16_t opcode = SCENE_OPCODE_STATUS;

    /* case: Optional params not included */
    pkt.status_code = SCENE_STATUS_SUCCESS;
    pkt.current_scene = SCENE_NUMBER_NO_SCENE;

    m_expected_status.status_code = pkt.status_code;
    m_expected_status.current_scene = pkt.current_scene;
    m_expected_status.target_scene = SCENE_NUMBER_NO_SCENE;
    m_expected_status.remaining_time_ms = 0;
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, SCENE_STATUS_MINLEN);
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);

    /* case: Optional params included, transition to target*/
    pkt.status_code = SCENE_STATUS_SUCCESS;
    pkt.current_scene = SCENE_NUMBER_NO_SCENE;
    pkt.target_scene = 0xA5A5;
    pkt.remaining_time = model_transition_time_encode(1000);

    m_expected_status.status_code = pkt.status_code;
    m_expected_status.current_scene = pkt.current_scene;
    m_expected_status.target_scene = pkt.target_scene;
    m_expected_status.remaining_time_ms = 1000;
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, SCENE_STATUS_MAXLEN);
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);

    /* case: Optional params included transition completed. */
    pkt.status_code = SCENE_STATUS_SUCCESS;
    pkt.current_scene = 0xA5A5;
    pkt.target_scene = SCENE_NUMBER_NO_SCENE;
    pkt.remaining_time = 0;

    m_expected_status.status_code = pkt.status_code;
    m_expected_status.current_scene = pkt.current_scene;
    m_expected_status.target_scene = pkt.target_scene;
    m_expected_status.remaining_time_ms = 0;
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, SCENE_STATUS_MAXLEN);
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);

    /* case: Message with invalid length was received, this should not trigger any TEST() */
    m_expected_status.status_code = SCENE_STATUS_SUCCESS;
    m_expected_status.current_scene = 0xCAFE;
    m_expected_status.target_scene = SCENE_NUMBER_NO_SCENE;
    m_expected_status.remaining_time_ms = 0;
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, SCENE_STATUS_MAXLEN+1);
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);
}

void test_register_status_handle(void)
{
    access_message_rx_t request_msg;
    scene_register_status_msg_pkt_t pkt;
    uint16_t len;

    typedef struct __attribute((packed))
    {
        uint8_t status_code;
        uint16_t current_scene;
        uint16_t scenes[SCENE_REGISTER_ARRAY_SIZE];
    } test_vector_t;
    
    test_vector_t test_vector[] = {
        {SCENE_STATUS_SUCCESS, 1, {1,0,0,0,0,0,0,0,0, 0, 0, 0, 0, 0, 0, 0}},
        {SCENE_STATUS_SUCCESS, 2, {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16}},
        {SCENE_STATUS_SUCCESS, 3, {1,2,3,4,5,6,7,8,0, 0, 0, 0, 0, 0, 0, 0}}
    };
    
    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_client_init(&m_client, TEST_ELEMENT_INDEX));

    uint16_t opcode = SCENE_OPCODE_REGISTER_STATUS;

    /* case: Scenes params not included */
    pkt.status_code = SCENE_STATUS_SUCCESS;
    pkt.current_scene = SCENE_NUMBER_NO_SCENE;
    len = SCENE_REGISTER_STATUS_MINLEN;
    m_expected_register_status.status_code = pkt.status_code;
    m_expected_register_status.current_scene = pkt.current_scene;
    memset(m_expected_register_status.scenes, 0, SCENE_REGISTER_ARRAY_SIZE);
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, len);
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);

    /* Run test vectors */
    for (uint32_t i = 0; i < ARRAY_SIZE(test_vector); ++i)
    {
        m_expected_register_status.status_code = test_vector[i].status_code;
        m_expected_register_status.current_scene = test_vector[i].current_scene;
        memset(m_expected_register_status.scenes, 0, SCENE_REGISTER_ARRAY_SIZE);
        uint32_t element = 0;
        while ((element < SCENE_REGISTER_ARRAY_SIZE) && 
               (test_vector[i].scenes[element] != SCENE_NUMBER_NO_SCENE))
        {
            m_expected_register_status.scenes[element] = test_vector[i].scenes[element];
            element++;
        }
        len = SCENE_REGISTER_STATUS_MINLEN + (2 * element);
        ACCESS_MESSAGE_RX(request_msg, test_vector[i], opcode, len);
        helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);
    }

    /* case: Message with invalid length was received, this should not trigger any TEST() */
    m_expected_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_expected_register_status.current_scene = 0xCAFE;
    ACCESS_MESSAGE_RX(request_msg, pkt, opcode, (SCENE_REGISTER_STATUS_MAXLEN + 1));
    helper_call_opcode_handler(m_client.model_handle, &request_msg, &m_client);
}
