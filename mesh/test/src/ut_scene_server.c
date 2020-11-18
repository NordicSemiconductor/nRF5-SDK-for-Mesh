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

#include "scene_setup_server.h"
#include "scene_messages.h"

#include <stdint.h>
#include <string.h>

#include <unity.h>
#include <cmock.h>

#include "model_common.h"
#include "access_mock.h"
#include "access_config_mock.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "generic_dtt_server_mock.h"


/* Include module to be tested (to get access to internal state) */
#include "../../../models/model_spec/scene/src/scene_setup_server.c"

#define TEST_ELEMENT_INDEX          (7)
#define TEST_MODEL_HANDLE           (14)
#define TEST_MODEL_UNACK_REPEATS    (3)

#define TIMER_NOW_VAL         (0x0100)

static scene_setup_server_t m_setup_server;
static scene_setup_server_callbacks_t m_setup_server_cbs;
static generic_dtt_server_t m_test_dtt_server_for_scene;

static scene_store_params_t m_expected_p_in_store;
static scene_delete_params_t m_expected_p_in_delete;
static scene_recall_params_t m_expected_p_in_recall;
static model_transition_t m_expected_p_in_transition;
static bool m_expected_p_in_null;

static void scene_state_store_cb(const scene_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const scene_store_params_t * p_in,
                                 scene_register_status_params_t * p_out);

static void scene_state_delete_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_delete_params_t * p_in,
                                  scene_register_status_params_t * p_out);

static void scene_state_get_cb(const scene_setup_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               scene_status_params_t * p_out);

static void scene_state_register_get_cb(const scene_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        scene_register_status_params_t * p_out);

static void scene_state_recall_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_recall_params_t * p_in,
                                  const model_transition_t * p_in_transition,
                                  scene_status_params_t * p_out);

static scene_status_params_t m_test_status;
static scene_register_status_params_t m_test_register_status;

#define ACCESS_MESSAGE_RX(var, data, op, len) \
    do { \
        var.opcode.opcode = op; \
        var.opcode.company_id = ACCESS_COMPANY_ID_NONE; \
        var.p_data = (const uint8_t *) &data; \
        var.length = len; \
        var.meta_data.src.type = NRF_MESH_ADDRESS_TYPE_UNICAST; \
        var.meta_data.src.value = 0x0100; \
        var.meta_data.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST; \
        var.meta_data.dst.value = 0x0200; \
    } while(0)

static void * mp_model_args;
static access_model_handle_t m_model_handle;
static access_model_add_params_t m_model_init_params;
static const access_opcode_handler_t * mp_opcode_handlers[2];
static uint16_t m_num_opcodes[2];
static uint8_t m_opcode_handlers_cnt;
static uint16_t m_element_index = TEST_ELEMENT_INDEX;

static uint32_t access_model_add_mock(const access_model_add_params_t * p_init_params,
        access_model_handle_t * p_model_handle, int count)
{
    m_model_init_params = *p_init_params;

    m_model_handle = *p_model_handle;
    mp_opcode_handlers[m_opcode_handlers_cnt] = p_init_params->p_opcode_handlers;
    m_num_opcodes[m_opcode_handlers_cnt] = p_init_params->opcode_count;
    m_opcode_handlers_cnt++;

    mp_model_args = p_init_params->p_args;

    if (count == 0)
    {
        *p_model_handle = TEST_MODEL_HANDLE + 1;
        TEST_ASSERT_EQUAL(SCENE_SERVER_MODEL_ID, p_init_params->model_id.model_id);
    }
    else
    {
        *p_model_handle = TEST_MODEL_HANDLE;
        TEST_ASSERT_EQUAL(SCENE_SETUP_SERVER_MODEL_ID, p_init_params->model_id.model_id);
    }
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_init_params->model_id.company_id);
    TEST_ASSERT_EQUAL(TEST_ELEMENT_INDEX, p_init_params->element_index);

    return NRF_SUCCESS;
}

static bool m_publish_expected;
static access_model_handle_t m_publish_expected_handle;
static uint16_t m_publish_expected_opcode;
static void * mp_publish_expected_data;
static uint16_t m_publish_expected_data_length;

#define EXPECT_STATUS_PUBLISH_TX(x_handle, x_expected_data) \
    do { \
        TEST_ASSERT_FALSE(m_publish_expected); \
        m_publish_expected = true; \
        access_model_publish_StubWithCallback(access_model_status_publish_mock);\
        m_publish_expected_handle = x_handle; \
        m_publish_expected_opcode = SCENE_OPCODE_STATUS; \
        static scene_status_msg_pkt_t expected_data; \
        expected_data.status_code = x_expected_data.status_code; \
        expected_data.current_scene = x_expected_data.current_scene; \
        expected_data.target_scene = x_expected_data.target_scene; \
        expected_data.remaining_time = model_transition_time_encode(x_expected_data.remaining_time_ms);\
        mp_publish_expected_data = (void *) &expected_data; \
        if (x_expected_data.remaining_time_ms > 0) \
        { \
            m_publish_expected_data_length = SCENE_STATUS_MAXLEN; \
        } \
        else \
        { \
            m_publish_expected_data_length = SCENE_STATUS_MINLEN; \
        } \
    } while(0)

static uint32_t access_model_status_publish_mock(access_model_handle_t handle,
                                                 const access_message_tx_t * p_message,
                                                 int count)
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

static bool m_reply_expected;
static access_model_handle_t m_reply_expected_handle;
static uint16_t m_reply_expected_opcode;
static void * mp_reply_expected_data;
static uint16_t m_reply_expected_data_length;

#define EXPECT_STATUS_REPLY(x_handle, x_opcode, x_expected_data, x_expected_data_length) \
    do { \
        TEST_ASSERT_FALSE(m_reply_expected); \
        m_reply_expected = true; \
        m_reply_expected_handle = x_handle; \
        m_reply_expected_opcode = x_opcode; \
        static scene_status_msg_pkt_t expected_status_data; \
        expected_status_data.status_code = x_expected_data.status_code; \
        expected_status_data.current_scene = x_expected_data.current_scene; \
        expected_status_data.target_scene = x_expected_data.target_scene; \
        expected_status_data.remaining_time = model_transition_time_encode(x_expected_data.remaining_time_ms);\
        mp_reply_expected_data = (void *) &expected_status_data; \
        m_reply_expected_data_length = x_expected_data_length; \
        access_model_reply_StubWithCallback(access_model_reply_mock); \
    } while(0)

#define EXPECT_REGISTER_STATUS_REPLY(x_handle, x_opcode, x_expected_data, x_expected_data_length) \
    do { \
        TEST_ASSERT_FALSE(m_reply_expected); \
        m_reply_expected = true; \
        m_reply_expected_handle = x_handle; \
        m_reply_expected_opcode = x_opcode; \
        static uint8_t expected_register_status_data[SCENE_REGISTER_STATUS_MAXLEN]; \
        static scene_register_status_msg_pkt_t * p_expected_register_status_data = \
            (scene_register_status_msg_pkt_t *)expected_register_status_data; \
        p_expected_register_status_data->status_code = x_expected_data.status_code; \
        p_expected_register_status_data->current_scene = x_expected_data.current_scene; \
        uint32_t j =  0; \
        for (uint32_t k = 0; k < ARRAY_SIZE(x_expected_data.scenes); ++k) \
        { \
            if (x_expected_data.scenes[k] != SCENE_NUMBER_NO_SCENE) \
            { \
                p_expected_register_status_data->scenes[j++] = x_expected_data.scenes[k]; \
            } \
        } \
        mp_reply_expected_data = (void *) p_expected_register_status_data; \
        m_reply_expected_data_length = x_expected_data_length; \
        access_model_reply_StubWithCallback(access_model_reply_mock); \
    } while(0)

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

static void helper_expect_for_model_init_success(void)
{
    access_model_add_StubWithCallback(access_model_add_mock);
    access_model_add_StubWithCallback(access_model_add_mock);
    access_model_element_index_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_element_index_get_ReturnThruPtr_p_element_index(&m_element_index);
    access_model_subscription_lists_share_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_subscription_lists_share_ExpectAnyArgsAndReturn(NRF_SUCCESS);
}

static void helper_init_model_context_and_expectations(void)
{
    m_total_scene_instances = 0;
    m_setup_server_cbs.scene_cbs.store_cb = scene_state_store_cb;
    m_setup_server_cbs.scene_cbs.delete_cb = scene_state_delete_cb;
    m_setup_server_cbs.scene_cbs.get_cb = scene_state_get_cb;
    m_setup_server_cbs.scene_cbs.register_get_cb = scene_state_register_get_cb;
    m_setup_server_cbs.scene_cbs.recall_cb = scene_state_recall_cb;
    m_setup_server.settings.p_callbacks = &m_setup_server_cbs;
    m_setup_server.settings.force_segmented = false;
    m_setup_server.settings.transmic_size = NRF_MESH_TRANSMIC_SIZE_SMALL;
    m_opcode_handlers_cnt = 0;
    memset(&m_setup_server.scene_srv, 0, sizeof(scene_server_t));
    memset(&m_test_status, 0, sizeof(m_test_status));
    memset(&m_test_register_status, 0, sizeof(m_test_register_status));
    helper_expect_for_model_init_success();
}

static void helper_call_opcode_handler(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    bool handler_found = false;

    for (uint16_t cnt = 0; cnt < m_opcode_handlers_cnt; cnt++)
    {
        for (uint16_t i = 0; i < m_num_opcodes[cnt]; ++i)
        {
            if (memcmp(&(mp_opcode_handlers[cnt])[i].opcode, &p_message->opcode, sizeof(access_opcode_t)) == 0)
            {
                (mp_opcode_handlers[cnt])[i].handler(handle, p_message, p_args);
                handler_found = true;
                break;
           }
        }

        if (handler_found)
        {
            break;
        }
    }

    TEST_ASSERT_TRUE(handler_found);
}

static void helper_expected_recall_cb_values(scene_recall_msg_pkt_t * pkt, uint8_t pkt_len)
{
    m_expected_p_in_recall.scene_number = pkt->scene_number;
    m_expected_p_in_recall.tid = pkt->tid;
    if (pkt_len == SCENE_RECALL_MINLEN)
    {
        m_expected_p_in_null = true;
    }
    else
    {
        m_expected_p_in_null = false;
        m_expected_p_in_transition.transition_time_ms = model_transition_time_decode(pkt->transition_time);
        m_expected_p_in_transition.delay_ms = model_delay_decode(pkt->delay);
    }

    timer_now_ExpectAndReturn(TIMER_NOW_VAL);
    timer_sch_reschedule_ExpectAnyArgs();
}

/******** Model Callbacks ********/
static void scene_state_store_cb(const scene_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const scene_store_params_t * p_in,
                                 scene_register_status_params_t * p_out)
{
    printf("scene_state_store_cb\n");
    TEST_ASSERT_NOT_NULL(p_meta);

    TEST_ASSERT_EQUAL(m_expected_p_in_store.scene_number, p_in->scene_number);

    if (p_out != NULL)
    {
        *p_out = m_test_register_status;
    }
}

static void scene_state_delete_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_delete_params_t * p_in,
                                  scene_register_status_params_t * p_out)
{
    printf("scene_state_delete_cb\n");
    TEST_ASSERT_NOT_NULL(p_meta);

    TEST_ASSERT_EQUAL(m_expected_p_in_delete.scene_number, p_in->scene_number);

    if (p_out != NULL)
    {
        *p_out = m_test_register_status;
    }
}

static void scene_state_get_cb(const scene_setup_server_t * p_self,
                               const access_message_rx_meta_t * p_meta,
                               scene_status_params_t * p_out)
{
    printf("scene_state_get_cb\n");
    if (m_publish_expected)
    {
        TEST_ASSERT_EQUAL(NULL, p_meta);
    }
    else
    {
        TEST_ASSERT_NOT_NULL(p_meta);
    }

    if (p_out != NULL)
    {
        *p_out = m_test_status;
    }
}


static void scene_state_register_get_cb(const scene_setup_server_t * p_self,
                                        const access_message_rx_meta_t * p_meta,
                                        scene_register_status_params_t * p_out)
{
    printf("scene_state_register_get_cb\n");
    TEST_ASSERT_NOT_NULL(p_meta);

    if (p_out != NULL)
    {
        *p_out = m_test_register_status;
    }
}

static void scene_state_recall_cb(const scene_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  const scene_recall_params_t * p_in,
                                  const model_transition_t * p_in_transition,
                                  scene_status_params_t * p_out)
{
    printf("scene_state_recall_cb\n");
    TEST_ASSERT_NOT_NULL(p_meta);

    TEST_ASSERT_EQUAL(m_expected_p_in_recall.scene_number, p_in->scene_number);
    TEST_ASSERT_EQUAL(m_expected_p_in_recall.tid, p_in->tid);

    if (m_expected_p_in_null)
    {
        TEST_ASSERT_NULL(p_in_transition);
    }
    else
    {
        TEST_ASSERT_EQUAL(m_expected_p_in_transition.transition_time_ms, p_in_transition->transition_time_ms);
        TEST_ASSERT_EQUAL(m_expected_p_in_transition.delay_ms, p_in_transition->delay_ms);
    }

    if (p_out != NULL)
    {
        *p_out = m_test_status;
    }
}


/******** Setup and Tear Down ********/
void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
    generic_dtt_server_mock_Init();
}

void tearDown(void)
{
    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    generic_dtt_server_mock_Verify();
    generic_dtt_server_mock_Destroy();
}

/******** Tests ********/
void test_scene_setup_server_init(void)
{
    /* Null checks */
    memset(&m_setup_server, 0, sizeof(scene_setup_server_t));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(NULL, TEST_ELEMENT_INDEX));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    m_setup_server.settings.p_callbacks = &m_setup_server_cbs;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    m_setup_server_cbs.scene_cbs.store_cb = scene_state_store_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    m_setup_server_cbs.scene_cbs.delete_cb = scene_state_delete_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    m_setup_server_cbs.scene_cbs.get_cb = scene_state_get_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    m_setup_server_cbs.scene_cbs.register_get_cb = scene_state_register_get_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    m_setup_server_cbs.scene_cbs.recall_cb = scene_state_recall_cb;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    m_setup_server.p_gen_dtt_server = &m_test_dtt_server_for_scene;

    /* Wrong element index of DTT server */
    uint16_t wrong_element_index = TEST_ELEMENT_INDEX + 1;
    access_model_element_index_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_element_index_get_ReturnThruPtr_p_element_index(&wrong_element_index);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* Access model add/subscritpition failures. */
    access_model_add_ExpectAnyArgsAndReturn(NRF_ERROR_INVALID_LENGTH);
    access_model_element_index_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_element_index_get_ReturnThruPtr_p_element_index(&m_element_index);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    access_model_element_index_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_element_index_get_ReturnThruPtr_p_element_index(&m_element_index);
    access_model_add_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_add_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_subscription_lists_share_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    access_model_element_index_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_element_index_get_ReturnThruPtr_p_element_index(&m_element_index);
    access_model_add_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_add_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_subscription_lists_share_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_model_subscription_lists_share_ExpectAnyArgsAndReturn(NRF_ERROR_FORBIDDEN);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* All mandatory API inputs are correct */
    m_opcode_handlers_cnt = 0;
    helper_expect_for_model_init_success();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
    TEST_ASSERT_EQUAL(NRF_ERROR_RESOURCES, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));
}

void test_scene_server_status_publish(void)
{
    scene_status_params_t status = {0};

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* case: NULL checks */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_server_status_publish(NULL, &status));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, scene_server_status_publish(&m_setup_server.scene_srv, NULL));

    /* case: Invalid param check */
    status.status_code = SCENE_STATUS_SUCCESS;
    status.current_scene = SCENE_NUMBER_NO_SCENE;
    status.target_scene = SCENE_NUMBER_NO_SCENE;
    status.remaining_time_ms = TRANSITION_TIME_STEP_10M_MAX + 1;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, scene_server_status_publish(&m_setup_server.scene_srv, &status));

    /* case: Message ok, excluding optional params */
    status.status_code = SCENE_STATUS_SUCCESS;
    status.current_scene = SCENE_NUMBER_NO_SCENE;
    status.target_scene = 0;
    status.remaining_time_ms = 0;
    EXPECT_STATUS_PUBLISH_TX(m_setup_server.scene_srv.model_handle, status);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_server_status_publish(&m_setup_server.scene_srv, &status));

    /* case: Message ok, included optional params, transition to target */
    status.status_code = SCENE_STATUS_SUCCESS;
    status.current_scene = SCENE_NUMBER_NO_SCENE;
    status.target_scene = 0xA5A5;
    status.remaining_time_ms = 1000;
    EXPECT_STATUS_PUBLISH_TX(m_setup_server.scene_srv.model_handle, status);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_server_status_publish(&m_setup_server.scene_srv, &status));

    /* case: Message ok, excluding optional params, transition completed. */
    status.status_code = SCENE_STATUS_SUCCESS;
    status.current_scene = 0xA5A5;
    status.target_scene = SCENE_NUMBER_NO_SCENE;
    status.remaining_time_ms = 0;
    EXPECT_STATUS_PUBLISH_TX(m_setup_server.scene_srv.model_handle, status);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_server_status_publish(&m_setup_server.scene_srv, &status));
}

void test_periodic_publish_cb(void)
{
    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    m_test_status.current_scene = SCENE_STATUS_SUCCESS;
    m_test_status.current_scene = SCENE_NUMBER_NO_SCENE;
    m_test_status.target_scene = SCENE_NUMBER_NO_SCENE;
    m_test_status.remaining_time_ms = 0;

    EXPECT_STATUS_PUBLISH_TX(m_setup_server.model_handle, m_test_status);
    m_model_init_params.publish_timeout_cb(TEST_MODEL_HANDLE, &m_setup_server);
}

void test_handle_store(void)
{
    uint16_t set_scene;
    uint32_t len;

    access_message_rx_t request_msg;
    scene_store_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* STORE */
    /* case 1: valid message, reply no optional data. */
    set_scene = 1;

    pkt.scene_number = set_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_STORE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_store.scene_number = pkt.scene_number;

    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = set_scene;
    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, SCENE_REGISTER_STATUS_MINLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 2: Invalid length message */
    set_scene = 2;

    pkt.scene_number = set_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_STORE, SCENE_STORE_DELETE_LEN + 1);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 3: valid message, reply optional data. */
    set_scene = 3;

    pkt.scene_number = set_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_STORE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_store.scene_number = pkt.scene_number;

    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = set_scene;
    len = SCENE_REGISTER_STATUS_MINLEN;
    for (uint32_t i = 0; i < 3; i++)
    {
        m_test_register_status.scenes[i] = i + 1;
        len += 2;
    }
    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, len);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 4: valid message, reply register full with optional data. */
    set_scene = 116;

    pkt.scene_number = set_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_STORE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_store.scene_number = pkt.scene_number;

    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = set_scene;
    len = SCENE_REGISTER_STATUS_MINLEN;
    for (uint32_t i = 0; i < SCENE_REGISTER_ARRAY_SIZE; i++)
    {
        m_test_register_status.scenes[i] = i + 101;
        len += 2;
    }
    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, len);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 5: valid message, reply error code register is full with optional data. */
    set_scene = 17;

    pkt.scene_number = set_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_STORE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_store.scene_number = pkt.scene_number;

    m_test_register_status.status_code = SCENE_STATUS_REGISTER_FULL;
    m_test_register_status.current_scene = 16;
    len = SCENE_REGISTER_STATUS_MINLEN;
    for (uint32_t i = 0; i < SCENE_REGISTER_ARRAY_SIZE; i++)
    {
        m_test_register_status.scenes[i] = i + 1;
        len += 2;
    }
    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, len);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* STORE - Unacknowledged*/
    /* case 1: valid message */
    set_scene = 1;

    pkt.scene_number = set_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_STORE_UNACKNOWLEDGED, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_store.scene_number = pkt.scene_number;

    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 2: Invalid length message */
    set_scene = 2;

    pkt.scene_number = set_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_STORE_UNACKNOWLEDGED, SCENE_STORE_DELETE_LEN + 1);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* Do a check if any reply that was expected was not received. */
    TEST_ASSERT_FALSE(m_reply_expected);
}

void test_handle_delete(void)
{
    uint16_t delete_scene;
    uint32_t len;

    access_message_rx_t request_msg;
    scene_delete_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* DELETE */
    /* case 1: valid message, reply no optional data. */
    pkt.scene_number = 1;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_DELETE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_delete.scene_number = pkt.scene_number;

    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = SCENE_NUMBER_NO_SCENE;
    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, SCENE_REGISTER_STATUS_MINLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 2: Invalid length message */
    pkt.scene_number = 2;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_DELETE, SCENE_STORE_DELETE_LEN + 1);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* Setup test register used in the next cases 3 - 5. */
    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = 16;
    len = SCENE_REGISTER_STATUS_MINLEN;
    for (uint32_t i = 0; i < SCENE_REGISTER_ARRAY_SIZE; i++)
    {
        m_test_register_status.scenes[i] = i + 1;
        len += 2;
    }

    /* case 3: valid message, reply optional data. */
    delete_scene = 3;
    pkt.scene_number = delete_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_DELETE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_delete.scene_number = pkt.scene_number;

    m_test_register_status.scenes[delete_scene - 1] = SCENE_NUMBER_NO_SCENE;
    len -= 2;

    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, len);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 4: valid message, reply register one less than full with optional data. */
    delete_scene = 16;

    pkt.scene_number = delete_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_DELETE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_delete.scene_number = pkt.scene_number;

    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = SCENE_NUMBER_NO_SCENE;
    m_test_register_status.scenes[delete_scene - 1] = SCENE_NUMBER_NO_SCENE;
    len -= 2;

    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, len);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 5: valid message, reply register not found with optional data. */
    delete_scene = 17;

    pkt.scene_number = delete_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_DELETE, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_delete.scene_number = pkt.scene_number;

    m_test_register_status.status_code = SCENE_STATUS_NOT_FOUND;

    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.model_handle, SCENE_OPCODE_REGISTER_STATUS,
                                 m_test_register_status, len);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* DELETE - Unacknowledged*/
    /* case 6: valid message */
    delete_scene = 1;

    pkt.scene_number = delete_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_DELETE_UNACKNOWLEDGED, SCENE_STORE_DELETE_LEN);
    m_expected_p_in_delete.scene_number = pkt.scene_number;

    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* case 7: Invalid length message */
    delete_scene = 2;

    pkt.scene_number = delete_scene;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_DELETE_UNACKNOWLEDGED, SCENE_STORE_DELETE_LEN + 1);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server);

    /* Do a check if any reply that was expected was not received. */
    TEST_ASSERT_FALSE(m_reply_expected);
}

void test_handle_get(void)
{
    access_message_rx_t request_msg;
    uint8_t * p_pkt = NULL;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* GET */
    /* case 1: This message does not have any optional parameters */
    ACCESS_MESSAGE_RX(request_msg, p_pkt, SCENE_OPCODE_GET, 0);

    m_test_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_status.current_scene = SCENE_NUMBER_NO_SCENE;
    m_test_status.target_scene = SCENE_NUMBER_NO_SCENE;
    m_test_status.remaining_time_ms = 0;
    EXPECT_STATUS_REPLY(m_setup_server.scene_srv.model_handle, SCENE_OPCODE_STATUS, m_test_status, SCENE_STATUS_MINLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 2: Status is sent with optional parameters */
    ACCESS_MESSAGE_RX(request_msg, p_pkt, SCENE_OPCODE_GET, 0);

    m_test_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_status.current_scene = SCENE_NUMBER_NO_SCENE;
    m_test_status.target_scene = 1;
    m_test_status.remaining_time_ms = 100;
    EXPECT_STATUS_REPLY(m_setup_server.scene_srv.model_handle, SCENE_OPCODE_STATUS, m_test_status, SCENE_STATUS_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 3: Invalid length, do not expect reply */
    ACCESS_MESSAGE_RX(request_msg, p_pkt, SCENE_OPCODE_GET, 1);

    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* Do a check if any reply that was expected was not received. */
    TEST_ASSERT_FALSE(m_reply_expected);
}

void test_handle_regiser_get(void)
{
    access_message_rx_t request_msg;
    uint8_t * p_pkt = NULL;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* REGISTER_GET */
    /* case 1: This message does not have any optional parameters */
    ACCESS_MESSAGE_RX(request_msg, p_pkt, SCENE_OPCODE_REGISTER_GET, 0);

    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = SCENE_NUMBER_NO_SCENE;
    memset(m_test_register_status.scenes, 0, sizeof(m_test_register_status.scenes));
    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.scene_srv.model_handle, SCENE_OPCODE_REGISTER_STATUS, m_test_register_status, SCENE_REGISTER_STATUS_MINLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 2: Status is sent with optional parameters */
    ACCESS_MESSAGE_RX(request_msg, p_pkt, SCENE_OPCODE_REGISTER_GET, 0);

    m_test_register_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_register_status.current_scene = SCENE_NUMBER_NO_SCENE;
    for (uint32_t i = 0; i < SCENE_REGISTER_ARRAY_SIZE; i++)
    {
        m_test_register_status.scenes[i] = i+1;
    }
    EXPECT_REGISTER_STATUS_REPLY(m_setup_server.scene_srv.model_handle, SCENE_OPCODE_REGISTER_STATUS, m_test_register_status, SCENE_REGISTER_STATUS_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 3: Invalid length, do not expect reply */
    ACCESS_MESSAGE_RX(request_msg, p_pkt, SCENE_OPCODE_REGISTER_GET, 1);

    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* Do a check if any reply that was expected was not received. */
    TEST_ASSERT_FALSE(m_reply_expected);
}

void test_handle_recall(void)
{
    uint16_t set_scene;
    uint16_t previous_scene = SCENE_NUMBER_NO_SCENE;
    uint8_t set_tid;
    uint32_t set_time;
    uint32_t set_delay;
    access_message_rx_t request_msg;
    scene_recall_msg_pkt_t pkt;

    helper_init_model_context_and_expectations();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, scene_setup_server_init(&m_setup_server, TEST_ELEMENT_INDEX));

    /* RECALL */
    /* case 1: valid message, reply no optional data. */
    set_scene = 1;
    set_tid = 1;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = 0;
    pkt.delay = 0;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL, SCENE_RECALL_MINLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MINLEN);

    m_test_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_status.current_scene = set_scene;
    EXPECT_STATUS_REPLY(m_setup_server.scene_srv.model_handle, SCENE_OPCODE_STATUS, m_test_status, SCENE_STATUS_MINLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 2: Message with optional params, non-zero transition. Note: Actual transition behaviour
       is not tested here. */
    previous_scene = set_scene;
    set_scene = 2;
    set_tid++;
    set_time = 500;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = 0;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL, SCENE_RECALL_MAXLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MAXLEN);

    m_test_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_status.current_scene = previous_scene;
    m_test_status.target_scene = set_scene;
    m_test_status.remaining_time_ms = set_time;
    EXPECT_STATUS_REPLY(m_setup_server.scene_srv.model_handle, SCENE_OPCODE_STATUS, m_test_status, SCENE_STATUS_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 3: Message with delay, non-zero transition. Note: Actual transition behaviour is not
       tested here. */
    previous_scene = set_scene;
    set_scene = 3;
    set_tid++;
    set_time = 900;
    set_delay = 100;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL, SCENE_RECALL_MAXLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MAXLEN);

    m_test_status.status_code = SCENE_STATUS_SUCCESS;
    m_test_status.current_scene = previous_scene;
    m_test_status.target_scene = set_scene;
    m_test_status.remaining_time_ms = set_time;
    EXPECT_STATUS_REPLY(m_setup_server.scene_srv.model_handle, SCENE_OPCODE_STATUS, m_test_status, SCENE_STATUS_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 4: Same TID is ignored */
    set_scene = 4;
    set_time = 100;
    set_delay = 900;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL, SCENE_RECALL_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 5: Invalid transition time. */
    previous_scene = set_scene;
    set_scene = 4;
    set_tid++;
    set_time = TRANSITION_TIME_STEP_10M_MAX+1;
    set_delay = 100;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL, SCENE_RECALL_MAXLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 6: Invalid length message. */
    previous_scene = set_scene;
    set_scene = 3;
    set_tid++;
    set_time = 900;
    set_delay = 100;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL, SCENE_RECALL_MAXLEN + 1);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* RECALL - Unacknowledged*/
    /* case 1: Message without optional params, instant transition */
    set_scene = 1;
    set_tid = 1;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = 0;
    pkt.delay = 0;
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, SCENE_RECALL_MINLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MINLEN);

    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 2: Message with optional params, non-zero transition. Note: Actual transition behaviour
       is not tested here. */
    set_scene = 1;
    set_tid++;
    set_time = 500;
    set_delay = 0;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, SCENE_RECALL_MINLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MINLEN);

    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 3: Message with delay, non-zero transition. Note: Actual transition behaviour is not
       tested here. */
    previous_scene = set_scene;
    set_scene = 3;
    set_tid++;
    set_time = 900;
    set_delay = 100;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, SCENE_RECALL_MAXLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MAXLEN);

    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 4: Same TID is ignored */
    set_scene = 4;
    set_time = 100;
    set_delay = 900;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, SCENE_RECALL_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 5: Invalid transition time. */
    previous_scene = set_scene;
    set_scene = 4;
    set_tid++;
    set_time = TRANSITION_TIME_STEP_10M_MAX+1;
    set_delay = 100;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(set_time);
    pkt.delay = model_delay_encode(set_delay);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, SCENE_RECALL_MAXLEN);
    helper_expected_recall_cb_values(&pkt, SCENE_RECALL_MAXLEN);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* case 6: Invalid length message */
    set_scene = 2;
    set_tid++;

    pkt.scene_number = set_scene;
    pkt.tid = set_tid;
    pkt.transition_time = model_transition_time_encode(100);;
    pkt.delay = model_delay_encode(900);
    ACCESS_MESSAGE_RX(request_msg, pkt, SCENE_OPCODE_RECALL_UNACKNOWLEDGED, SCENE_RECALL_MAXLEN + 1);
    helper_call_opcode_handler(m_setup_server.model_handle, &request_msg, &m_setup_server.scene_srv);

    /* Do a check if any reply that was expected was not received. */
    TEST_ASSERT_FALSE(m_reply_expected);
}