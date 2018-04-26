/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#include <stdlib.h>

#include <cmock.h>
#include <unity.h>

#include "access_mock.h"
#include "access_config_mock.h"
#include "composition_data_mock.h"
#include "device_state_manager_mock.h"
#include "heartbeat_mock.h"
#include "net_beacon_mock.h"
#include "nrf_mesh_opt_mock.h"
#include "nrf_mesh_keygen_mock.h"
#include "rand_mock.h"
#include "nrf_mesh_events_mock.h"
#include "mesh_stack_mock.h"

#include "access.h"
#include "config_messages.h"
#include "config_opcodes.h"
#include "config_server.h"
#include "nordic_common.h"
#include "packed_index_list.h"

#define CONFIG_SERVER_MODEL_ID  0x0000

#define VERIFY_REPLY_OPCODE(c)                                          \
    do {                                                                \
        TEST_ASSERT_EQUAL(c, m_previous_reply.opcode.opcode);           \
        TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, m_previous_reply.opcode.company_id); \
    } while (0)

#define DSM_SUBNET_GET_ALL_MOCK_SETUP(key_array, index_count, retval)   \
    do {                                                                \
        mp_dsm_subnet_get_all_index_array = key_array;                  \
        m_dsm_subnet_get_all_index_count = index_count;                 \
        m_dsm_subnet_get_all_retval = retval;                           \
    } while (0)

#define DSM_APPKEY_GET_ALL_MOCK_SETUP(subnet, key_array, index_count, retval) \
    do {                                                                \
        mp_dsm_appkey_get_all_index_array = key_array;                  \
        m_dsm_appkey_get_all_subnet = subnet;                           \
        m_dsm_appkey_get_all_index_count = index_count;                 \
        m_dsm_appkey_get_all_retval = retval;                           \
    } while (0)

#define ACCESS_MODEL_SUBSCRIPTIONS_GET_MOCK_SETUP(model_handle, addr_handle_array, addr_count, retval) \
    do {                                                                \
        m_access_model_subscriptions_get_model_handle = model_handle;   \
        mp_access_model_subscriptions_get_handle_array = addr_handle_array; \
        m_access_model_subscriptions_get_handle_count = addr_count;     \
        m_access_model_subscriptions_get_retval = retval;               \
    } while (0)

#define ACCESS_MODEL_APPLICATIONS_GET_MOCK_SETUP(model_handle, appkey_array, appkey_count, retval) \
    do {                                                                \
        m_access_model_applications_get_model_handle = model_handle;    \
        mp_access_model_applications_get_appkey_array = appkey_array;   \
        m_access_model_applications_get_appkey_count = appkey_count;    \
        m_access_model_applications_get_retval = retval;                \
    } while (0)

#define EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(element_address, index)  \
    do {                                                                \
        static dsm_local_unicast_address_t address;                     \
        address.address_start = element_address - index;                \
        address.count = index + 1;                                      \
        dsm_local_unicast_addresses_get_Expect(NULL);                   \
        dsm_local_unicast_addresses_get_IgnoreArg_p_address();          \
        dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&address); \
    } while (0)

static access_model_handle_t m_server_handle = 0xffff;
static const access_opcode_handler_t * mp_opcode_handlers;
static uint16_t m_num_opcodes;

static uint8_t * mp_previous_reply_buffer;
static access_message_tx_t m_previous_reply;
static bool m_previous_reply_received = false;

static mesh_key_index_t * mp_dsm_subnet_get_all_index_array;
static uint32_t m_dsm_subnet_get_all_index_count;
static uint32_t m_dsm_subnet_get_all_retval;

static mesh_key_index_t * mp_dsm_appkey_get_all_index_array;
static dsm_handle_t m_dsm_appkey_get_all_subnet;
static uint32_t m_dsm_appkey_get_all_index_count;
static uint32_t m_dsm_appkey_get_all_retval;

static access_model_handle_t m_access_model_subscriptions_get_model_handle;
static dsm_handle_t * mp_access_model_subscriptions_get_handle_array;
static uint16_t m_access_model_subscriptions_get_handle_count;
static uint32_t m_access_model_subscriptions_get_retval;

static access_model_handle_t m_access_model_applications_get_model_handle;
static dsm_handle_t * mp_access_model_applications_get_appkey_array;
static uint16_t m_access_model_applications_get_appkey_count;
static uint32_t m_access_model_applications_get_retval;

/*********** Helper functions ***********/

static void send_message(uint16_t opcode, const uint8_t * p_data, uint16_t length)
{
    access_message_rx_t message = {};
    message.opcode.opcode = opcode;
    message.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    message.p_data = p_data;
    message.length = length;
    /* The configuration server doesn't care about the message metadata,
     * so those fields are left as 0.                                    */

    bool opcode_found = false;
    for (uint32_t i = 0; i < m_num_opcodes; ++i)
    {
        if (mp_opcode_handlers[i].opcode.opcode == opcode)
        {
            opcode_found = true;
            mp_opcode_handlers[i].handler(m_server_handle, &message, NULL);
            break;
        }
    }

    TEST_ASSERT_TRUE_MESSAGE(opcode_found, "an opcode was not found!");
}

/*********** Additional mock functions ***********/
static uint32_t access_model_add_mock(const access_model_add_params_t * p_init_params,
        access_model_handle_t * p_model_handle, int count)
{
    m_server_handle = 0;
    *p_model_handle = m_server_handle;

    mp_opcode_handlers = p_init_params->p_opcode_handlers;
    m_num_opcodes = p_init_params->opcode_count;

    TEST_ASSERT_EQUAL(CONFIG_SERVER_MODEL_ID, p_init_params->model_id.model_id);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_init_params->model_id.company_id);
    TEST_ASSERT_EQUAL(0, p_init_params->element_index);

    return NRF_SUCCESS;
}

static uint32_t access_model_reply_mock(access_model_handle_t handle, const access_message_rx_t * p_message,
                                        const access_message_tx_t * p_reply, int count)
{
    TEST_ASSERT_EQUAL(m_server_handle, handle);

    if (mp_previous_reply_buffer != NULL)
    {
        free(mp_previous_reply_buffer);
    }

    mp_previous_reply_buffer = malloc(p_reply->length);
    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    memcpy(mp_previous_reply_buffer, p_reply->p_buffer, p_reply->length);

    m_previous_reply.opcode = p_reply->opcode;
    m_previous_reply.p_buffer = mp_previous_reply_buffer;
    m_previous_reply.length = p_reply->length;
    m_previous_reply.force_segmented = false;
    m_previous_reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    m_previous_reply_received = true;
    return NRF_SUCCESS;
}

static uint32_t access_model_subscriptions_get_mock(access_model_handle_t handle,
                                                    dsm_handle_t * p_address_handles, uint16_t * p_count, int calls)
{
    TEST_ASSERT_EQUAL(m_access_model_subscriptions_get_model_handle, handle);

    memcpy(p_address_handles, mp_access_model_subscriptions_get_handle_array,
           m_access_model_subscriptions_get_handle_count * sizeof(dsm_handle_t));
    *p_count = m_access_model_subscriptions_get_handle_count;
    return m_access_model_subscriptions_get_retval;
}

static uint32_t access_model_applications_get_mock(access_model_handle_t handle,
                                                   dsm_handle_t * p_appkey_handles, uint16_t * p_count, int calls)
{
    TEST_ASSERT_EQUAL(m_access_model_applications_get_model_handle, handle);

    memcpy(p_appkey_handles, mp_access_model_applications_get_appkey_array,
           m_access_model_applications_get_appkey_count * sizeof(dsm_handle_t));
    *p_count = m_access_model_applications_get_appkey_count;
    return m_access_model_applications_get_retval;
}

static uint32_t dsm_subnet_get_all_mock(mesh_key_index_t * p_netkey_indexes, uint32_t * p_count, int count)
{
    memcpy(p_netkey_indexes, mp_dsm_subnet_get_all_index_array, m_dsm_subnet_get_all_index_count * sizeof(mesh_key_index_t));
    *p_count = m_dsm_subnet_get_all_index_count;
    return m_dsm_subnet_get_all_retval;
}

static uint32_t dsm_appkey_get_all_mock(dsm_handle_t subnet_handle, mesh_key_index_t * p_appkey_indexes, uint32_t * p_count, int count)
{
    TEST_ASSERT_EQUAL(m_dsm_appkey_get_all_subnet, subnet_handle);
    memcpy(p_appkey_indexes, mp_dsm_appkey_get_all_index_array, m_dsm_appkey_get_all_index_count * sizeof(mesh_key_index_t));
    *p_count = m_dsm_appkey_get_all_index_count;
    return m_dsm_appkey_get_all_retval;
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{

}

/*********** Test Initialization and Finalization ***********/

void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    composition_data_mock_Init();
    device_state_manager_mock_Init();
    heartbeat_mock_Init();
    net_beacon_mock_Init();
    nrf_mesh_opt_mock_Init();
    nrf_mesh_keygen_mock_Init();
    rand_mock_Init();
    nrf_mesh_events_mock_Init();
    mesh_stack_mock_Init();

    m_previous_reply_received = false;
    mp_previous_reply_buffer = NULL;

    access_model_add_StubWithCallback(access_model_add_mock);
    access_model_reply_StubWithCallback(access_model_reply_mock);

    heartbeat_config_server_cb_set_Expect(NULL, NULL);
    heartbeat_config_server_cb_set_IgnoreArg_p_cb();
    heartbeat_config_server_cb_set_IgnoreArg_p_pub_cnt_cb();
    nrf_mesh_evt_handler_add_ExpectAnyArgs();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_server_init(config_server_evt_cb));
}

void tearDown(void)
{
    if (mp_previous_reply_buffer != NULL)
    {
        free(mp_previous_reply_buffer);
        mp_previous_reply_buffer = NULL;
    }

    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    composition_data_mock_Verify();
    composition_data_mock_Destroy();
    device_state_manager_mock_Verify();
    device_state_manager_mock_Destroy();
    heartbeat_mock_Verify();
    heartbeat_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    nrf_mesh_opt_mock_Verify();
    nrf_mesh_opt_mock_Destroy();
    nrf_mesh_keygen_mock_Verify();
    nrf_mesh_keygen_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    mesh_stack_mock_Verify();
    mesh_stack_mock_Destroy();
}

/*********** Test Cases ***********/

void test_config_beacon_get(void)
{
    send_message(CONFIG_OPCODE_BEACON_GET, NULL, 4); /* Test with invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    net_beacon_state_get_ExpectAndReturn(true);
    send_message(CONFIG_OPCODE_BEACON_GET, NULL, 0); /* Message with no parameters */

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_BEACON_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(1, m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_NET_BEACON_STATE_ENABLED, m_previous_reply.p_buffer[0]);

    m_previous_reply_received = false;
    net_beacon_state_get_ExpectAndReturn(false);
    send_message(CONFIG_OPCODE_BEACON_GET, NULL, 0);

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_BEACON_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(1, m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_NET_BEACON_STATE_DISABLED, m_previous_reply.p_buffer[0]);
}

void test_config_beacon_set(void)
{

    net_beacon_state_set_Expect(true);          /* This is the value we are trying to set the beacon state to */
    net_beacon_state_get_ExpectAndReturn(true); /* This is the value that determines the current beacon state, used in the reply */

    config_msg_net_beacon_set_t message = { .beacon_state = CONFIG_NET_BEACON_STATE_ENABLED };

    send_message(CONFIG_OPCODE_BEACON_SET, (const uint8_t *) &message, sizeof(message) + 3); /* Test with invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    send_message(CONFIG_OPCODE_BEACON_SET, (const uint8_t *) &message, sizeof(message) - 1); /* Test with invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    send_message(CONFIG_OPCODE_BEACON_SET, (const uint8_t *) &message, sizeof(message)); /* Test with correct parameters */

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_BEACON_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(1, m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_NET_BEACON_STATE_ENABLED, m_previous_reply.p_buffer[0]);

    m_previous_reply_received = false;
    net_beacon_state_set_Expect(false);
    net_beacon_state_get_ExpectAndReturn(false);

    message.beacon_state = CONFIG_NET_BEACON_STATE_DISABLED;
    send_message(CONFIG_OPCODE_BEACON_SET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_BEACON_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(1, m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_NET_BEACON_STATE_DISABLED, m_previous_reply.p_buffer[0]);
}

void test_composition_data_get(void)
{
    config_msg_composition_data_get_t message;

    /* Try first with an invalid page number: */
    message.page_number = 4;
    send_message(CONFIG_OPCODE_COMPOSITION_DATA_GET, (const uint8_t *) &message, sizeof(message));
    TEST_ASSERT_FALSE(m_previous_reply_received); /* no reply on invalid messages */

    /* Try with a valid message: */
    message.page_number = 0;

    uint16_t size = CONFIG_COMPOSITION_DATA_SIZE;
    config_composition_data_get_Expect(NULL, NULL);
    config_composition_data_get_IgnoreArg_p_data();
    config_composition_data_get_IgnoreArg_p_size();
    config_composition_data_get_ReturnThruPtr_p_size(&size);
    send_message(CONFIG_OPCODE_COMPOSITION_DATA_GET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_COMPOSITION_DATA_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_composition_data_status_t) + CONFIG_COMPOSITION_DATA_SIZE, m_previous_reply.length);

    const config_msg_composition_data_status_t * p_reply = (const config_msg_composition_data_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(0, p_reply->page_number);
}

void test_config_default_ttl_get(void)
{
    access_default_ttl_get_ExpectAndReturn(4);
    send_message(CONFIG_OPCODE_DEFAULT_TTL_GET, NULL, 0); /* Message with no parameters */

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_DEFAULT_TTL_STATUS);
    TEST_ASSERT_EQUAL(1, m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(4, m_previous_reply.p_buffer[0]);

    m_previous_reply_received = false;
    send_message(CONFIG_OPCODE_DEFAULT_TTL_GET, (const uint8_t *) "abcdef", 6); /* Invalid message length */
    TEST_ASSERT_FALSE(m_previous_reply_received);
}

void test_config_default_ttl_set(void)
{
    /* Test valid parameter path */
    const config_msg_default_ttl_set_t valid_message = { 8 };
    access_default_ttl_set_ExpectAndReturn(8, NRF_SUCCESS);
    send_message(CONFIG_OPCODE_DEFAULT_TTL_SET, (const uint8_t *) &valid_message, sizeof(valid_message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_DEFAULT_TTL_STATUS);
    TEST_ASSERT_EQUAL(1, m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(8, m_previous_reply.p_buffer[0]);

    /* Test invalid parameter path */
    m_previous_reply_received = false;
    const config_msg_default_ttl_set_t invalid_message_1 = { 1 };
    access_default_ttl_set_ExpectAndReturn(1, NRF_ERROR_INVALID_PARAM);
    send_message(CONFIG_OPCODE_DEFAULT_TTL_SET, (const uint8_t *) &invalid_message_1, sizeof(invalid_message_1));
    TEST_ASSERT_FALSE(m_previous_reply_received);

    m_previous_reply_received = false;
    const config_msg_default_ttl_set_t invalid_message_high = { NRF_MESH_TTL_MAX + 1 };
    access_default_ttl_set_ExpectAndReturn(NRF_MESH_TTL_MAX + 1, NRF_ERROR_INVALID_PARAM);
    send_message(CONFIG_OPCODE_DEFAULT_TTL_SET, (const uint8_t *) &invalid_message_high, sizeof(invalid_message_high));
    TEST_ASSERT_FALSE(m_previous_reply_received);

    m_previous_reply_received = false;
    send_message(CONFIG_OPCODE_DEFAULT_TTL_SET, NULL, 0); /* Message length too short */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    m_previous_reply_received = false;
    send_message(CONFIG_OPCODE_DEFAULT_TTL_SET, (const uint8_t *) "abcdefghijklmn", 14); /* Message length too long */
    TEST_ASSERT_FALSE(m_previous_reply_received);
}

void test_gatt_proxy_get(void)
{
    send_message(CONFIG_OPCODE_GATT_PROXY_GET, NULL, 4); /* Invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    send_message(CONFIG_OPCODE_GATT_PROXY_GET, NULL, 0);

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_GATT_PROXY_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(sizeof(config_msg_proxy_status_t), m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_GATT_PROXY_STATE_UNSUPPORTED, m_previous_reply.p_buffer[0]);
}

void test_gatt_proxy_set(void)
{
    const config_msg_proxy_set_t message = { .proxy_state = CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED };

    send_message(CONFIG_OPCODE_GATT_PROXY_SET, (const uint8_t *) &message, sizeof(config_msg_proxy_set_t) + 2); /* Invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    send_message(CONFIG_OPCODE_GATT_PROXY_SET, (const uint8_t *) &message, sizeof(config_msg_proxy_set_t) - 1); /* Invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    send_message(CONFIG_OPCODE_GATT_PROXY_SET, (const uint8_t *) &message, sizeof(config_msg_proxy_set_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_GATT_PROXY_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(sizeof(config_msg_proxy_status_t), m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_GATT_PROXY_STATE_UNSUPPORTED, m_previous_reply.p_buffer[0]);
}

void test_config_relay_get(void)
{
    nrf_mesh_opt_t relay_state = { .len = 4, .opt.val = CONFIG_RELAY_STATE_SUPPORTED_ENABLED };
    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_RELAY_ENABLE, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&relay_state);

    nrf_mesh_opt_t retransmits = { .len = 4, .opt.val = 2};
    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&retransmits);

    nrf_mesh_opt_t interval = { .len = 4, .opt.val = 50};
    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&interval);

    send_message(CONFIG_OPCODE_RELAY_GET, NULL, 0); /* Message with no parameters */

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_RELAY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_relay_status_t), m_previous_reply.length);

    const config_msg_relay_status_t * p_reply = (const config_msg_relay_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(CONFIG_RELAY_STATE_SUPPORTED_ENABLED, p_reply->relay_state);
    TEST_ASSERT_EQUAL(1, p_reply->relay_retransmit_count);
    TEST_ASSERT_EQUAL(4, p_reply->relay_retransmit_interval_steps);
}

void test_config_relay_set(void)
{
    const config_msg_relay_set_t message =
        {
            .relay_state = CONFIG_RELAY_STATE_SUPPORTED_ENABLED,
            .relay_retransmit_count = 2,
            .relay_retransmit_interval_steps = 1
        };

    nrf_mesh_opt_t expected_optval_enable = { .len = 4, .opt.val = 1 };
    nrf_mesh_opt_set_ExpectWithArrayAndReturn(NRF_MESH_OPT_NET_RELAY_ENABLE, &expected_optval_enable, 1, NRF_SUCCESS);

    nrf_mesh_opt_t expected_optval_retransmits = { .len = 4, .opt.val = message.relay_retransmit_count + 1 };
    nrf_mesh_opt_set_ExpectWithArrayAndReturn(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT,
                                              &expected_optval_retransmits, 1, NRF_SUCCESS);

    /* 10 ms * (steps + 1) according to Mesh Profile spec. 4.2.20.2. */
    nrf_mesh_opt_t expected_optval_interval = { .len = 4, .opt.val = 10*(message.relay_retransmit_interval_steps + 1) };
    nrf_mesh_opt_set_ExpectWithArrayAndReturn(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS,
                                              &expected_optval_interval, 1, NRF_SUCCESS);

    /* The server reads out the expected state back again when replying. */
    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_RELAY_ENABLE, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&expected_optval_enable);

    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_COUNT, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&expected_optval_retransmits);

    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_RELAY_RETRANSMIT_INTERVAL_MS, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&expected_optval_interval);

    send_message(CONFIG_OPCODE_RELAY_SET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_RELAY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_relay_status_t), m_previous_reply.length);

    const config_msg_relay_status_t * p_reply = (const config_msg_relay_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(CONFIG_RELAY_STATE_SUPPORTED_ENABLED, p_reply->relay_state);
    TEST_ASSERT_EQUAL(message.relay_retransmit_count, p_reply->relay_retransmit_count);
    TEST_ASSERT_EQUAL(message.relay_retransmit_interval_steps, p_reply->relay_retransmit_interval_steps);
}

void test_publication_get(void)
{
    config_msg_publication_get_t message[2] =
    {
        {
            .element_address = 0x3132,
            .model_id.sig        = { .model_id = 0x1351 }
        },
        {
            .element_address = 0x3132,
            .model_id.vendor        = {.model_id = 0x4321, .company_id = 8765 }
        }
    };

    for (uint8_t i = 0; i < 2; ++i)
    {
        bool sig_identifier = !i;

        uint16_t element_index = 4;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(message[i].element_address, element_index);

        access_model_handle_t model_handle = 8;
        access_model_id_t expected_id =
        {
            .model_id = sig_identifier ? message[i].model_id.sig.model_id : message[i].model_id.vendor.model_id,
            .company_id = sig_identifier ? ACCESS_COMPANY_ID_NONE : message[i].model_id.vendor.company_id,
        };

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        access_model_id_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_id_get_IgnoreArg_p_model_id();
        access_model_id_get_ReturnThruPtr_p_model_id(&expected_id);

        dsm_handle_t publish_address_handle = 9;
        access_model_publish_address_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_publish_address_get_IgnoreArg_p_address_handle();
        access_model_publish_address_get_ReturnThruPtr_p_address_handle(&publish_address_handle);

        nrf_mesh_address_t publish_address = { .type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = 0x1234 };
        dsm_address_get_ExpectAndReturn(publish_address_handle, NULL, NRF_SUCCESS);
        dsm_address_get_IgnoreArg_p_address();
        dsm_address_get_ReturnThruPtr_p_address(&publish_address);

        dsm_handle_t publish_appkey_handle = 10;
        access_model_publish_application_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_publish_application_get_IgnoreArg_p_appkey_handle();
        access_model_publish_application_get_ReturnThruPtr_p_appkey_handle(&publish_appkey_handle);

        mesh_key_index_t appkey_index = 22;
        dsm_appkey_handle_to_appkey_index_ExpectAndReturn(publish_appkey_handle, NULL, NRF_SUCCESS);
        dsm_appkey_handle_to_appkey_index_IgnoreArg_p_index();
        dsm_appkey_handle_to_appkey_index_ReturnThruPtr_p_index(&appkey_index);

        uint8_t publish_ttl = 54;
        access_model_publish_ttl_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_publish_ttl_get_IgnoreArg_p_ttl();
        access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&publish_ttl);

        access_publish_resolution_t publish_resolution = ACCESS_PUBLISH_RESOLUTION_1S;
        uint8_t publish_steps = 21;
        access_model_publish_period_get_ExpectAndReturn(model_handle, NULL, NULL, NRF_SUCCESS);
        access_model_publish_period_get_IgnoreArg_p_resolution();
        access_model_publish_period_get_IgnoreArg_p_step_number();
        access_model_publish_period_get_ReturnThruPtr_p_resolution(&publish_resolution);
        access_model_publish_period_get_ReturnThruPtr_p_step_number(&publish_steps);

        access_publish_retransmit_t publish_retransmit =
        {
            .count = 0x07,
            .interval_steps = 0x1F
        };
        access_model_publish_retransmit_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_model_publish_retransmit_get_ReturnThruPtr_p_retransmit_params(&publish_retransmit);

        send_message(CONFIG_OPCODE_MODEL_PUBLICATION_GET, (const uint8_t *) &message[i], sizeof(config_msg_publication_get_t) - sig_identifier * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_publication_status_t) - sig_identifier * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_publication_status_t * p_reply = (const config_msg_publication_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(message[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(0x1234, p_reply->publish_address);
        TEST_ASSERT_EQUAL(appkey_index, p_reply->state.appkey_index);
        TEST_ASSERT_EQUAL(0, p_reply->state.credential_flag); /* TODO: When friendship is supported, this flag can be used. */
        TEST_ASSERT_EQUAL(0, p_reply->state.rfu);
        TEST_ASSERT_EQUAL(publish_ttl, p_reply->state.publish_ttl);

        uint8_t expected_period = publish_resolution << ACCESS_PUBLISH_STEP_NUM_BITS | publish_steps;
        TEST_ASSERT_EQUAL(expected_period, p_reply->state.publish_period);
        TEST_ASSERT_EQUAL(publish_retransmit.count, p_reply->state.retransmit_count);
        TEST_ASSERT_EQUAL(publish_retransmit.interval_steps, p_reply->state.retransmit_interval);

        if (sig_identifier)
        {
            TEST_ASSERT_EQUAL_UINT16(message[i].model_id.sig.model_id, *((uint16_t *) &p_reply->state.model_id));
        }
        else
        {
            TEST_ASSERT_EQUAL_MEMORY(&message[i].model_id, &p_reply->state.model_id, sizeof(config_model_id_t));
        }
    }
}

void test_publication_set(void)
{
    config_msg_publication_set_t messages[2] =
    {
        {
            .element_address = 0x1234,
            .publish_address = 0x4321,
            .state = {
                .appkey_index = 56,
                .publish_ttl = 8,
                .publish_period = ACCESS_PUBLISH_RESOLUTION_10S << ACCESS_PUBLISH_STEP_NUM_BITS | 24,
                .retransmit_count = 2,
                .retransmit_interval = 3,
                .model_id.sig.model_id = 0x4421,
            }
        },
        {
            .element_address = 0x4321,
            .publish_address = 0x1234,
            .state = {
                .appkey_index = 65,
                .publish_ttl = 100,
                .publish_period = ACCESS_PUBLISH_RESOLUTION_100MS << ACCESS_PUBLISH_STEP_NUM_BITS | 12,
                .retransmit_count = 4,
                .retransmit_interval = 5,
                .model_id.vendor.model_id = 0x1244,
                .model_id.vendor.company_id = 0x2288
            }
        }
    };


    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 24;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 33;
        access_model_id_t expected_id =
        {
            .model_id = sig_model ? messages[i].state.model_id.sig.model_id : messages[i].state.model_id.vendor.model_id,
            .company_id = sig_model ? ACCESS_COMPANY_ID_NONE : messages[i].state.model_id.vendor.company_id,
        };
        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t appkey_handle = 33331;
        dsm_appkey_index_to_appkey_handle_ExpectAndReturn(messages[i].state.appkey_index, appkey_handle);

        dsm_handle_t address_handle = 22115;
        access_model_publish_address_set_ExpectAndReturn(model_handle, address_handle, NRF_SUCCESS);
        access_model_publish_period_set_ExpectAndReturn(model_handle, ACCESS_PUBLISH_RESOLUTION_100MS, 0, NRF_SUCCESS);
        access_model_publish_period_set_ExpectAndReturn(model_handle, (access_publish_resolution_t) (messages[i].state.publish_period >> ACCESS_PUBLISH_STEP_NUM_BITS),
            messages[i].state.publish_period & (0xff >> (8 - ACCESS_PUBLISH_STEP_NUM_BITS)), NRF_SUCCESS);
        access_publish_retransmit_t publish_retransmit =
        {
            .count = messages[i].state.retransmit_count,
            .interval_steps = messages[i].state.retransmit_interval
        };
        access_model_publish_retransmit_set_ExpectAndReturn(model_handle, publish_retransmit, NRF_SUCCESS);
        access_model_publish_application_set_ExpectAndReturn(model_handle, appkey_handle, NRF_SUCCESS);
        access_model_publish_ttl_set_ExpectAndReturn(model_handle, messages[i].state.publish_ttl, NRF_SUCCESS);
        access_flash_config_store_Expect();
        /* The following functions are called when the server assembles the response packet: */
        access_model_id_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_id_get_IgnoreArg_p_model_id();
        access_model_id_get_ReturnThruPtr_p_model_id(&expected_id);

        // the first is for handle_config_model_publication_set
        access_model_publish_address_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_publish_address_get_IgnoreArg_p_address_handle();
        access_model_publish_address_get_ReturnThruPtr_p_address_handle(&address_handle);
        // the second is for send_publication_status
        access_model_publish_address_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_publish_address_get_IgnoreArg_p_address_handle();
        access_model_publish_address_get_ReturnThruPtr_p_address_handle(&address_handle);

        nrf_mesh_address_t publish_address = { .type = NRF_MESH_ADDRESS_TYPE_UNICAST, .value = messages[i].publish_address };
        // the first is for handle_config_model_publication_set
        dsm_address_get_ExpectAndReturn(address_handle, NULL, NRF_SUCCESS);
        dsm_address_get_IgnoreArg_p_address();
        dsm_address_get_ReturnThruPtr_p_address(&publish_address);
        // the second is for send_publication_status
        dsm_address_get_ExpectAndReturn(address_handle, NULL, NRF_SUCCESS);
        dsm_address_get_IgnoreArg_p_address();
        dsm_address_get_ReturnThruPtr_p_address(&publish_address);

        access_model_publish_application_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_publish_application_get_IgnoreArg_p_appkey_handle();
        access_model_publish_application_get_ReturnThruPtr_p_appkey_handle(&appkey_handle);

        mesh_key_index_t appkey_index = messages[i].state.appkey_index;

        dsm_appkey_handle_to_appkey_index_ExpectAndReturn(appkey_handle, NULL, NRF_SUCCESS);
        dsm_appkey_handle_to_appkey_index_IgnoreArg_p_index();
        dsm_appkey_handle_to_appkey_index_ReturnThruPtr_p_index(&appkey_index);

        access_model_publish_ttl_get_ExpectAndReturn(model_handle, NULL, NRF_SUCCESS);
        access_model_publish_ttl_get_IgnoreArg_p_ttl();
        access_model_publish_ttl_get_ReturnThruPtr_p_ttl(&messages[i].state.publish_ttl);

        access_publish_resolution_t publish_resolution = (access_publish_resolution_t) (messages[i].state.publish_period >> ACCESS_PUBLISH_STEP_NUM_BITS);
        uint8_t publish_steps = messages[i].state.publish_period & ~(0xff << ACCESS_PUBLISH_STEP_NUM_BITS);
        access_model_publish_period_get_ExpectAndReturn(model_handle, NULL, NULL, NRF_SUCCESS);
        access_model_publish_period_get_IgnoreArg_p_resolution();
        access_model_publish_period_get_IgnoreArg_p_step_number();
        access_model_publish_period_get_ReturnThruPtr_p_resolution(&publish_resolution);
        access_model_publish_period_get_ReturnThruPtr_p_step_number(&publish_steps);

        access_model_publish_retransmit_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_model_publish_retransmit_get_ReturnThruPtr_p_retransmit_params(&publish_retransmit);

        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_PUBLICATION_SET, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_publication_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_publication_status_t * p_reply = (const config_msg_publication_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(messages[i].publish_address, p_reply->publish_address);
        TEST_ASSERT_EQUAL(messages[i].state.appkey_index, p_reply->state.appkey_index);
        TEST_ASSERT_EQUAL(0, p_reply->state.credential_flag); /* TODO: When friendship is supported, this flag can be used. */
        TEST_ASSERT_EQUAL(0, p_reply->state.rfu);
        TEST_ASSERT_EQUAL(messages[i].state.publish_ttl, p_reply->state.publish_ttl);
        TEST_ASSERT_EQUAL_HEX(messages[i].state.publish_period, p_reply->state.publish_period);
        TEST_ASSERT_EQUAL(publish_retransmit.count, p_reply->state.retransmit_count);
        TEST_ASSERT_EQUAL(publish_retransmit.interval_steps, p_reply->state.retransmit_interval);

        if (sig_model)
        {
            TEST_ASSERT_EQUAL_UINT16(messages[i].state.model_id.sig.model_id, p_reply->state.model_id.sig.model_id);
        }
        else
        {
            TEST_ASSERT_EQUAL_UINT16(messages[i].state.model_id.vendor.model_id, p_reply->state.model_id.vendor.model_id);
            TEST_ASSERT_EQUAL_UINT16(messages[i].state.model_id.vendor.company_id, p_reply->state.model_id.vendor.company_id);
        }
    }
}

void test_netkey_add(void)
{
    const config_msg_netkey_add_update_t message =
        {
            .netkey_index = 4,
            .netkey = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 ,15 }
        };

    dsm_handle_t subnet_handle = 4;
    dsm_subnet_add_ExpectAndReturn(message.netkey_index, message.netkey, NULL, NRF_SUCCESS);
    dsm_subnet_add_IgnoreArg_p_subnet_handle();
    dsm_subnet_add_ReturnThruPtr_p_subnet_handle(&subnet_handle);

    send_message(CONFIG_OPCODE_NETKEY_ADD, (const uint8_t *) &message, sizeof(config_msg_netkey_add_update_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NETKEY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_netkey_status_t), m_previous_reply.length);

    const config_msg_netkey_status_t * p_reply = (const config_msg_netkey_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.netkey_index, p_reply->netkey_index);
}

void test_netkey_update(void)
{
    const config_msg_netkey_add_update_t message =
        {
            .netkey_index = 7,
            .netkey = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 }
        };

    dsm_net_key_index_to_subnet_handle_ExpectAndReturn(message.netkey_index, 2);
    dsm_subnet_update_ExpectAndReturn(2, message.netkey, NRF_SUCCESS);

    send_message(CONFIG_OPCODE_NETKEY_UPDATE, (const uint8_t *) &message, sizeof(config_msg_netkey_add_update_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NETKEY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_netkey_status_t), m_previous_reply.length);

    const config_msg_netkey_status_t * p_reply = (const config_msg_netkey_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.netkey_index, p_reply->netkey_index);
}

void test_netkey_delete(void)
{
    const config_msg_netkey_delete_t message =
    {
        .netkey_index = 12
    };

    dsm_net_key_index_to_subnet_handle_ExpectAndReturn(message.netkey_index, 9);

    dsm_appkey_get_all_ExpectAndReturn(9, NULL, NULL, NRF_SUCCESS);
    dsm_appkey_get_all_IgnoreArg_p_key_list();
    dsm_appkey_get_all_IgnoreArg_p_count();

    uint32_t bound_app_key_count = 1;
    dsm_appkey_get_all_ReturnThruPtr_p_count(&bound_app_key_count);
    dsm_appkey_index_to_appkey_handle_ExpectAnyArgsAndReturn(13);

    dsm_subnet_delete_ExpectAndReturn(9, NRF_SUCCESS);
    access_model_publication_by_appkey_stop_ExpectAndReturn(13, NRF_SUCCESS);

    send_message(CONFIG_OPCODE_NETKEY_DELETE, (const uint8_t *) &message, sizeof(config_msg_netkey_delete_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NETKEY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_netkey_status_t), m_previous_reply.length);

    const config_msg_netkey_status_t * p_reply = (const config_msg_netkey_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.netkey_index, p_reply->netkey_index);
}

void test_netkey_get(void)
{
    mesh_key_index_t netkey_indexes[] = { 2, 4, 8, 7, 31, 1609, 26 };
    uint8_t netkey_indexes_packed[] =
        {
            0x02, 0x40, 0x00, /* (2, 4) */
            0x08, 0x70, 0x00, /* (8, 7) */
            0x1f, 0x90, 0x64, /* (31, 1609) */
            0x1a, 0x00
        };

    uint32_t netkey_index_count = ARRAY_SIZE(netkey_indexes);

    DSM_SUBNET_GET_ALL_MOCK_SETUP(netkey_indexes, netkey_index_count, NRF_SUCCESS);
    dsm_subnet_get_all_StubWithCallback(dsm_subnet_get_all_mock);
    send_message(CONFIG_OPCODE_NETKEY_GET, NULL, 0); /* Message with no parameters. */

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NETKEY_LIST);
    TEST_ASSERT_EQUAL(sizeof(netkey_indexes_packed), m_previous_reply.length);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(netkey_indexes_packed, m_previous_reply.p_buffer, sizeof(netkey_indexes_packed));
}

void test_appkey_add(void)
{
    const uint16_t netkey_index = 42;
    const uint16_t appkey_index = 24;

    const uint8_t appkey[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };

    config_msg_appkey_add_t message;
    config_msg_key_index_24_set(&message.key_indexes, netkey_index, appkey_index);
    memcpy(message.appkey, appkey, sizeof(appkey));

    dsm_handle_t appkey_handle = 55;
    dsm_net_key_index_to_subnet_handle_ExpectAndReturn(netkey_index, 4);
    dsm_appkey_add_ExpectAndReturn(appkey_index, 4, message.appkey, NULL, NRF_SUCCESS);
    dsm_appkey_add_IgnoreArg_p_app_handle();
    dsm_appkey_add_ReturnThruPtr_p_app_handle(&appkey_handle);

    send_message(CONFIG_OPCODE_APPKEY_ADD, (const uint8_t *) &message, sizeof(config_msg_appkey_add_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_APPKEY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_appkey_status_t), m_previous_reply.length);

    const config_msg_appkey_status_t * p_reply = (const config_msg_appkey_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);

    uint16_t received_netkey, received_appkey;
    config_msg_key_index_24_get(&p_reply->key_indexes, &received_netkey, &received_appkey);
    TEST_ASSERT_EQUAL_UINT16(netkey_index, received_netkey);
    TEST_ASSERT_EQUAL_UINT16(appkey_index, received_appkey);
}

void test_appkey_update(void)
{
    const uint16_t netkey_index = 22;
    const uint16_t appkey_index = 88;

    const uint8_t appkey[] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };

    config_msg_appkey_update_t message;
    config_msg_key_index_24_set(&message.key_indexes, netkey_index, appkey_index);
    memcpy(message.appkey, appkey, sizeof(appkey));

    dsm_handle_t network_handle = 13;
    dsm_net_key_index_to_subnet_handle_ExpectAndReturn(netkey_index, network_handle);

    dsm_handle_t appkey_handle = 55;
    dsm_appkey_index_to_appkey_handle_ExpectAndReturn(appkey_index, appkey_handle);
    dsm_appkey_update_ExpectAndReturn(appkey_handle, message.appkey, NRF_SUCCESS);

    dsm_appkey_handle_to_subnet_handle_ExpectAndReturn(appkey_handle, NULL, NRF_SUCCESS);
    dsm_appkey_handle_to_subnet_handle_IgnoreArg_p_netkey_handle();
    dsm_appkey_handle_to_subnet_handle_ReturnThruPtr_p_netkey_handle(&network_handle);

    send_message(CONFIG_OPCODE_APPKEY_UPDATE, (const uint8_t *) &message, sizeof(config_msg_appkey_add_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_APPKEY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_appkey_status_t), m_previous_reply.length);

    const config_msg_appkey_status_t * p_reply = (const config_msg_appkey_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);

    uint16_t received_netkey, received_appkey;
    config_msg_key_index_24_get(&p_reply->key_indexes, &received_netkey, &received_appkey);
    TEST_ASSERT_EQUAL_UINT16(netkey_index, received_netkey);
    TEST_ASSERT_EQUAL_UINT16(appkey_index, received_appkey);
}

void test_appkey_delete(void)
{
    const uint16_t netkey_index = 118;
    const uint16_t appkey_index = 500;
    const dsm_handle_t appkey_handle = 1337;
    config_msg_appkey_delete_t message;
    config_msg_key_index_24_set(&message.key_indexes, netkey_index, appkey_index);
    dsm_appkey_index_to_appkey_handle_ExpectAndReturn(appkey_index, appkey_handle);

    dsm_handle_t network_handle = 13;
    dsm_net_key_index_to_subnet_handle_ExpectAndReturn(netkey_index, network_handle);

    dsm_appkey_handle_to_subnet_handle_ExpectAndReturn(appkey_handle, NULL, NRF_SUCCESS);
    dsm_appkey_handle_to_subnet_handle_IgnoreArg_p_netkey_handle();
    dsm_appkey_handle_to_subnet_handle_ReturnThruPtr_p_netkey_handle(&network_handle);

    access_model_publication_by_appkey_stop_ExpectAndReturn(appkey_handle, NRF_SUCCESS);
    dsm_appkey_delete_ExpectAndReturn(appkey_handle, NRF_SUCCESS);

    send_message(CONFIG_OPCODE_APPKEY_DELETE, (const uint8_t *) &message, sizeof(config_msg_appkey_delete_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_APPKEY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_appkey_status_t), m_previous_reply.length);

    const config_msg_appkey_status_t * p_reply = (const config_msg_appkey_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);

    uint16_t received_netkey, received_appkey;
    config_msg_key_index_24_get(&p_reply->key_indexes, &received_netkey, &received_appkey);
    TEST_ASSERT_EQUAL_UINT16(netkey_index, received_netkey);
    TEST_ASSERT_EQUAL_UINT16(appkey_index, received_appkey);
}

void test_appkey_get(void)
{
    const uint16_t netkey_index = 994;

    mesh_key_index_t appkey_indexes[] = { 2, 4, 8, 7, 31, 1609, 2002 };
    uint8_t appkey_indexes_packed[] =
        {
            0x02, 0x40, 0x00, /* (2, 4) */
            0x08, 0x70, 0x00, /* (8, 7) */
            0x1f, 0x90, 0x64, /* (31, 1609) */
            0xd2, 0x07        /* (2002) */
        };

    config_msg_appkey_get_t message;
    message.netkey_index = netkey_index & CONFIG_MSG_KEY_INDEX_12_MASK;

    dsm_handle_t subnet_handle = 1021;
    dsm_net_key_index_to_subnet_handle_ExpectAndReturn(netkey_index, subnet_handle);

    int appkey_count = ARRAY_SIZE(appkey_indexes);
    DSM_APPKEY_GET_ALL_MOCK_SETUP(subnet_handle, appkey_indexes, appkey_count, NRF_SUCCESS);
    dsm_appkey_get_all_StubWithCallback(dsm_appkey_get_all_mock);

    send_message(CONFIG_OPCODE_APPKEY_GET, (const uint8_t *) &message, sizeof(config_msg_appkey_get_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_APPKEY_LIST);
    TEST_ASSERT_EQUAL(sizeof(config_msg_appkey_list_t) + sizeof(appkey_indexes_packed), m_previous_reply.length);

    const config_msg_appkey_list_t * p_reply = (const config_msg_appkey_list_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);

    uint16_t received_netkey_index = p_reply->netkey_index;
    TEST_ASSERT_EQUAL_UINT16(netkey_index, received_netkey_index);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(appkey_indexes_packed, p_reply->packed_appkey_indexes, sizeof(appkey_indexes_packed));
}

void test_identity_get(void)
{
    const config_msg_identity_get_t message = { .netkey_index = 4 };

    send_message(CONFIG_OPCODE_NODE_IDENTITY_GET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NODE_IDENTITY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_identity_status_t), m_previous_reply.length);

    const config_msg_identity_status_t * p_reply = (const config_msg_identity_status_t *) m_previous_reply.p_buffer;

    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.netkey_index, p_reply->netkey_index);
    TEST_ASSERT_EQUAL(CONFIG_IDENTITY_STATE_UNSUPPORTED, p_reply->identity_state);
}

void test_identity_set(void)
{
    const config_msg_identity_set_t message =
        {
            .netkey_index = 5,
            .identity_state = CONFIG_IDENTITY_STATE_RUNNING
        };

    send_message(CONFIG_OPCODE_NODE_IDENTITY_SET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NODE_IDENTITY_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_identity_status_t), m_previous_reply.length);

    const config_msg_identity_status_t * p_reply = (const config_msg_identity_status_t *) m_previous_reply.p_buffer;

    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.netkey_index, p_reply->netkey_index);
    TEST_ASSERT_EQUAL(CONFIG_IDENTITY_STATE_UNSUPPORTED, p_reply->identity_state);
}

void test_model_app_bind_unbind(void)
{
    const config_msg_app_bind_unbind_t messages[2] =
    {
        {
            .element_address = 0x4321,
            .appkey_index = 4,
            .model_id.sig.model_id = 0x1453
        },
        {
            .element_address = 0x1234,
            .appkey_index = 5,
            .model_id.vendor.model_id = 0x6651,
            .model_id.vendor.company_id = 0x4449
        }
    };

    /* This loop runs 4 times, the first two testing the Model App Bind message, and the last two testing the
     * Model App Unbind message.
     */
    for (int i = 0; i < 4; ++i)
    {
        bool sig_model = !(i % 2);

        uint16_t element_index = 88;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i % 2].element_address, element_index);

        access_model_handle_t model_handle = 77;
        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t appkey_handle = 15;
        dsm_appkey_index_to_appkey_handle_ExpectAndReturn(messages[i % 2].appkey_index, appkey_handle);
        access_flash_config_store_Expect();
        if (i < 2)
        {
            access_model_application_bind_ExpectAndReturn(model_handle, appkey_handle, NRF_SUCCESS);
            send_message(CONFIG_OPCODE_MODEL_APP_BIND, (const uint8_t *) &messages[i % 2], sizeof(messages[i % 2]) - sig_model * sizeof(uint16_t));
        }
        else
        {
            access_model_application_unbind_ExpectAndReturn(model_handle, appkey_handle, NRF_SUCCESS);

            access_model_publish_application_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
            access_model_publish_application_get_ReturnThruPtr_p_appkey_handle(&appkey_handle);
            access_model_publication_stop_ExpectAndReturn(model_handle, NRF_SUCCESS);

            send_message(CONFIG_OPCODE_MODEL_APP_UNBIND, (const uint8_t *) &messages[i % 2], sizeof(messages[i % 2]) - sig_model * sizeof(uint16_t));
        }

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_APP_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_app_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_app_status_t * p_reply = (const config_msg_app_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);

        TEST_ASSERT_EQUAL(messages[i % 2].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(messages[i % 2].appkey_index, p_reply->appkey_index);
        TEST_ASSERT_EQUAL_MEMORY(&messages[i % 2].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_friend_get(void)
{
    send_message(CONFIG_OPCODE_FRIEND_GET, NULL, 0); /* Message with no parameters */

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_FRIEND_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(sizeof(config_msg_friend_status_t), m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_FRIEND_STATE_UNSUPPORTED, m_previous_reply.p_buffer[0]);
}

void test_friend_set(void)
{
    const config_msg_friend_set_t message = { .friend_state = CONFIG_FRIEND_STATE_SUPPORTED_ENABLED };
    send_message(CONFIG_OPCODE_FRIEND_SET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_FRIEND_STATUS);

    TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
    TEST_ASSERT_EQUAL(sizeof(config_msg_friend_status_t), m_previous_reply.length);
    TEST_ASSERT_EQUAL_UINT8(CONFIG_FRIEND_STATE_UNSUPPORTED, m_previous_reply.p_buffer[0]);
}

void test_key_refresh_phase_get(void)
{
    send_message(CONFIG_OPCODE_KEY_REFRESH_PHASE_GET, NULL, 42); /* Invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    nrf_mesh_key_refresh_phase_t expected_phases[] =
        { NRF_MESH_KEY_REFRESH_PHASE_0, NRF_MESH_KEY_REFRESH_PHASE_1, NRF_MESH_KEY_REFRESH_PHASE_2 };
    for (uint32_t i = 0; i < 3; ++i)
    {
        dsm_net_key_index_to_subnet_handle_ExpectAndReturn(4, 2);
        dsm_subnet_kr_phase_get_ExpectAndReturn(2, NULL, NRF_SUCCESS);
        dsm_subnet_kr_phase_get_IgnoreArg_p_phase();
        dsm_subnet_kr_phase_get_ReturnThruPtr_p_phase(&expected_phases[i]);

        config_msg_key_refresh_phase_get_t message = { .netkey_index = 4 };
        send_message(CONFIG_OPCODE_KEY_REFRESH_PHASE_GET, (const uint8_t *) &message, sizeof(message));
        TEST_ASSERT_TRUE(m_previous_reply_received);

        TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_key_refresh_phase_status_t), m_previous_reply.length);

        const config_msg_key_refresh_phase_status_t * p_reply = (const config_msg_key_refresh_phase_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(4, p_reply->netkey_index);
        TEST_ASSERT_EQUAL((uint8_t) expected_phases[i], p_reply->phase);
    }
}

void test_key_refresh_phase_set(void)
{
    send_message(CONFIG_OPCODE_KEY_REFRESH_PHASE_SET, NULL, 31); /* Invalid length */
    TEST_ASSERT_FALSE(m_previous_reply_received);

    const config_msg_key_index_12_t netkey_index = 22;
    const dsm_handle_t netkey_handle = 11;
    access_status_t reply_status = ACCESS_STATUS_UNSPECIFIED_ERROR;
    nrf_mesh_key_refresh_phase_t reply_phase = NRF_MESH_KEY_REFRESH_PHASE_0;

    for (uint8_t initial_phase = 0; initial_phase < 3; initial_phase++)
    {
        for (uint8_t transition = 0; transition < 4; transition++)
        {
            bool is_replayed = false;
            m_previous_reply_received = false;

            dsm_net_key_index_to_subnet_handle_ExpectAndReturn(netkey_index, netkey_handle);

            dsm_subnet_kr_phase_get_ExpectAndReturn(netkey_handle, NULL, NRF_SUCCESS);
            dsm_subnet_kr_phase_get_IgnoreArg_p_phase();
            dsm_subnet_kr_phase_get_ReturnThruPtr_p_phase((nrf_mesh_key_refresh_phase_t *) &initial_phase);

            switch (transition)
            {
                case 2:
                    if (initial_phase == NRF_MESH_KEY_REFRESH_PHASE_2 || initial_phase == NRF_MESH_KEY_REFRESH_PHASE_1)
                    {
                        if (initial_phase == NRF_MESH_KEY_REFRESH_PHASE_1)
                        {
                            dsm_subnet_update_swap_keys_ExpectAndReturn(netkey_handle, NRF_SUCCESS);
                        }
                        reply_status = ACCESS_STATUS_SUCCESS;
                        reply_phase = NRF_MESH_KEY_REFRESH_PHASE_2;
                        is_replayed = true;

                    }
                    break;
                case 3:
                    if (initial_phase != NRF_MESH_KEY_REFRESH_PHASE_3)
                    {
                        if (initial_phase != NRF_MESH_KEY_REFRESH_PHASE_0)
                        {
                            dsm_subnet_update_commit_ExpectAndReturn(netkey_handle, NRF_SUCCESS);
                        }
                        reply_status = ACCESS_STATUS_SUCCESS;
                        reply_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
                        is_replayed = true;
                    }
                    break;
            }

            const config_msg_key_refresh_phase_set_t message = { netkey_index, transition };
            send_message(CONFIG_OPCODE_KEY_REFRESH_PHASE_SET, (const uint8_t *) &message, sizeof(message));

            if (is_replayed)
            {
                TEST_ASSERT_TRUE(m_previous_reply_received);
                TEST_ASSERT_NOT_NULL(mp_previous_reply_buffer);
                VERIFY_REPLY_OPCODE(CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS);

                const config_msg_key_refresh_phase_status_t * p_reply = (const config_msg_key_refresh_phase_status_t *) m_previous_reply.p_buffer;
                TEST_ASSERT_EQUAL(reply_status, p_reply->status);
                TEST_ASSERT_EQUAL(reply_phase, p_reply->phase);
                TEST_ASSERT_EQUAL(netkey_index, p_reply->netkey_index);
            }
            else
            {
                TEST_ASSERT_FALSE(m_previous_reply_received);
            }
        }
    }
}

void test_subscription_add(void)
{
    const config_msg_subscription_add_del_owr_t messages[2] =
    {
        {
            .element_address = 0x1234,
            .address = 0xc345,
            .model_id.sig.model_id = 0x4211,
        },
        {
            .element_address = 0x4321,
            .address = 0xc432,
            .model_id.vendor.model_id = 0x1289,
            .model_id.vendor.company_id = 0x9922
        }
    };

    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 75;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 0x9962;

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t address_handle = 83;
        dsm_address_subscription_add_ExpectAndReturn(messages[i].address, NULL, NRF_SUCCESS);
        dsm_address_subscription_add_IgnoreArg_p_address_handle();
        dsm_address_subscription_add_ReturnThruPtr_p_address_handle(&address_handle);

        access_model_subscription_add_ExpectAndReturn(model_handle, address_handle, NRF_SUCCESS);
        access_flash_config_store_Expect();

        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_subscription_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_subscription_status_t * p_reply = (const config_msg_subscription_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(messages[i].address, p_reply->address);

        TEST_ASSERT_EQUAL_MEMORY(&messages[i].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_subscription_delete(void)
{
    const config_msg_subscription_add_del_owr_t messages[2] =
    {
        {
            .element_address = 0x1234,
            .address = 0xc345,
            .model_id.sig.model_id = 0x4211,
        },
        {
            .element_address = 0x4321,
            .address = 0xc432,
            .model_id.vendor.model_id = 0x1289,
            .model_id.vendor.company_id = 0x9922
        }
    };

    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 76;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 0x9962;

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t address_handle = 923;

        dsm_address_handle_get_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
        dsm_address_handle_get_IgnoreArg_p_address(); /* The value this parameter points to should probably also be checked. */
        dsm_address_handle_get_IgnoreArg_p_address_handle();
        dsm_address_handle_get_ReturnThruPtr_p_address_handle(&address_handle);

        access_model_subscription_remove_ExpectAndReturn(model_handle, address_handle, NRF_SUCCESS);
        dsm_address_subscription_remove_ExpectAndReturn(address_handle, NRF_SUCCESS);
        access_flash_config_store_Expect();

        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_subscription_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_subscription_status_t * p_reply = (const config_msg_subscription_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(messages[i].address, p_reply->address);

        TEST_ASSERT_EQUAL_MEMORY(&messages[i].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_subscription_overwrite(void)
{
    const config_msg_subscription_add_del_owr_t messages[2] =
    {
        {
            .element_address = 0x1234,
            .address = 0xc345,
            .model_id.sig.model_id = 0x4211,
        },
        {
            .element_address = 0x4321,
            .address = 0xc432,
            .model_id.vendor.model_id = 0x1289,
            .model_id.vendor.company_id = 0x9922
        }
    };

    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 77;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 0x9962;

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t subscriptions[] = { 2, 98, 14 };
        uint16_t subscription_count = ARRAY_SIZE(subscriptions);
        access_model_subscriptions_get_StubWithCallback(access_model_subscriptions_get_mock);
        ACCESS_MODEL_SUBSCRIPTIONS_GET_MOCK_SETUP(model_handle, subscriptions,
                                                  subscription_count, NRF_SUCCESS);

        for (int j = 0; j < subscription_count; ++j)
        {
            access_model_subscription_remove_ExpectAndReturn(model_handle, subscriptions[j], NRF_SUCCESS);
            dsm_address_subscription_remove_ExpectAndReturn(subscriptions[j], NRF_SUCCESS);
        }

        dsm_handle_t address_handle = 89;
        dsm_address_subscription_add_ExpectAndReturn(messages[i].address, NULL, NRF_SUCCESS);
        dsm_address_subscription_add_IgnoreArg_p_address_handle();
        dsm_address_subscription_add_ReturnThruPtr_p_address_handle(&address_handle);
        access_model_subscription_add_ExpectAndReturn(model_handle, address_handle, NRF_SUCCESS);
        access_flash_config_store_Expect();

        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_subscription_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_subscription_status_t * p_reply = (const config_msg_subscription_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(messages[i].address, p_reply->address);
        TEST_ASSERT_EQUAL_MEMORY(&messages[i].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_subscription_delete_all(void)
{
    const config_msg_subscription_delete_all_t messages[2] =
    {
        {
            .element_address = 0x2277,
            .model_id.sig.model_id = 0x1425,
        },
        {
            .element_address = 0x3366,
            .model_id.vendor.model_id = 0x3647,
            .model_id.vendor.company_id = 0x8675
        }
    };

    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 77;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 0x9962;

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t subscriptions[] = { 2, 98, 14 };
        uint16_t subscription_count = ARRAY_SIZE(subscriptions);
        access_model_subscriptions_get_StubWithCallback(access_model_subscriptions_get_mock);
        ACCESS_MODEL_SUBSCRIPTIONS_GET_MOCK_SETUP(model_handle, subscriptions,
                                                  subscription_count, NRF_SUCCESS);

        for (int j = 0; j < subscription_count; ++j)
        {
            access_model_subscription_remove_ExpectAndReturn(model_handle, subscriptions[j], NRF_SUCCESS);
            dsm_address_subscription_remove_ExpectAndReturn(subscriptions[j], NRF_SUCCESS);
        }

        access_flash_config_store_Expect();
        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_subscription_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_subscription_status_t * p_reply = (const config_msg_subscription_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(NRF_MESH_ADDR_UNASSIGNED, p_reply->address);
        TEST_ASSERT_EQUAL_MEMORY(&messages[i].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_subscription_virtual_add(void)
{
    const config_msg_subscription_virtual_add_del_owr_t messages[] =
    {
        {
            .element_address = 0x1234,
            .virtual_uuid = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
            .model_id.sig.model_id = 0x9182,
        },
        {
            .element_address = 0x4321,
            .virtual_uuid = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 },
            .model_id.vendor.model_id = 0x8765,
            .model_id.vendor.company_id = 0x4321
        }
    };

    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 2;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 0x0013;

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t address_handle = 442;
        dsm_address_subscription_virtual_add_ExpectWithArrayAndReturn(messages[i].virtual_uuid, NRF_MESH_UUID_SIZE, NULL, 0, NRF_SUCCESS);
        dsm_address_subscription_virtual_add_IgnoreArg_p_address_handle();
        dsm_address_subscription_virtual_add_ReturnThruPtr_p_address_handle(&address_handle);

        access_model_subscription_add_ExpectAndReturn(model_handle, address_handle, NRF_SUCCESS);
        access_flash_config_store_Expect();

        nrf_mesh_address_t address = { .type = NRF_MESH_ADDRESS_TYPE_VIRTUAL, .value = 0x8331 /* Not a real address */ };
        dsm_address_get_ExpectAndReturn(address_handle, NULL, NRF_SUCCESS);
        dsm_address_get_IgnoreArg_p_address();
        dsm_address_get_ReturnThruPtr_p_address(&address);

        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_subscription_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_subscription_status_t * p_reply = (const config_msg_subscription_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(0x8331, p_reply->address);
        TEST_ASSERT_EQUAL_MEMORY(&messages[i].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_subscription_virtual_overwrite(void)
{
    const config_msg_subscription_virtual_add_del_owr_t messages[] =
    {
        {
            .element_address = 0x1234,
            .virtual_uuid = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
            .model_id.sig.model_id = 0x9182
        },
        {
            .element_address = 0x4321,
            .virtual_uuid = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 },
            .model_id.vendor.model_id = 0x8765,
            .model_id.vendor.company_id = 0x4321
        }
    };

    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 2;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 0x0013;

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        dsm_handle_t subscriptions[] = { 2, 98, 14 };
        uint16_t subscription_count = ARRAY_SIZE(subscriptions);
        access_model_subscriptions_get_StubWithCallback(access_model_subscriptions_get_mock);
        ACCESS_MODEL_SUBSCRIPTIONS_GET_MOCK_SETUP(model_handle, subscriptions,
                                                  subscription_count, NRF_SUCCESS);

        for (int j = 0; j < subscription_count; ++j)
        {
            access_model_subscription_remove_ExpectAndReturn(model_handle, subscriptions[j], NRF_SUCCESS);
            dsm_address_subscription_remove_ExpectAndReturn(subscriptions[j], NRF_SUCCESS);
        }

        dsm_handle_t address_handle = 442;
        dsm_address_subscription_virtual_add_ExpectWithArrayAndReturn(messages[i].virtual_uuid, NRF_MESH_UUID_SIZE, NULL, 0, NRF_SUCCESS);
        dsm_address_subscription_virtual_add_IgnoreArg_p_address_handle();
        dsm_address_subscription_virtual_add_ReturnThruPtr_p_address_handle(&address_handle);

        access_model_subscription_add_ExpectAndReturn(model_handle, address_handle, NRF_SUCCESS);
        access_flash_config_store_Expect();

        nrf_mesh_address_t address = { .type = NRF_MESH_ADDRESS_TYPE_VIRTUAL, .value = 0x8331 /* Not a real address */ };
        dsm_address_get_ExpectAndReturn(address_handle, NULL, NRF_SUCCESS);
        dsm_address_get_IgnoreArg_p_address();
        dsm_address_get_ReturnThruPtr_p_address(&address);

        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_subscription_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_subscription_status_t * p_reply = (const config_msg_subscription_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(0x8331, p_reply->address);
        TEST_ASSERT_EQUAL_MEMORY(&messages[i].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_subscription_virtual_delete(void)
{
    const config_msg_subscription_virtual_add_del_owr_t messages[] =
    {
        {
            .element_address = 0x1234,
            .virtual_uuid = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
            .model_id.sig.model_id = 0x9182,
        },
        {
            .element_address = 0x4321,
            .virtual_uuid = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 },
            .model_id.vendor.model_id = 0x8765,
            .model_id.vendor.company_id = 0x4321
        }
    };

    for (int i = 0; i < 2; ++i)
    {
        bool sig_model = !i;

        uint16_t element_index = 2;
        EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(messages[i].element_address, element_index);

        access_model_handle_t model_handle = 0x0013;

        access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        access_handle_get_ReturnThruPtr_p_handle(&model_handle);

        uint16_t mock_address = 0x8998;
        nrf_mesh_keygen_virtual_address_ExpectAndReturn(messages[i].virtual_uuid, NULL, NRF_SUCCESS);
        nrf_mesh_keygen_virtual_address_IgnoreArg_p_address();
        nrf_mesh_keygen_virtual_address_ReturnThruPtr_p_address(&mock_address);

        dsm_handle_t address_handle = 2665;
        dsm_address_handle_get_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
        dsm_address_handle_get_IgnoreArg_p_address();
        dsm_address_handle_get_IgnoreArg_p_address_handle();
        dsm_address_handle_get_ReturnThruPtr_p_address_handle(&address_handle);

        access_model_subscription_remove_ExpectAndReturn(model_handle, address_handle, NRF_SUCCESS);
        access_flash_config_store_Expect();
        dsm_address_subscription_remove_ExpectAndReturn(address_handle, NRF_SUCCESS);

        m_previous_reply_received = false;
        send_message(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE, (const uint8_t *) &messages[i], sizeof(messages[i]) - sig_model * sizeof(uint16_t));

        TEST_ASSERT_TRUE(m_previous_reply_received);
        VERIFY_REPLY_OPCODE(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS);
        TEST_ASSERT_EQUAL(sizeof(config_msg_subscription_status_t) - sig_model * sizeof(uint16_t), m_previous_reply.length);

        const config_msg_subscription_status_t * p_reply = (const config_msg_subscription_status_t *) m_previous_reply.p_buffer;
        TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
        TEST_ASSERT_EQUAL(messages[i].element_address, p_reply->element_address);
        TEST_ASSERT_EQUAL(0x8998, p_reply->address);
        TEST_ASSERT_EQUAL_MEMORY(&messages[i].model_id, &p_reply->model_id, sizeof(config_model_id_t) - sig_model * sizeof(uint16_t));
    }
}

void test_sig_model_subscription_get(void)
{
    const config_msg_model_subscription_get_t message =
    {
        .element_address = 0x6411,
        .model_id.sig.model_id = 0x1144
    };

    uint16_t element_index = 2;
    EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(message.element_address, element_index);

    access_model_handle_t model_handle = 0x9c21;
    access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_handle_get_IgnoreArg_p_handle();
    access_handle_get_ReturnThruPtr_p_handle(&model_handle);

    dsm_handle_t subscriptions[] = { 1, 22, 882, 31771 };
    uint16_t subscription_count = ARRAY_SIZE(subscriptions);
    access_model_subscriptions_get_StubWithCallback(access_model_subscriptions_get_mock);
    ACCESS_MODEL_SUBSCRIPTIONS_GET_MOCK_SETUP(model_handle, subscriptions,
                                              subscription_count, NRF_SUCCESS);

    nrf_mesh_address_t addr[sizeof(subscriptions)];
    for (uint8_t itr = 0; itr < ARRAY_SIZE(subscriptions); itr++)
    {
        addr[itr].type = NRF_MESH_ADDRESS_TYPE_GROUP;
        addr[itr].value = subscriptions[itr];
        dsm_address_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        dsm_address_get_ReturnThruPtr_p_address(&addr[itr]);
    }

    send_message(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET, (const uint8_t *) &message, sizeof(message) - sizeof(uint16_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST);
    TEST_ASSERT_EQUAL(sizeof(config_msg_sig_model_subscription_list_t) + subscription_count * sizeof(uint16_t), m_previous_reply.length);

    const config_msg_sig_model_subscription_list_t * p_reply = (const config_msg_sig_model_subscription_list_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.element_address, p_reply->element_address);
    TEST_ASSERT_EQUAL(message.model_id.sig.model_id, p_reply->sig_model_id);
    for (uint16_t i = 0; i < subscription_count; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(subscriptions[i], p_reply->subscriptions[i]);
    }
}

void test_vendor_model_subscription_get(void)
{
    const config_msg_model_subscription_get_t message =
    {
        .element_address = 0x1144,
        .model_id.vendor.model_id = 0x4321,
        .model_id.vendor.company_id = 0x1234
    };

    uint16_t element_index = 5;
    EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(message.element_address, element_index);

    access_model_handle_t model_handle = 0x9c2f;
    access_model_id_t model_id =
    {
        .model_id = message.model_id.vendor.model_id,
        .company_id = message.model_id.vendor.company_id
    };
    access_handle_get_ExpectAndReturn(element_index, model_id, NULL, NRF_SUCCESS);
    access_handle_get_IgnoreArg_p_handle();
    access_handle_get_ReturnThruPtr_p_handle(&model_handle);

    dsm_handle_t subscriptions[] = { 1, 22, 882, 31771 };
    uint16_t subscription_count = ARRAY_SIZE(subscriptions);
    access_model_subscriptions_get_StubWithCallback(access_model_subscriptions_get_mock);
    ACCESS_MODEL_SUBSCRIPTIONS_GET_MOCK_SETUP(model_handle, subscriptions,
                                              subscription_count, NRF_SUCCESS);

    nrf_mesh_address_t addr[sizeof(subscriptions)];
    for (uint8_t itr = 0; itr < ARRAY_SIZE(subscriptions); itr++)
    {
        addr[itr].type = NRF_MESH_ADDRESS_TYPE_GROUP;
        addr[itr].value = subscriptions[itr];
        dsm_address_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
        dsm_address_get_ReturnThruPtr_p_address(&addr[itr]);
    }

    send_message(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST);
    TEST_ASSERT_EQUAL(sizeof(config_msg_vendor_model_subscription_list_t) + subscription_count * sizeof(uint16_t), m_previous_reply.length);

    const config_msg_vendor_model_subscription_list_t * p_reply = (const config_msg_vendor_model_subscription_list_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.element_address, p_reply->element_address);
    TEST_ASSERT_EQUAL(message.model_id.vendor.model_id, p_reply->vendor_model_id);
    TEST_ASSERT_EQUAL(message.model_id.vendor.company_id, p_reply->vendor_company_id);
    for (uint16_t i = 0; i < subscription_count; ++i)
    {
        TEST_ASSERT_EQUAL_UINT16(subscriptions[i], p_reply->subscriptions[i]);
    }
}

void test_sig_model_app_get(void)
{
    const config_msg_model_app_get_t message =
    {
        .element_address = 0x1111,
        .model_id.sig.model_id = 0x2222
    };

    uint16_t element_index = 7;
    EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(message.element_address, element_index);

    access_model_handle_t model_handle = 0x2ffa;
    access_handle_get_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    access_handle_get_ReturnThruPtr_p_handle(&model_handle);

    dsm_handle_t appkey_handles[] = { 1, 2, 3, 4, 5 };
    uint16_t appkey_handle_count = ARRAY_SIZE(appkey_handles);
    access_model_applications_get_StubWithCallback(access_model_applications_get_mock);
    ACCESS_MODEL_APPLICATIONS_GET_MOCK_SETUP(model_handle, appkey_handles, appkey_handle_count, NRF_SUCCESS);

    mesh_key_index_t appkey_indexes[] = { 11, 12, 13, 14, 15 };
    for (uint16_t i = 0; i < appkey_handle_count; ++i)
    {
        dsm_appkey_handle_to_appkey_index_ExpectAndReturn(appkey_handles[i], NULL, NRF_SUCCESS);
        dsm_appkey_handle_to_appkey_index_IgnoreArg_p_index();
        dsm_appkey_handle_to_appkey_index_ReturnThruPtr_p_index(&appkey_indexes[i]);
    }

    send_message(CONFIG_OPCODE_SIG_MODEL_APP_GET, (const uint8_t *) &message, sizeof(message) - sizeof(uint16_t));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_SIG_MODEL_APP_LIST);
    TEST_ASSERT_EQUAL(sizeof(config_msg_sig_model_app_list_t) + 8 /* 5 x 12-bit packed key index = 8 byte */, m_previous_reply.length);

    const config_msg_sig_model_app_list_t * p_reply = (const config_msg_sig_model_app_list_t *) m_previous_reply.p_buffer;

    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.element_address, p_reply->element_address);
    TEST_ASSERT_EQUAL(message.model_id.sig.model_id, p_reply->sig_model_id);

    const uint8_t appkey_indexes_packed[] = {
        0x0b, 0xc0, 0x00, /* (11, 12) */
        0x0d, 0xe0, 0x00, /* (13, 14) */
        0x0f, 0x00        /* (15,   ) */
    };
    TEST_ASSERT_EQUAL_MEMORY(appkey_indexes_packed, p_reply->key_indexes, sizeof(appkey_indexes_packed));
}

void test_vendor_model_app_get(void)
{
    const config_msg_model_app_get_t message =
        {
            .element_address = 0x2222,
            .model_id.vendor.model_id = 0x3333,
            .model_id.vendor.company_id = 0x4444,
        };

    uint16_t element_index = 8;
    EXPECT_DSM_LOCAL_UNICAST_ADDRESSES_GET(message.element_address, element_index);

    access_model_handle_t model_handle = 0x9ffc;
    access_model_id_t model_id =
        {
            .model_id = message.model_id.vendor.model_id,
            .company_id = message.model_id.vendor.company_id
        };
    access_handle_get_ExpectAndReturn(element_index, model_id, NULL, NRF_SUCCESS);
    access_handle_get_IgnoreArg_p_handle();
    access_handle_get_ReturnThruPtr_p_handle(&model_handle);

    dsm_handle_t appkey_handles[] = { 1, 2, 3, 4, 5 };
    uint16_t appkey_handle_count = ARRAY_SIZE(appkey_handles);
    access_model_applications_get_StubWithCallback(access_model_applications_get_mock);
    ACCESS_MODEL_APPLICATIONS_GET_MOCK_SETUP(model_handle, appkey_handles, appkey_handle_count, NRF_SUCCESS);

    mesh_key_index_t appkey_indexes[] = { 11, 12, 13, 14, 15 };
    for (uint16_t i = 0; i < appkey_handle_count; ++i)
    {
        dsm_appkey_handle_to_appkey_index_ExpectAndReturn(appkey_handles[i], NULL, NRF_SUCCESS);
        dsm_appkey_handle_to_appkey_index_IgnoreArg_p_index();
        dsm_appkey_handle_to_appkey_index_ReturnThruPtr_p_index(&appkey_indexes[i]);
    }

    send_message(CONFIG_OPCODE_VENDOR_MODEL_APP_GET, (const uint8_t *) &message, sizeof(message));

    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_VENDOR_MODEL_APP_LIST);
    TEST_ASSERT_EQUAL(sizeof(config_msg_vendor_model_app_list_t) + 8 /* 5 x 12-bit packed key index = 8 byte */, m_previous_reply.length);

    const config_msg_vendor_model_app_list_t * p_reply = (const config_msg_vendor_model_app_list_t *) m_previous_reply.p_buffer;

    TEST_ASSERT_EQUAL(ACCESS_STATUS_SUCCESS, p_reply->status);
    TEST_ASSERT_EQUAL(message.element_address, p_reply->element_address);
    TEST_ASSERT_EQUAL(message.model_id.vendor.model_id, p_reply->vendor_model_id);
    TEST_ASSERT_EQUAL(message.model_id.vendor.company_id, p_reply->vendor_company_id);

    const uint8_t appkey_indexes_packed[] = {
        0x0b, 0xc0, 0x00, /* (11, 12) */
        0x0d, 0xe0, 0x00, /* (13, 14) */
        0x0f, 0x00        /* (15,   ) */
    };
    TEST_ASSERT_EQUAL_MEMORY(appkey_indexes_packed, p_reply->key_indexes, sizeof(appkey_indexes_packed));
}

void test_network_transmit_set(void)
{
    const uint8_t TRANSMIT_COUNT = 2;
    const uint8_t INTERVAL_STEPS = 3;
    const uint32_t INTERVAL_MS = 10 * (INTERVAL_STEPS + 1);

    const config_msg_network_transmit_set_t message =
        {
            .network_transmit_count = TRANSMIT_COUNT,
            .network_transmit_interval_steps = INTERVAL_STEPS
        };

    nrf_mesh_opt_t expected_optval_retransmits = { .len = 4, .opt.val = TRANSMIT_COUNT + 1 };
    nrf_mesh_opt_set_ExpectWithArrayAndReturn(NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT,
                                              &expected_optval_retransmits, 1, NRF_SUCCESS);

    /* 10 ms * (steps + 1) according to Mesh Profile spec. 4.2.19.2. */
    nrf_mesh_opt_t expected_optval_interval = { .len = 4, .opt.val = INTERVAL_MS};
    nrf_mesh_opt_set_ExpectWithArrayAndReturn(NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS,
                                              &expected_optval_interval, 1, NRF_SUCCESS);

    /* The server reads out the expected state back again when replying. */
    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&expected_optval_retransmits);

    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&expected_optval_interval);

    send_message(CONFIG_OPCODE_NETWORK_TRANSMIT_SET, (const uint8_t *) &message, sizeof(message));
    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_network_transmit_status_t), m_previous_reply.length);

    const config_msg_network_transmit_status_t * p_reply =
        (const config_msg_network_transmit_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(message.network_transmit_count, p_reply->network_transmit_count);
    TEST_ASSERT_EQUAL(message.network_transmit_interval_steps, p_reply->network_transmit_interval_steps);
}

void test_network_transmit_get(void)
{
    const uint8_t TRANSMIT_COUNT = 2;
    const uint8_t INTERVAL_STEPS = 3;
    /* 10 ms * (steps + 1) according to Mesh Profile spec. 4.2.19.2. */
    const uint32_t INTERVAL_MS = 10 * (INTERVAL_STEPS + 1);

    nrf_mesh_opt_t expected_optval_retransmits = { .len = 4, .opt.val = TRANSMIT_COUNT + 1 };
    nrf_mesh_opt_t expected_optval_interval    = { .len = 4, .opt.val = INTERVAL_MS};
    /* The server reads out the expected state back again when replying. */
    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_NETWORK_TRANSMIT_COUNT, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&expected_optval_retransmits);

    nrf_mesh_opt_get_ExpectAndReturn(NRF_MESH_OPT_NET_NETWORK_TRANSMIT_INTERVAL_MS, NULL, NRF_SUCCESS);
    nrf_mesh_opt_get_IgnoreArg_p_opt();
    nrf_mesh_opt_get_ReturnThruPtr_p_opt(&expected_optval_interval);

    send_message(CONFIG_OPCODE_NETWORK_TRANSMIT_GET, NULL, 0);
    TEST_ASSERT_TRUE(m_previous_reply_received);
    VERIFY_REPLY_OPCODE(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS);
    TEST_ASSERT_EQUAL(sizeof(config_msg_network_transmit_status_t), m_previous_reply.length);

    const config_msg_network_transmit_status_t * p_reply =
        (const config_msg_network_transmit_status_t *) m_previous_reply.p_buffer;
    TEST_ASSERT_EQUAL(TRANSMIT_COUNT, p_reply->network_transmit_count);
    TEST_ASSERT_EQUAL(INTERVAL_STEPS, p_reply->network_transmit_interval_steps);
}
