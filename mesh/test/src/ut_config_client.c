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

/*lint -e64 Lint incorrectly flags type mismatches in struct initializers. */

#include "config_client.h"
#include "config_messages.h"

#include <stdbool.h>

#include "cmock.h"
#include "unity.h"

#include "packed_index_list.h"
#include "test_assert.h"

#include "access_mock.h"
#include "access_reliable_mock.h"
#include "access_config_mock.h"
#include "nrf_mesh_mock.h"

/*****************************************************************************
 * Defines
 *****************************************************************************/

#define APPKEY {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
#define NETKEY {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
#define VIRTUAL_UUID {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}
#define CONFIG_ELEMENT_INDEX (0)
#define ERROR_CODE (0xCAFEBABE)

#define EXPECT_TX(OPCODE, DATA, LENGTH)                                 \
    do                                                                  \
    {                                                                   \
        TEST_ASSERT_MESSAGE(!m_expect_ack, "Already waiting for an ACK"); \
        TEST_ASSERT_MESSAGE(m_buffer.free, "Buffer not free");          \
        if (LENGTH > 0)                                                 \
        {                                                               \
            memcpy(m_buffer.buffer, DATA, LENGTH);                      \
        }                                                               \
        m_buffer.length = LENGTH;                                       \
        m_buffer.opcode = OPCODE;                                       \
        m_buffer.free = false;                                          \
        m_expect_ack = true;                                            \
        access_model_reliable_publish_StubWithCallback(send_reliable_cb); \
    } while (0)

#define EXPECT_ACK(OPCODE, DATA, LENGTH)                                \
    do                                                                  \
    {                                                                   \
        TEST_ASSERT_MESSAGE(m_buffer.free, "Buffer not free");          \
        TEST_ASSERT_MESSAGE(!m_expect_timeout, "Already waiting for a timeout"); \
        TEST_ASSERT_MESSAGE(!m_expect_cancelled, "Already waiting for a cancel"); \
        if (LENGTH > 0)                                                 \
        {                                                               \
            memcpy(m_buffer.buffer, DATA, LENGTH);                      \
        }                                                               \
        m_buffer.length = LENGTH;                                       \
        m_buffer.opcode = OPCODE;                                       \
        m_buffer.free = false;                                          \
    } while (0)

#define EXPECT_TIMEOUT()                                                \
    do                                                                  \
    {                                                                   \
        TEST_ASSERT_MESSAGE(m_expect_ack, "Not TX-ing a message. Did you forget EXPECT_TX()?"); \
        TEST_ASSERT_MESSAGE(!m_expect_timeout, "Already waiting for a timeout"); \
        TEST_ASSERT_MESSAGE(!m_expect_cancelled, "Already waiting for a cancel"); \
        m_expect_timeout = true;                                        \
        m_expect_ack = false;                                           \
    } while (0)

#define EXPECT_CANCELLED()                                              \
    do                                                                  \
    {                                                                   \
        TEST_ASSERT_MESSAGE(m_expect_ack, "Not TX-ing a message. Did you forget EXPECT_TX()?"); \
        TEST_ASSERT_MESSAGE(!m_expect_timeout, "Already waiting for a timeout"); \
        TEST_ASSERT_MESSAGE(!m_expect_cancelled, "Already waiting for a cancel"); \
        m_expect_cancelled = true;                                      \
        m_expect_ack = false;                                           \
    } while (0)

/*****************************************************************************
 * Static data
 *****************************************************************************/

static uint32_t m_retval = NRF_SUCCESS;
static access_model_handle_t m_handle;
static access_model_add_params_t m_model_params;
static struct
{
    bool free;
    uint32_t length;
    config_opcode_t opcode;
    uint8_t buffer[NRF_MESH_SEG_PAYLOAD_SIZE_MAX];
} m_buffer;

static access_reliable_cb_t m_reliable_cb;
static bool m_expect_timeout;
static bool m_expect_cancelled;
static bool m_expect_ack;
static int m_pacman_refcount;

void config_client_reset(void);

/*****************************************************************************
 * CMock Callbacks
 *****************************************************************************/

static uint32_t alloc_cb(const access_model_add_params_t * p_params, access_model_handle_t * p_handle, int num_calls)
{
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_params->model_id.company_id);
    TEST_ASSERT_EQUAL(0x0001, p_params->model_id.model_id);
    TEST_ASSERT(p_params->opcode_count > 0);
    TEST_ASSERT_EQUAL(CONFIG_ELEMENT_INDEX, p_params->element_index);
    TEST_ASSERT_NOT_NULL(p_params->p_opcode_handlers);
    *p_handle = m_handle;
    memcpy(&m_model_params, p_params, sizeof(access_model_add_params_t));
    return m_retval;
}

static uint32_t send_reliable_cb(const access_reliable_t * p_reliable, int num_calls)
{
    TEST_ASSERT_NOT_NULL(p_reliable);
    TEST_ASSERT_EQUAL(m_handle, p_reliable->model_handle);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reliable->message.opcode.company_id);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, p_reliable->reply_opcode.company_id);
    TEST_ASSERT_EQUAL(m_buffer.opcode, p_reliable->message.opcode.opcode);

    if (p_reliable->message.p_buffer == NULL)
    {
        TEST_ASSERT_EQUAL(0, p_reliable->message.length);
    }
    else
    {
        TEST_ASSERT(!m_buffer.free);
        TEST_ASSERT(p_reliable->message.length > 0);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(m_buffer.buffer, p_reliable->message.p_buffer, m_buffer.length);
    }
    m_reliable_cb = p_reliable->status_cb;

    /* Clear the callback. */
    access_model_reliable_publish_StubWithCallback(NULL);
    m_buffer.free = true;
    return m_retval;
}

static void event_cb(config_client_event_type_t event_type, const config_client_event_t * p_evt, uint16_t length)
{
    switch (event_type)
    {
        case CONFIG_CLIENT_EVENT_TYPE_TIMEOUT:
            TEST_ASSERT(m_expect_timeout);
            TEST_ASSERT_NULL(p_evt);
            m_expect_timeout = false;
            break;

        case CONFIG_CLIENT_EVENT_TYPE_CANCELLED:
            TEST_ASSERT(m_expect_cancelled);
            TEST_ASSERT_NULL(p_evt);
            m_expect_cancelled = false;
            break;

        case CONFIG_CLIENT_EVENT_TYPE_MSG:
            TEST_ASSERT(m_expect_ack);
            TEST_ASSERT(!m_buffer.free);
            TEST_ASSERT_NOT_NULL(p_evt);
            TEST_ASSERT_EQUAL(m_buffer.opcode, p_evt->opcode);
            TEST_ASSERT_EQUAL(m_buffer.length, length);
            if (m_buffer.length > 0)
            {
                TEST_ASSERT_EQUAL_MEMORY(m_buffer.buffer, p_evt->p_msg, m_buffer.length);
            }
            m_expect_ack = false;
            m_buffer.free = true;
            break;

        default:
            TEST_FAIL_MESSAGE("Unknown event type");
            break;
    }

}

/*****************************************************************************
 * Helper functions
 *****************************************************************************/

static void __setup(void)
{
    access_model_add_StubWithCallback(alloc_cb);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_init(event_cb));
}

/*lint -ecall(569, send_ack) Loss of precision when converting size_t (from sizeof()) to uint16_t for the length field. */
static void send_ack(config_opcode_t opcode, const uint8_t * p_data, uint16_t length)
{
    access_opcode_handler_cb_t p_cb = NULL;
    for (uint32_t i = 0; i < m_model_params.opcode_count; ++i)
    {
        if (m_model_params.p_opcode_handlers[i].opcode.opcode == opcode)
        {
            p_cb = m_model_params.p_opcode_handlers[i].handler;
            break;
        }
    }

    TEST_ASSERT_NOT_NULL_MESSAGE(p_cb, "Could not find opcode handler");
    access_message_rx_t msg = {{0}};
    msg.opcode.opcode = opcode;
    msg.opcode.company_id = ACCESS_COMPANY_ID_NONE;
    msg.p_data = p_data;
    msg.length = length;
    p_cb(m_handle, &msg, m_model_params.p_args);
}

/*****************************************************************************
 * Setup
 *****************************************************************************/

void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    access_reliable_mock_Init();
    nrf_mesh_mock_Init();
    memset(&m_buffer, 0, sizeof(m_buffer));
    m_buffer.free = true;
    m_handle = 0;
    m_retval = NRF_SUCCESS;
    memset(&m_model_params, 0, sizeof(m_model_params));
    nrf_mesh_unique_token_get_IgnoreAndReturn((nrf_mesh_tx_token_t)0x55AA55AAul);
    m_expect_timeout = false;
    m_expect_cancelled = false;
    m_expect_ack = false;
    m_pacman_refcount = 0;
}

void tearDown(void)
{
    /* Nothing unhandled left. */
    TEST_ASSERT_MESSAGE(!m_expect_timeout, "TIMEOUT event not sent");
    TEST_ASSERT_MESSAGE(!m_expect_cancelled, "CANCELLED event not sent");
    TEST_ASSERT_MESSAGE(!m_expect_ack, "ACK not sent");
    TEST_ASSERT_MESSAGE(m_buffer.free, "Buffer not freed (did we lose a TX?)");
    TEST_ASSERT_EQUAL(0, m_pacman_refcount);
    config_client_reset();
    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    access_reliable_mock_Verify();
    access_reliable_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
}

/*****************************************************************************
 * Tests
 *****************************************************************************/

void test_invalid_state(void)
{
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_server_set(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_server_bind(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_composition_data_get(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_appkey_add(0, 0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_model_publication_set(NULL));

    access_model_id_t model_id = {};
    nrf_mesh_address_t address = {};
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_model_subscription_add(0, address, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_model_subscription_delete(0, address, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_model_subscription_overwrite(0, address, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_model_app_bind(0, 0, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, config_client_model_app_unbind(0, 0, model_id));
}

void test_server_set(void)
{
    m_handle = ACCESS_HANDLE_INVALID;
    __setup();
    TEST_NRF_MESH_ASSERT_EXPECT(config_client_server_set(0, 0));

    config_client_reset();
    m_handle = 1;
    __setup();

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, config_client_server_set(DSM_HANDLE_INVALID, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_ADDR, config_client_server_set(0, DSM_HANDLE_INVALID));

    access_model_publish_address_get_ExpectAndReturn(m_handle, NULL, ERROR_CODE);
    access_model_publish_address_get_IgnoreArg_p_address_handle();
    TEST_ASSERT_EQUAL(ERROR_CODE, config_client_server_set(0, 0));


    /* Check that failing to set the server address results in the old one added back in. */
    access_model_publish_address_get_ExpectAndReturn(m_handle, NULL, NRF_SUCCESS);
    access_model_publish_address_get_IgnoreArg_p_address_handle();
    dsm_handle_t address = 1;
    access_model_publish_address_get_ReturnThruPtr_p_address_handle(&address);
    access_model_publish_address_set_ExpectAndReturn(m_handle, 42, NRF_SUCCESS);
    access_model_publish_application_set_ExpectAndReturn(m_handle, 52, ERROR_CODE);
    access_model_publish_address_set_ExpectAndReturn(m_handle, address, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(ERROR_CODE, config_client_server_set(52, 42));

    /* Happy path */
    address = 2;
    access_model_publish_address_get_ExpectAndReturn(m_handle, NULL, NRF_SUCCESS);
    access_model_publish_address_get_IgnoreArg_p_address_handle();
    access_model_publish_address_get_ReturnThruPtr_p_address_handle(&address);
    access_model_publish_address_set_ExpectAndReturn(m_handle, 42, NRF_SUCCESS);
    access_model_publish_application_set_ExpectAndReturn(m_handle, 52, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_server_set(52, 42));

}

void test_server_bind(void)
{
    __setup();
    access_model_application_bind_ExpectAndReturn(m_handle, 3, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_server_bind(3));
}


void test_composition_data_get(void)
{
    __setup();
    config_msg_composition_data_get_t composition_data = {0};
    EXPECT_TX(CONFIG_OPCODE_COMPOSITION_DATA_GET, &composition_data, sizeof(composition_data));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_composition_data_get(0));
    EXPECT_TIMEOUT();
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_TIMEOUT);
}


void test_appkey_add(void)
{
    __setup();

    config_msg_key_index_24_t key_indexes;
    config_msg_key_index_24_set(&key_indexes, 1, 2);

    const config_msg_appkey_add_t appkey_add =
        {
            .key_indexes = key_indexes,
            .appkey = APPKEY
        };

    EXPECT_TX(CONFIG_OPCODE_APPKEY_ADD, &appkey_add, sizeof(appkey_add));

    uint8_t appkey[NRF_MESH_KEY_SIZE] = APPKEY;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_appkey_add(1, 2, appkey));

    const config_msg_appkey_status_t status =
        {
            .status = 2,
            .key_indexes = key_indexes
        };
    EXPECT_ACK(CONFIG_OPCODE_APPKEY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_APPKEY_STATUS, (const uint8_t*) &status, sizeof(status));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_appkey_update(void)
{
    __setup();

    config_msg_key_index_24_t key_indexes;
    config_msg_key_index_24_set(&key_indexes, 1, 2);

    const config_msg_appkey_add_t appkey_update =
        {
            .key_indexes = key_indexes,
            .appkey = APPKEY
        };

    EXPECT_TX(CONFIG_OPCODE_APPKEY_UPDATE, &appkey_update, sizeof(appkey_update));

    uint8_t appkey[NRF_MESH_KEY_SIZE] = APPKEY;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_appkey_update(1, 2, appkey));


    const config_msg_appkey_status_t status =
        {
            .status = 2,
            .key_indexes = key_indexes
        };
    EXPECT_ACK(CONFIG_OPCODE_APPKEY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_APPKEY_STATUS, (const uint8_t*) &status, sizeof(status));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_appkey_delete(void)
{
    __setup();

    config_msg_key_index_24_t key_indexes;
    config_msg_key_index_24_set(&key_indexes, 1, 2);

    const config_msg_appkey_delete_t appkey_delete =
        {
            .key_indexes = key_indexes
        };

    EXPECT_TX(CONFIG_OPCODE_APPKEY_DELETE, &appkey_delete, sizeof(appkey_delete));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_appkey_delete(1, 2));


    const config_msg_appkey_status_t status =
        {
            .status = 2,
            .key_indexes = key_indexes
        };
    EXPECT_ACK(CONFIG_OPCODE_APPKEY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_APPKEY_STATUS, (const uint8_t*) &status, sizeof(status));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_appkey_get(void)
{
    __setup();

    config_msg_key_index_12_t key_index = 23;

    const config_msg_appkey_get_t appkey_get =
        {
            .netkey_index = key_index
        };

    EXPECT_TX(CONFIG_OPCODE_APPKEY_GET, &appkey_get, sizeof(appkey_get));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_appkey_get(23));

    const config_msg_appkey_list_t list =
        {
            .status = 3,
            .netkey_index = key_index
        };

    EXPECT_ACK(CONFIG_OPCODE_APPKEY_LIST, &list, sizeof(list));
    send_ack(CONFIG_OPCODE_APPKEY_LIST, (const uint8_t*) &list, sizeof(list));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_netkey_add(void)
{
    __setup();
    config_msg_key_index_12_t netkey_index = 12;

    const config_msg_netkey_add_update_t netkey_add =
        {
            .netkey_index = netkey_index,
            .netkey = NETKEY
        };

    EXPECT_TX(CONFIG_OPCODE_NETKEY_ADD, &netkey_add, sizeof(netkey_add));

    const uint8_t netkey[NRF_MESH_KEY_SIZE] = NETKEY;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_netkey_add(12, netkey));

    const config_msg_netkey_status_t status =
        {
            .status = 3,
            .netkey_index = netkey_index
        };

    EXPECT_ACK(CONFIG_OPCODE_NETKEY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_NETKEY_STATUS, (const uint8_t *) &status, sizeof(status));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_netkey_delete(void)
{
    __setup();
    config_msg_key_index_12_t netkey_index = 12;

    const config_msg_netkey_delete_t netkey_delete =
        {
            .netkey_index = netkey_index
        };

    EXPECT_TX(CONFIG_OPCODE_NETKEY_DELETE, &netkey_delete, sizeof(netkey_delete));

    const config_msg_netkey_status_t status =
        {
            .status = 3,
            .netkey_index = netkey_index
        };
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_netkey_delete(12));

    EXPECT_ACK(CONFIG_OPCODE_NETKEY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_NETKEY_STATUS, (const uint8_t *) &status, sizeof(status));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_netkey_get(void)
{
    __setup();
    EXPECT_TX(CONFIG_OPCODE_NETKEY_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_netkey_get());

    const uint16_t index_list[] = {1, 2, 3, 4};
    uint8_t packed_list[PACKED_INDEX_LIST_SIZE(4)];

    packed_index_list_create(index_list, packed_list, 4);
    EXPECT_ACK(CONFIG_OPCODE_NETKEY_LIST, packed_list, sizeof(packed_list)); /*lint !e419 Apparent data overrun - safe in this case*/
    send_ack(CONFIG_OPCODE_NETKEY_LIST, packed_list, sizeof(packed_list));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_netkey_update(void)
{
    __setup();
    config_msg_key_index_12_t netkey_index = 12;

    const config_msg_netkey_add_update_t netkey_update =
        {
            .netkey_index = netkey_index,
            .netkey = NETKEY
        };

    EXPECT_TX(CONFIG_OPCODE_NETKEY_UPDATE, &netkey_update, sizeof(netkey_update));

    const uint8_t netkey[NRF_MESH_KEY_SIZE] = NETKEY;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_netkey_update(12, netkey));

    const config_msg_netkey_status_t status =
        {
            .status = 3,
            .netkey_index = netkey_index
        };

    EXPECT_ACK(CONFIG_OPCODE_NETKEY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_NETKEY_STATUS, (const uint8_t *) &status, sizeof(status));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_publication_get(void)
{
    __setup();
    const config_msg_publication_get_t msg =
    {
        .element_address = 3,
        .model_id.vendor.model_id = 1,
        .model_id.vendor.company_id = 2
    };

    const access_model_id_t model_id =
    {
        .model_id = 1,
        .company_id = 2
    };
    EXPECT_TX(CONFIG_OPCODE_MODEL_PUBLICATION_GET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_publication_get(3, model_id));

    config_msg_publication_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &ack, sizeof(ack));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_publication_set(void)
{
    __setup();
    config_publication_state_t ps;
    ps.element_address = 1;
    ps.publish_address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    ps.publish_address.value = 2;
    ps.appkey_index = 3;
    ps.frendship_credential_flag = false;
    ps.publish_ttl = 4;
    ps.publish_period.step_num = 5;
    ps.publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_100MS;
    ps.retransmit_count = 6;
    ps.retransmit_interval = 7;
    ps.model_id.company_id = 8;
    ps.model_id.model_id = 9;

    config_msg_publication_set_t msg;
    msg.element_address = ps.element_address;
    msg.publish_address = ps.publish_address.value;
    msg.state.appkey_index = ps.appkey_index;
    msg.state.credential_flag = 0;
    msg.state.rfu = 0;
    msg.state.publish_ttl = ps.publish_ttl;
    msg.state.publish_period = ps.publish_period.step_res << 6u | ps.publish_period.step_num;
    msg.state.retransmit_count = ps.retransmit_count;
    msg.state.retransmit_interval = ps.retransmit_interval;
    msg.state.model_id.vendor.company_id = ps.model_id.company_id;
    msg.state.model_id.vendor.model_id = ps.model_id.model_id;

    EXPECT_TX(CONFIG_OPCODE_MODEL_PUBLICATION_SET, &msg, sizeof(msg));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_publication_set(&ps));

    config_msg_publication_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &ack, sizeof(ack));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_publication_virtual_set(void)
{
    __setup();

    const uint8_t virtual_uuid[NRF_MESH_UUID_SIZE] = VIRTUAL_UUID;
    config_publication_state_t ps;
    ps.element_address = 1;
    ps.publish_address.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    ps.publish_address.p_virtual_uuid = virtual_uuid;
    ps.appkey_index = 3;
    ps.frendship_credential_flag = false;
    ps.publish_ttl = 4;
    ps.publish_period.step_num = 5;
    ps.publish_period.step_res = ACCESS_PUBLISH_RESOLUTION_100MS;
    ps.retransmit_count = 6;
    ps.retransmit_interval = 7;
    ps.model_id.company_id = 8;
    ps.model_id.model_id = 9;

    config_msg_publication_virtual_set_t msg;
    msg.element_address = ps.element_address;
    memcpy(msg.publish_uuid, virtual_uuid, sizeof(virtual_uuid));
    msg.state.appkey_index = ps.appkey_index;
    msg.state.credential_flag = 0;
    msg.state.rfu = 0;
    msg.state.publish_ttl = ps.publish_ttl;
    msg.state.publish_period = ps.publish_period.step_res << 6u | ps.publish_period.step_num;
    msg.state.retransmit_count = ps.retransmit_count;
    msg.state.retransmit_interval = ps.retransmit_interval;
    msg.state.model_id.vendor.company_id = ps.model_id.company_id;
    msg.state.model_id.vendor.model_id = ps.model_id.model_id;

    EXPECT_TX(CONFIG_OPCODE_MODEL_PUBLICATION_VIRTUAL_ADDRESS_SET, &msg, sizeof(msg));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_publication_set(&ps));

    config_msg_publication_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_PUBLICATION_STATUS, (const uint8_t *) &ack, sizeof(ack));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_subscription_add(void)
{
    __setup();
    uint16_t element_address = 1;
    nrf_mesh_address_t address = {0};
    address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    address.value = 2;
    access_model_id_t model_id = {0};
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    model_id.model_id = 3;

    config_msg_subscription_add_del_owr_t msg;
    msg.address = address.value;
    msg.element_address = element_address;
    msg.model_id.sig.model_id = model_id.model_id;

    /* Length is shortened by 2 bytes because of SIG model ID. */
    uint16_t length = (sizeof(msg) - sizeof(uint16_t));
    EXPECT_TX(CONFIG_OPCODE_MODEL_SUBSCRIPTION_ADD, &msg, length);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_add(element_address, address, model_id));

    config_msg_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, sizeof(ack));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_subscription_virtual_add(void)
{
    __setup();
    uint16_t element_address = 1;
    const uint8_t virtual_uuid[NRF_MESH_UUID_SIZE] = VIRTUAL_UUID;
    nrf_mesh_address_t address = {0};
    address.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    address.p_virtual_uuid = virtual_uuid;

    access_model_id_t model_id = {0};
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    model_id.model_id = 3;

    config_msg_subscription_virtual_add_del_owr_t msg;
    memcpy(msg.virtual_uuid, virtual_uuid, sizeof(virtual_uuid));
    msg.element_address = element_address;
    msg.model_id.sig.model_id = model_id.model_id;

    /* Length is shortened by 2 bytes because of SIG model ID. */
    uint16_t length = (sizeof(msg) - sizeof(uint16_t));
    EXPECT_TX(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_ADD, &msg, length);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_add(element_address, address, model_id));

    config_msg_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, sizeof(ack));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_subscription_delete(void)
{
    __setup();
    uint16_t element_address = 1;
    nrf_mesh_address_t address = {0};
    address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    address.value = 2;
    access_model_id_t model_id = {0};
    model_id.company_id = 4;
    model_id.model_id = 3;

    config_msg_subscription_add_del_owr_t msg;
    msg.address = address.value;
    msg.element_address = element_address;
    msg.model_id.vendor.model_id = model_id.model_id;
    msg.model_id.vendor.company_id = model_id.company_id;

    EXPECT_TX(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_delete(element_address, address, model_id));

    config_msg_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, sizeof(ack));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_subscription_virtual_delete(void)
{
    __setup();
    uint16_t element_address = 1;
    const uint8_t virtual_uuid[NRF_MESH_UUID_SIZE] = VIRTUAL_UUID;
    nrf_mesh_address_t address = {0};
    address.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    address.p_virtual_uuid = virtual_uuid;

    access_model_id_t model_id = {0};
    model_id.company_id = 4;
    model_id.model_id = 3;

    config_msg_subscription_virtual_add_del_owr_t msg;
    memcpy(msg.virtual_uuid, virtual_uuid, sizeof(virtual_uuid));
    msg.element_address = element_address;
    msg.model_id.vendor.model_id = model_id.model_id;
    msg.model_id.vendor.company_id = model_id.company_id;

    EXPECT_TX(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_DELETE, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_delete(element_address, address, model_id));

    config_msg_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, sizeof(ack));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_subscription_overwrite(void)
{
    __setup();
    uint16_t element_address = 1;
    nrf_mesh_address_t address = {0};
    address.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    address.value = 2;
    access_model_id_t model_id = {0};
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    model_id.model_id = 3;

    config_msg_subscription_add_del_owr_t msg;
    msg.address = address.value;
    msg.element_address = element_address;
    msg.model_id.sig.model_id = model_id.model_id;

    /* Length is shortened by 2 bytes because of SIG model ID. */
    uint16_t length = (sizeof(msg) - sizeof(uint16_t));
    EXPECT_TX(CONFIG_OPCODE_MODEL_SUBSCRIPTION_OVERWRITE, &msg, length);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_overwrite(element_address, address, model_id));

    config_msg_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));
    length = sizeof(ack) - sizeof(uint16_t);
    EXPECT_ACK(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, &ack, length);
    send_ack(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, length);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_subscription_virtual_overwrite(void)
{
    __setup();
    uint16_t element_address = 1;
    const uint8_t virtual_uuid[NRF_MESH_UUID_SIZE] = VIRTUAL_UUID;
    nrf_mesh_address_t address = {0};
    address.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    address.p_virtual_uuid = virtual_uuid;

    access_model_id_t model_id = {0};
    model_id.company_id = 4;
    model_id.model_id = 3;

    config_msg_subscription_virtual_add_del_owr_t msg;
    memcpy(msg.virtual_uuid, virtual_uuid, sizeof(virtual_uuid));
    msg.element_address = element_address;
    msg.model_id.vendor.model_id = model_id.model_id;
    msg.model_id.vendor.company_id = model_id.company_id;

    /* Length is shortened by 2 bytes because of SIG model ID. */
    uint16_t length = (sizeof(msg) - sizeof(uint16_t));
    EXPECT_TX(CONFIG_OPCODE_MODEL_SUBSCRIPTION_VIRTUAL_ADDRESS_OVERWRITE, &msg, length);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_overwrite(element_address, address, model_id));

    config_msg_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));
    length = sizeof(ack) - sizeof(uint16_t);
    EXPECT_ACK(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, &ack, length);
    send_ack(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, length);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}


void test_subscription_delete_all(void)
{
    __setup();
    uint16_t element_address = 1;
    access_model_id_t model_id = {0};
    model_id.company_id = 4;
    model_id.model_id = 3;

    config_msg_subscription_delete_all_t msg;
    msg.element_address = element_address;
    msg.model_id.vendor.model_id = model_id.model_id;
    msg.model_id.vendor.company_id = model_id.company_id;

    EXPECT_TX(CONFIG_OPCODE_MODEL_SUBSCRIPTION_DELETE_ALL, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_delete_all(element_address, model_id));

    config_msg_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, sizeof(ack));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_subscription_get(void)
{
    __setup();
    uint16_t element_address = 1;

    /* Vendor version */
    access_model_id_t model_id = {0};
    model_id.company_id = 4;
    model_id.model_id = 3;

    config_msg_model_subscription_get_t msg;
    msg.element_address = element_address;
    msg.model_id.vendor.model_id = model_id.model_id;
    msg.model_id.vendor.company_id = model_id.company_id;

    EXPECT_TX(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_GET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_get(element_address, model_id));

    union {
        config_msg_vendor_model_subscription_list_t vendor_subscription_list;
        config_msg_sig_model_subscription_list_t sig_subscription_list;
        uint8_t raw[NRF_MESH_SEG_PAYLOAD_SIZE_MAX];
    } ack;
    ack.vendor_subscription_list.element_address = 1;
    ack.vendor_subscription_list.vendor_company_id = 4;
    ack.vendor_subscription_list.vendor_model_id = 3;
    memset(&ack.vendor_subscription_list.subscriptions[0], 0x12, NRF_MESH_SEG_PAYLOAD_SIZE_MAX - sizeof(config_msg_vendor_model_app_list_t));

    EXPECT_ACK(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST, &ack.vendor_subscription_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    send_ack(CONFIG_OPCODE_VENDOR_MODEL_SUBSCRIPTION_LIST, (const uint8_t *) &ack.sig_subscription_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);

    /* SIG version */
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    model_id.model_id = 6;

    msg.element_address = element_address;
    msg.model_id.sig.model_id = model_id.model_id;

    EXPECT_TX(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_GET, &msg, (sizeof(msg) - sizeof(uint16_t)));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_subscription_get(element_address, model_id));

    ack.sig_subscription_list.element_address = 1;
    ack.sig_subscription_list.sig_model_id = 6;
    memset(&ack.sig_subscription_list.subscriptions[0], 0x12, NRF_MESH_SEG_PAYLOAD_SIZE_MAX - sizeof(config_msg_sig_model_app_list_t));

    EXPECT_ACK(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, &ack.sig_subscription_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    send_ack(CONFIG_OPCODE_SIG_MODEL_SUBSCRIPTION_LIST, (const uint8_t *) &ack.sig_subscription_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_app_bind_unbind(void)
{
    __setup();
    uint16_t element_address = 1;
    uint16_t appkey_index = 5;
    access_model_id_t model_id = {0};
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    model_id.model_id = 3;

    config_msg_app_bind_unbind_t msg;
    msg.appkey_index = appkey_index;
    msg.element_address = element_address;
    msg.model_id.sig.model_id = model_id.model_id;

    /* Length is shortened by 2 bytes because of SIG model ID. */
    uint16_t length = (sizeof(msg) - sizeof(uint16_t));
    EXPECT_TX(CONFIG_OPCODE_MODEL_APP_BIND, &msg, length);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_app_bind(element_address, appkey_index, model_id));

    config_msg_app_status_t ack;
    memset(&ack, 0x12, sizeof(ack));
    length = sizeof(ack) - sizeof(uint16_t);
    EXPECT_ACK(CONFIG_OPCODE_MODEL_APP_STATUS, &ack, length);
    send_ack(CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &ack, length);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);

    /* Unbind */
    model_id.company_id = 42;
    msg.model_id.vendor.model_id = model_id.model_id;
    msg.model_id.vendor.company_id = model_id.company_id;
    EXPECT_TX(CONFIG_OPCODE_MODEL_APP_UNBIND, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_app_unbind(element_address, appkey_index, model_id));

    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_MODEL_APP_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_MODEL_APP_STATUS, (const uint8_t *) &ack, sizeof(ack));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_app_get(void)
{
    __setup();
    uint16_t element_address = 1;

    /* Vendor version */
    access_model_id_t model_id = {0};
    model_id.company_id = 4;
    model_id.model_id = 3;

    config_msg_model_app_get_t msg;
    msg.element_address = element_address;
    msg.model_id.vendor.model_id = model_id.model_id;
    msg.model_id.vendor.company_id = model_id.company_id;

    EXPECT_TX(CONFIG_OPCODE_VENDOR_MODEL_APP_GET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_app_get(element_address, model_id));

    union {
        config_msg_vendor_model_app_list_t vendor_app_list;
        config_msg_sig_model_app_list_t sig_app_list;
        uint8_t raw[NRF_MESH_SEG_PAYLOAD_SIZE_MAX];
    } ack;
    ack.vendor_app_list.element_address = 1;
    ack.vendor_app_list.vendor_company_id = 4;
    ack.vendor_app_list.vendor_model_id = 3;
    memset(&ack.vendor_app_list.key_indexes[0], 0x12, NRF_MESH_SEG_PAYLOAD_SIZE_MAX - sizeof(config_msg_vendor_model_app_list_t));

    EXPECT_ACK(CONFIG_OPCODE_VENDOR_MODEL_APP_LIST, &ack.vendor_app_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    send_ack(CONFIG_OPCODE_VENDOR_MODEL_APP_LIST, (const uint8_t *) &ack.vendor_app_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);

    /* SIG version */
    model_id.company_id = ACCESS_COMPANY_ID_NONE;
    model_id.model_id = 6;

    msg.element_address = element_address;
    msg.model_id.sig.model_id = model_id.model_id;

    EXPECT_TX(CONFIG_OPCODE_SIG_MODEL_APP_GET, &msg, (sizeof(msg) - sizeof(uint16_t)));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_model_app_get(element_address, model_id));

    ack.sig_app_list.element_address = 1;
    ack.sig_app_list.sig_model_id = 6;
    memset(&ack.sig_app_list.key_indexes[0], 0x12, NRF_MESH_SEG_PAYLOAD_SIZE_MAX - sizeof(config_msg_sig_model_app_list_t));

    EXPECT_ACK(CONFIG_OPCODE_SIG_MODEL_APP_LIST, &ack.sig_app_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    send_ack(CONFIG_OPCODE_SIG_MODEL_APP_LIST, (const uint8_t *) &ack.sig_app_list, NRF_MESH_SEG_PAYLOAD_SIZE_MAX);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_ttl_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_DEFAULT_TTL_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_default_ttl_get());

    const config_msg_default_ttl_status_t status =
        {
            .ttl = 4
        };
    EXPECT_ACK(CONFIG_OPCODE_DEFAULT_TTL_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_DEFAULT_TTL_STATUS, (const uint8_t*) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_ttl_set(void)
{
    __setup();

    const config_msg_default_ttl_set_t msg = {42};
    EXPECT_TX(CONFIG_OPCODE_DEFAULT_TTL_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, config_client_default_ttl_set(NRF_MESH_TTL_MAX + 1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_default_ttl_set(42));

    const config_msg_default_ttl_status_t status =
        {
            .ttl = 4
        };
    EXPECT_ACK(CONFIG_OPCODE_DEFAULT_TTL_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_DEFAULT_TTL_STATUS, (const uint8_t*) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_relay_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_RELAY_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_relay_get());

    const config_msg_relay_status_t status =
        {
            .relay_state = CONFIG_RELAY_STATE_SUPPORTED_ENABLED,
            .relay_retransmit_count = 2,
            .relay_retransmit_interval_steps = 4
        };
    EXPECT_ACK(CONFIG_OPCODE_RELAY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_RELAY_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_relay_set(void)
{
    __setup();

    const config_msg_relay_status_t msg =
        {
            .relay_state = CONFIG_RELAY_STATE_SUPPORTED_ENABLED,
            .relay_retransmit_count = CONFIG_RETRANSMIT_COUNT_MAX,
            .relay_retransmit_interval_steps = CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX
        };

    EXPECT_TX(CONFIG_OPCODE_RELAY_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, config_client_relay_set(CONFIG_RELAY_STATE_SUPPORTED_ENABLED, CONFIG_RETRANSMIT_COUNT_MAX+1, 4));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, config_client_relay_set(CONFIG_RELAY_STATE_SUPPORTED_ENABLED, 2, CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX+1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_relay_set(CONFIG_RELAY_STATE_SUPPORTED_ENABLED, CONFIG_RETRANSMIT_COUNT_MAX, CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX));

    const config_msg_relay_status_t status =
        {
            .relay_state = CONFIG_RELAY_STATE_SUPPORTED_ENABLED,
            .relay_retransmit_count = CONFIG_RETRANSMIT_COUNT_MAX,
            .relay_retransmit_interval_steps = CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX
        };
    EXPECT_ACK(CONFIG_OPCODE_RELAY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_RELAY_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_network_transmit_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_NETWORK_TRANSMIT_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_network_transmit_get());

    const config_msg_network_transmit_status_t status =
        {
            .network_transmit_count = 2,
            .network_transmit_interval_steps = 4
        };
    EXPECT_ACK(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_network_transmit_set(void)
{
    __setup();

    const config_msg_network_transmit_status_t msg =
        {
            .network_transmit_count = CONFIG_RETRANSMIT_COUNT_MAX,
            .network_transmit_interval_steps = CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX
        };

    EXPECT_TX(CONFIG_OPCODE_NETWORK_TRANSMIT_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, config_client_network_transmit_set(CONFIG_RETRANSMIT_COUNT_MAX+1, 4));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, config_client_network_transmit_set(2, CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX+1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_network_transmit_set(CONFIG_RETRANSMIT_COUNT_MAX, CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX));

    const config_msg_network_transmit_status_t status =
        {
            .network_transmit_count = CONFIG_RETRANSMIT_COUNT_MAX,
            .network_transmit_interval_steps = CONFIG_RETRANSMIT_INTERVAL_STEPS_MAX
        };
    EXPECT_ACK(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_NETWORK_TRANSMIT_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_beacon_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_BEACON_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_net_beacon_get());

    const config_msg_net_beacon_status_t status =
        {
            .beacon_state = CONFIG_NET_BEACON_STATE_ENABLED
        };
    EXPECT_ACK(CONFIG_OPCODE_BEACON_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_BEACON_STATUS, (const uint8_t*) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_beacon_set(void)
{
    __setup();

    const config_msg_net_beacon_set_t msg =
        {
            .beacon_state = CONFIG_NET_BEACON_STATE_ENABLED
        };

    EXPECT_TX(CONFIG_OPCODE_BEACON_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_net_beacon_set(CONFIG_NET_BEACON_STATE_ENABLED));

    const config_msg_net_beacon_status_t status =
        {
            .beacon_state = CONFIG_NET_BEACON_STATE_ENABLED
        };
    EXPECT_ACK(CONFIG_OPCODE_BEACON_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_BEACON_STATUS, (const uint8_t*) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_node_reset(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_NODE_RESET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_node_reset());

    EXPECT_ACK(CONFIG_OPCODE_NODE_RESET_STATUS, NULL, 0);
    send_ack(CONFIG_OPCODE_NODE_RESET_STATUS, NULL, 0);
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_key_refresh_phase_get(void)
{
    __setup();

    config_msg_key_index_12_t netkey_index = 12;
    const config_msg_key_refresh_phase_get_t msg =
        {
            .netkey_index = netkey_index
        };
    EXPECT_TX(CONFIG_OPCODE_KEY_REFRESH_PHASE_GET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_key_refresh_phase_get(netkey_index));

    const config_msg_key_refresh_phase_status_t status =
        {
            .netkey_index = netkey_index,
            .phase = NRF_MESH_KEY_REFRESH_PHASE_2
        };
    EXPECT_ACK(CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_key_refresh_phase_set(void)
{
    __setup();

    config_msg_key_index_12_t netkey_index = 12;
    const config_msg_key_refresh_phase_set_t msg =
        {
            .netkey_index = netkey_index,
            .transition = NRF_MESH_KEY_REFRESH_PHASE_2
        };
    EXPECT_TX(CONFIG_OPCODE_KEY_REFRESH_PHASE_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_key_refresh_phase_set(netkey_index, NRF_MESH_KEY_REFRESH_PHASE_2));

    const config_msg_key_refresh_phase_status_t status =
        {
            .netkey_index = netkey_index,
            .phase = NRF_MESH_KEY_REFRESH_PHASE_2
        };
    EXPECT_ACK(CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_KEY_REFRESH_PHASE_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_config_client_heartbeat_publication_set(void)
{
    __setup();

    config_msg_heartbeat_publication_set_t msg;
    msg.destination  = 0x1234;
    msg.count_log    = 0x10;
    msg.period_log   = 0x10;
    msg.ttl          = 0x10;
    msg.features     = 0x07;
    msg.netkey_index = 0x00;

    EXPECT_TX(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_heartbeat_publication_set(&msg));

    config_msg_heartbeat_publication_status_t ack = {0};
    ack.status       = 0x00;
    ack.destination  = msg.destination;
    ack.count_log    = msg.count_log;
    ack.period_log   = msg.period_log;
    ack.ttl          = msg.ttl;
    ack.features     = msg.features;
    ack.netkey_index = msg.netkey_index;

    EXPECT_ACK(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS, (const uint8_t *) &ack, sizeof(ack));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_config_client_heartbeat_publication_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_heartbeat_publication_get());

    config_msg_heartbeat_publication_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_STATUS, (const uint8_t *) &ack, sizeof(ack));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_config_client_heartbeat_subscription_set(void)
{
    __setup();

    config_msg_heartbeat_subscription_set_t msg;
    msg.source       = 0x4321;
    msg.destination  = 0x1234;
    msg.period_log   = 0x10;

    EXPECT_TX(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_heartbeat_subscription_set(&msg));

    config_msg_heartbeat_subscription_status_t ack = {0};
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, sizeof(ack));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_config_client_heartbeat_subscription_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_heartbeat_subscription_get());

    config_msg_heartbeat_subscription_status_t ack;
    memset(&ack, 0x12, sizeof(ack));

    EXPECT_ACK(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS, &ack, sizeof(ack));
    send_ack(CONFIG_OPCODE_HEARTBEAT_SUBSCRIPTION_STATUS, (const uint8_t *) &ack, sizeof(ack));

    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_friend_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_FRIEND_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_friend_get());

    const config_msg_friend_status_t status =
        {
            .friend_state = CONFIG_FRIEND_STATE_SUPPORTED_ENABLED,
        };
    EXPECT_ACK(CONFIG_OPCODE_FRIEND_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_FRIEND_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_friend_set(void)
{
    __setup();

    const config_msg_friend_set_t msg =
        {
            .friend_state = CONFIG_FRIEND_STATE_SUPPORTED_ENABLED,
        };
    EXPECT_TX(CONFIG_OPCODE_FRIEND_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_friend_set(CONFIG_FRIEND_STATE_SUPPORTED_ENABLED));

    const config_msg_friend_status_t status =
        {
            .friend_state = CONFIG_FRIEND_STATE_SUPPORTED_ENABLED,
        };
    EXPECT_ACK(CONFIG_OPCODE_FRIEND_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_FRIEND_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_gatt_proxy_get(void)
{
    __setup();

    EXPECT_TX(CONFIG_OPCODE_GATT_PROXY_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_gatt_proxy_get());

    const config_msg_proxy_status_t status =
        {
            .proxy_state = CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED,
        };
    EXPECT_ACK(CONFIG_OPCODE_GATT_PROXY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_GATT_PROXY_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_gatt_proxy_set(void)
{
    __setup();

    const config_msg_proxy_set_t msg =
        {
            .proxy_state = CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED,
        };
    EXPECT_TX(CONFIG_OPCODE_GATT_PROXY_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_gatt_proxy_set(CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED));

    const config_msg_proxy_status_t status =
        {
            .proxy_state = CONFIG_GATT_PROXY_STATE_RUNNING_ENABLED,
        };
    EXPECT_ACK(CONFIG_OPCODE_GATT_PROXY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_GATT_PROXY_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_node_identity_get(void)
{
    __setup();

    config_msg_key_index_12_t netkey_index = 12;
    const config_msg_identity_get_t msg =
        {
            .netkey_index = netkey_index
        };
    EXPECT_TX(CONFIG_OPCODE_NODE_IDENTITY_GET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_node_identity_get(netkey_index));

    const config_msg_identity_status_t status =
        {
            .netkey_index = netkey_index,
            .identity_state = CONFIG_IDENTITY_STATE_RUNNING
        };
    EXPECT_ACK(CONFIG_OPCODE_NODE_IDENTITY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_NODE_IDENTITY_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_node_identity_set(void)
{
    __setup();

    config_msg_key_index_12_t netkey_index = 12;
    const config_msg_identity_set_t msg =
        {
            .netkey_index = netkey_index,
            .identity_state = CONFIG_IDENTITY_STATE_RUNNING
        };
    EXPECT_TX(CONFIG_OPCODE_NODE_IDENTITY_SET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_node_identity_set(netkey_index, CONFIG_IDENTITY_STATE_RUNNING));

    const config_msg_identity_status_t status =
        {
            .netkey_index = netkey_index,
            .identity_state = CONFIG_IDENTITY_STATE_RUNNING
        };
    EXPECT_ACK(CONFIG_OPCODE_NODE_IDENTITY_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_NODE_IDENTITY_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_low_power_node_polltimeout_get(void)
{
    __setup();

    const config_msg_low_power_node_polltimeout_get_t msg =
        {
            .lpn_address = 0x1234,
        };
    EXPECT_TX(CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_GET, &msg, sizeof(msg));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_low_power_node_polltimeout_get(0x1234));

    const config_msg_low_power_node_polltimeout_status_t status =
        {
            .lpn_address = 0x1234,
            .polltimeout = {0xDE,0xAD,0xBE},
        };
    EXPECT_ACK(CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS, &status, sizeof(status));
    send_ack(CONFIG_OPCODE_LOW_POWER_NODE_POLLTIMEOUT_STATUS, (const uint8_t *) &status, sizeof(status));
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_SUCCESS);
}

void test_busy_state(void)
{
    __setup();
    config_msg_composition_data_get_t composition_data = {0};
    EXPECT_TX(CONFIG_OPCODE_COMPOSITION_DATA_GET, &composition_data, sizeof(composition_data));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_composition_data_get(0));

    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_server_set(0, 0));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_server_bind(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_composition_data_get(0));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_appkey_add(0, 0, NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_model_publication_set(NULL));

    access_model_id_t model_id = {};
    nrf_mesh_address_t address = {};
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_model_subscription_add(0, address, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_model_subscription_delete(0, address, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_model_subscription_overwrite(0, address, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_model_app_bind(0, 0, model_id));
    TEST_ASSERT_EQUAL(NRF_ERROR_BUSY, config_client_model_app_unbind(0, 0, model_id));
    EXPECT_TIMEOUT();
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_TIMEOUT);
}

void test_alloc_fail(void)
{
    __setup();
    access_model_reliable_publish_IgnoreAndReturn(NRF_ERROR_NO_MEM);
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, config_client_composition_data_get(0));
}

void test_cancelled(void)
{
    __setup();
    EXPECT_TX(CONFIG_OPCODE_HEARTBEAT_PUBLICATION_GET, NULL, 0);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, config_client_heartbeat_publication_get());
    EXPECT_CANCELLED();
    m_reliable_cb(m_handle, NULL, ACCESS_RELIABLE_TRANSFER_CANCELLED);
}

void test_pending_msg_cancel(void)
{
    __setup();
    access_model_reliable_cancel_ExpectAndReturn(m_handle, NRF_SUCCESS);
    config_client_pending_msg_cancel();
}
