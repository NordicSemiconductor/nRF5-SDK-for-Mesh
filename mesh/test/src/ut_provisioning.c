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

#include "provisioning.h"

#include <unity.h>
#include <cmock.h>
#include <stdint.h>

#include "nrf_mesh_prov_bearer.h"

/** Arbitrary error code using the full 4 byte error code space in NRF_ERROR_*. Used to ensure full error is forwarded from other modules. */
#define ARBITRARY_ERROR     (0x12345678)

static void m_prov_cb_rx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);
static void m_prov_cb_ack(prov_bearer_t * p_bearer);
static void m_prov_cb_link_opened(prov_bearer_t * p_bearer);
static void m_prov_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason);

static uint32_t m_cb_rx_expected;
static uint32_t m_cb_ack_expected;
static uint32_t m_cb_link_opened_expected;
static uint32_t m_cb_link_closed_expected;

static uint32_t m_bearer_if_tx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length);
static uint32_t m_bearer_if_listen_start(prov_bearer_t * p_bearer, const char * p_uri, uint16_t oob_info, uint32_t link_timeout_us);
static uint32_t m_bearer_if_listen_stop(prov_bearer_t * p_bearer);
static uint32_t m_bearer_if_link_open(prov_bearer_t * p_bearer, const uint8_t * p_uuid, uint32_t link_timeout_us);
static void m_bearer_if_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason);

static nrf_mesh_prov_link_close_reason_t m_close_reason;
static uint8_t * mp_rx_pointer;
static uint16_t m_rx_length;

static prov_bearer_interface_t m_interface =
{
    .tx = m_bearer_if_tx,
    .listen_start = m_bearer_if_listen_start,
    .listen_stop = m_bearer_if_listen_stop,
    .link_open = m_bearer_if_link_open,
    .link_close = m_bearer_if_link_close
};
static prov_bearer_callbacks_t m_callbacks =
{
    m_prov_cb_rx,
    m_prov_cb_ack,
    m_prov_cb_link_opened,
    m_prov_cb_link_closed
};

static prov_bearer_t m_bearer;

/********** Test Initialization and Finalization ***********/

void setUp(void)
{
    m_bearer.node.p_next = NULL;
    m_bearer.bearer_type = NRF_MESH_PROV_BEARER_ADV;
    m_bearer.p_interface = &m_interface;
    m_bearer.p_callbacks = &m_callbacks;
    m_bearer.p_parent = (void *) 0xCAFEBABE;
    m_cb_rx_expected = 0;
    m_cb_ack_expected = 0;
    m_cb_link_opened_expected = 0;
    m_cb_link_closed_expected = 0;
}

void tearDown(void)
{
}

/********** Callbacks ***********/

#define TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(p_bearer, p_data, data_length, length, retval) \
    do { \
        mp_bearer_if_tx_expect_bearer = (p_bearer); \
        mp_bearer_if_tx_expect_data = (p_data); \
        m_bearer_if_tx_expect_data_length = (data_length); \
        m_bearer_if_tx_expect_length = (length); \
        m_bearer_if_tx_retval = (retval); \
    } while (0);

static prov_bearer_t * mp_bearer_if_tx_expect_bearer;
static const uint8_t * mp_bearer_if_tx_expect_data;
static uint16_t m_bearer_if_tx_expect_data_length;
static uint16_t m_bearer_if_tx_expect_length;
static uint32_t m_bearer_if_tx_retval;

static uint32_t m_bearer_if_tx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    TEST_ASSERT_EQUAL_PTR(mp_bearer_if_tx_expect_bearer, p_bearer);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(mp_bearer_if_tx_expect_data, p_data, m_bearer_if_tx_expect_data_length);
    TEST_ASSERT_EQUAL_UINT16(m_bearer_if_tx_expect_length, length);
    return m_bearer_if_tx_retval;
}

#define LISTEN_START_EXPECT_AND_RETURN(p_bearer, p_uri, oob_info, link_timeout, retval) \
    do { \
        mp_bearer_if_listen_start_expect_bearer = p_bearer; \
        mp_bearer_if_listen_start_expect_uri = p_uri; \
        m_bearer_if_listen_start_expect_oob_info = oob_info; \
        m_bearer_if_listen_start_expect_timeout = link_timeout; \
        m_bearer_if_listen_start_return = retval; \
    } while (0);

static prov_bearer_t * mp_bearer_if_listen_start_expect_bearer;
static const char * mp_bearer_if_listen_start_expect_uri;
static uint16_t m_bearer_if_listen_start_expect_oob_info;
static uint32_t m_bearer_if_listen_start_expect_timeout;
static uint32_t m_bearer_if_listen_start_return;

static uint32_t m_bearer_if_listen_start(prov_bearer_t * p_bearer, const char * p_uri, uint16_t oob_info, uint32_t link_timeout_us)
{
    TEST_ASSERT_EQUAL_PTR(mp_bearer_if_listen_start_expect_bearer, p_bearer);
    TEST_ASSERT_EQUAL_PTR(mp_bearer_if_listen_start_expect_uri, p_uri);
    TEST_ASSERT_EQUAL_UINT16(m_bearer_if_listen_start_expect_oob_info, oob_info);
    TEST_ASSERT_EQUAL_UINT32(m_bearer_if_listen_start_expect_timeout, link_timeout_us);
    return m_bearer_if_listen_start_return;
}

#define LISTEN_STOP_EXPECT_AND_RETURN(p_bearer, retval) \
    do { \
        mp_bearer_if_listen_stop_expect_bearer = p_bearer; \
        m_bearer_if_listen_stop_retval = reval; \
    } while (0);

static prov_bearer_t * mp_bearer_if_listen_stop_expect_bearer;
static uint32_t m_bearer_if_listen_stop_return;

static uint32_t m_bearer_if_listen_stop(prov_bearer_t * p_bearer)
{
    TEST_ASSERT_EQUAL_PTR(mp_bearer_if_listen_stop_expect_bearer, p_bearer);
    return m_bearer_if_listen_stop_return;
}

#define LINK_OPEN_EXPECT_AND_RETURN(p_bearer, p_uuid, link_timeout, retval) \
    do { \
        mp_bearer_if_link_open_expect_bearer = p_bearer; \
        mp_bearer_if_link_open_expect_uuid = p_uuid; \
        m_bearer_if_link_open_expect_timeout = link_timeout; \
        m_bearer_if_link_open_retval = retval; \
    } while (0);

static prov_bearer_t * mp_bearer_if_link_open_expect_bearer;
static const uint8_t * mp_bearer_if_link_open_expect_uuid;
static uint32_t m_bearer_if_link_open_expect_timeout;
static uint32_t m_bearer_if_link_open_retval;

static uint32_t m_bearer_if_link_open(prov_bearer_t * p_bearer, const uint8_t * p_uuid, uint32_t link_timeout_us)
{
    TEST_ASSERT_EQUAL_PTR(mp_bearer_if_link_open_expect_bearer, p_bearer);
    TEST_ASSERT_EQUAL_PTR(mp_bearer_if_link_open_expect_uuid, p_uuid);
    TEST_ASSERT_EQUAL_UINT32(m_bearer_if_link_open_expect_timeout, link_timeout_us);
    return m_bearer_if_link_open_retval;
}

#define LINK_CLOSE_EXPECT(p_bearer, close_reason) \
    do { \
        mp_bearer_if_link_close_expect_bearer = p_bearer; \
        m_bearer_if_link_close_close_reason = close_reason; \
    } while (0);

static prov_bearer_t * mp_bearer_if_link_close_expect_bearer;
static nrf_mesh_prov_link_close_reason_t m_bearer_if_link_close_close_reason;

static void m_bearer_if_link_close(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t close_reason)
{
    TEST_ASSERT_EQUAL_PTR(mp_bearer_if_link_close_expect_bearer, p_bearer);
    TEST_ASSERT_EQUAL(m_bearer_if_link_close_close_reason, close_reason);
}

static void m_prov_cb_rx(prov_bearer_t * p_bearer, const uint8_t * p_data, uint16_t length)
{
    TEST_ASSERT_EQUAL_PTR(&m_bearer, p_bearer);
    TEST_ASSERT_TRUE(m_cb_rx_expected > 0);
    m_cb_rx_expected--;
    TEST_ASSERT_EQUAL_PTR(mp_rx_pointer, p_data);
    TEST_ASSERT_EQUAL(m_rx_length, length);
}

static void m_prov_cb_ack(prov_bearer_t * p_bearer)
{
    TEST_ASSERT_EQUAL_PTR(&m_bearer, p_bearer);
    TEST_ASSERT_TRUE(m_cb_ack_expected > 0);
    m_cb_ack_expected--;
}

static void m_prov_cb_link_opened(prov_bearer_t * p_bearer)
{
    TEST_ASSERT_EQUAL_PTR(&m_bearer, p_bearer);
    TEST_ASSERT_TRUE(m_cb_link_opened_expected > 0);
    m_cb_link_opened_expected--;
}

static void m_prov_cb_link_closed(prov_bearer_t * p_bearer, nrf_mesh_prov_link_close_reason_t reason)
{
    TEST_ASSERT_EQUAL_PTR(&m_bearer, p_bearer);
    TEST_ASSERT_TRUE(m_cb_link_closed_expected > 0);
    m_cb_link_closed_expected--;
    TEST_ASSERT_EQUAL(m_close_reason, reason);
}

/********** Test Cases **********/

void test_link_management(void)
{
    const char * URI = "hello";
    uint16_t oob_info_sources = NRF_MESH_PROV_OOB_INFO_SOURCE_NUMBER | NRF_MESH_PROV_OOB_INFO_SOURCE_INSIDE_MANUAL;


    /* Start as provisionee */
    LISTEN_START_EXPECT_AND_RETURN(&m_bearer, URI, oob_info_sources, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, m_interface.listen_start(&m_bearer, URI, oob_info_sources, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));

    LISTEN_START_EXPECT_AND_RETURN(&m_bearer, URI, oob_info_sources, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, m_interface.listen_start(&m_bearer, URI, oob_info_sources, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));

    /* Start as provisioner */
    uint8_t UUID[NRF_MESH_UUID_SIZE] = {};

    LINK_OPEN_EXPECT_AND_RETURN(&m_bearer, UUID, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, m_interface.link_open(&m_bearer, UUID, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));

    LINK_OPEN_EXPECT_AND_RETURN(&m_bearer, UUID, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, m_interface.link_open(&m_bearer, UUID, NRF_MESH_PROV_LINK_TIMEOUT_MIN_US));

    /* Close link */
    LINK_CLOSE_EXPECT(&m_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
    m_interface.link_close(&m_bearer, NRF_MESH_PROV_LINK_CLOSE_REASON_ERROR);
}

void test_prov_tx(void)
{

    /* Invite */
    prov_pdu_invite_t invite_pdu;
    invite_pdu.pdu_type = PROV_PDU_TYPE_INVITE;
    invite_pdu.attention_duration_s = 0x29; //arbitrary 8 bit number
    uint8_t confirmation_inputs[17];
    memset(confirmation_inputs, 0xFE, 17);

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &invite_pdu, 2, 2, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_invite(&m_bearer, 0x29, confirmation_inputs));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &invite_pdu, 2, 2, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_invite(&m_bearer, 0x29, confirmation_inputs));
    TEST_ASSERT_EQUAL_HEX8(0x29, confirmation_inputs[0]);

    /* capabilities */
    prov_pdu_caps_t capabilities_pdu;
    capabilities_pdu.pdu_type = PROV_PDU_TYPE_CAPABILITIES;
    capabilities_pdu.algorithms = 0x1234;
    capabilities_pdu.num_elements = 99;
    capabilities_pdu.oob_input_actions = 0x5678;
    capabilities_pdu.oob_input_size = 88;
    capabilities_pdu.oob_output_actions = 0x9ABC;
    capabilities_pdu.oob_output_size = 12;
    capabilities_pdu.oob_static_types = 11;
    capabilities_pdu.pubkey_type = 32;

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &capabilities_pdu, 12, 12, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_capabilities(&m_bearer, &capabilities_pdu, confirmation_inputs));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &capabilities_pdu, 12, 12, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_capabilities(&m_bearer, &capabilities_pdu, confirmation_inputs));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&((uint8_t *) &capabilities_pdu)[1], &confirmation_inputs[1], 11);

    /* start */
    prov_pdu_prov_start_t start_pdu;
    start_pdu.pdu_type = PROV_PDU_TYPE_START;
    start_pdu.algorithm = 0x12;
    start_pdu.auth_action = 99;
    start_pdu.auth_method = 0x34;
    start_pdu.auth_size = 88;
    start_pdu.public_key = 0x9A;

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &start_pdu, 6, 6, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_start(&m_bearer, &start_pdu, confirmation_inputs));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &start_pdu, 6, 6, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_start(&m_bearer, &start_pdu, confirmation_inputs));

    /* Ensure that the confirmation value is correct */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&((uint8_t *) &start_pdu)[1], &confirmation_inputs[12], 5);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&((uint8_t *) &capabilities_pdu)[1], &confirmation_inputs[1], 11);
    TEST_ASSERT_EQUAL_HEX8(0x29, confirmation_inputs[0]);

    /* public_key */
    uint8_t pubkey[NRF_MESH_ECDH_PUBLIC_KEY_SIZE];
    memset(pubkey, 0xAB, NRF_MESH_ECDH_PUBLIC_KEY_SIZE);
    prov_pdu_pubkey_t pubkey_pdu;
    pubkey_pdu.pdu_type = PROV_PDU_TYPE_PUBLIC_KEY;
    memcpy(pubkey_pdu.public_key, pubkey, NRF_MESH_ECDH_PUBLIC_KEY_SIZE);

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &pubkey_pdu, 65, 65, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_public_key(&m_bearer, pubkey));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &pubkey_pdu, 65, 65, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_public_key(&m_bearer, pubkey));

    /* input complete */
    prov_pdu_input_complete_t input_complete_pdu;
    input_complete_pdu.pdu_type = PROV_PDU_TYPE_INPUT_COMPLETE;

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &input_complete_pdu, 1, 1, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_input_complete(&m_bearer));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &input_complete_pdu, 1, 1, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_input_complete(&m_bearer));

    /* confirmation */
    prov_pdu_confirm_t confirmation_pdu;
    confirmation_pdu.pdu_type = PROV_PDU_TYPE_CONFIRMATION;
    memset(confirmation_pdu.confirmation, 0xCF, 16);

    uint8_t confirmation[16];
    memset(confirmation, 0xCF, 16);

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &confirmation_pdu, 17, 17, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_confirmation(&m_bearer, confirmation));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &confirmation_pdu, 17, 17, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_confirmation(&m_bearer, confirmation));

    /* random */
    prov_pdu_random_t random_pdu;
    random_pdu.pdu_type = PROV_PDU_TYPE_RANDOM;
    memset(random_pdu.random, 0xD3, 16);
    uint8_t random[16];
    memcpy(random, random_pdu.random, 16);

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &random_pdu, 17, 17, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_random(&m_bearer, random));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &random_pdu, 17, 17, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_random(&m_bearer, random));

    /* data */
    prov_pdu_data_t data_pdu;
    data_pdu.pdu_type = PROV_PDU_TYPE_DATA;
    memset(data_pdu.mic, 0x1C, PROV_PDU_DATA_MIC_LENGTH);
    memset(&data_pdu.data, 0xDA, sizeof(data_pdu.data));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &data_pdu, 34, 34, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_data(&m_bearer, &data_pdu));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &data_pdu, 34, 34, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_data(&m_bearer, &data_pdu));

    /* complete */
    prov_pdu_complete_t complete_pdu;
    complete_pdu.pdu_type = PROV_PDU_TYPE_COMPLETE;

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &complete_pdu, 1, 1, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_complete(&m_bearer));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &complete_pdu, 1, 1, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_complete(&m_bearer));

    /* failed */
    prov_pdu_failed_t failed_pdu;
    failed_pdu.pdu_type = PROV_PDU_TYPE_FAILED;
    failed_pdu.failure_code = 0x43; //arbitrary 8bit number

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &failed_pdu, 2, 2, ARBITRARY_ERROR);
    TEST_ASSERT_EQUAL_HEX32(ARBITRARY_ERROR, prov_tx_failed(&m_bearer, (nrf_mesh_prov_failure_code_t) 0x43));

    TX_EXPECT_WITH_DATA_ARRAY_AND_RETURN(&m_bearer, (const void *) &failed_pdu, 2, 2, NRF_SUCCESS);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_tx_failed(&m_bearer, (nrf_mesh_prov_failure_code_t) 0x43));
}

void test_callbacks(void)
{

    /* Link opened */
    m_cb_link_opened_expected = 1;
    m_callbacks.opened(&m_bearer);
    TEST_ASSERT_EQUAL(0, m_cb_link_opened_expected);

    /* Link closed */
    m_close_reason = (nrf_mesh_prov_link_close_reason_t) 0x43;
    m_cb_link_closed_expected = 1;
    m_callbacks.closed(&m_bearer, (nrf_mesh_prov_link_close_reason_t) 0x43);
    TEST_ASSERT_EQUAL(0, m_cb_link_closed_expected);

    /* Packet in */
    uint8_t data_packet[40];
    m_cb_rx_expected = 1;
    mp_rx_pointer = data_packet;
    m_rx_length = 40;
    m_callbacks.rx(&m_bearer, data_packet, 40);
    TEST_ASSERT_EQUAL(0, m_cb_rx_expected);

    /* ACK in */
    m_cb_ack_expected = 1;
    m_callbacks.ack(&m_bearer);
    TEST_ASSERT_EQUAL(0, m_cb_ack_expected);
}

void test_length_check(void)
{
    uint32_t length[PROV_PDU_TYPE_COUNT];

    /* Lengths from @tagMeshSp sample data, including the opcode field */
    length[PROV_PDU_TYPE_INVITE]         = 2;
    length[PROV_PDU_TYPE_CAPABILITIES]   = 12;
    length[PROV_PDU_TYPE_START]          = 6;
    length[PROV_PDU_TYPE_PUBLIC_KEY]     = 65;
    length[PROV_PDU_TYPE_INPUT_COMPLETE] = 1;
    length[PROV_PDU_TYPE_CONFIRMATION]   = 17;
    length[PROV_PDU_TYPE_RANDOM]         = 17;
    length[PROV_PDU_TYPE_DATA]           = 34;
    length[PROV_PDU_TYPE_COMPLETE]       = 1;
    length[PROV_PDU_TYPE_FAILED]         = 2;

    for (prov_pdu_type_t pdu = (prov_pdu_type_t) 0; pdu < PROV_PDU_TYPE_COUNT; pdu++)
    {
        TEST_ASSERT_EQUAL(false, prov_packet_length_valid(NULL, 0));
        TEST_ASSERT_EQUAL(false, prov_packet_length_valid(NULL, length[(uint32_t) pdu]));
        TEST_ASSERT_EQUAL(false, prov_packet_length_valid((uint8_t *) &pdu, 0));
        TEST_ASSERT_EQUAL(false, prov_packet_length_valid((uint8_t *) &pdu, length[(uint32_t) pdu] - 1));
        TEST_ASSERT_EQUAL(false, prov_packet_length_valid((uint8_t *) &pdu, length[(uint32_t) pdu] + 1));
        TEST_ASSERT_EQUAL(false, prov_packet_length_valid((uint8_t *) &pdu, 0xFFFF));
        TEST_ASSERT_EQUAL(false, prov_packet_length_valid((uint8_t *) &pdu, 0xFF));
        TEST_ASSERT_EQUAL(true, prov_packet_length_valid((uint8_t *) &pdu, length[(uint32_t) pdu]));
    }
}

void test_prov_data_check(void)
{
    nrf_mesh_prov_provisioning_data_t prov_data = {
        .netkey = {0},
        .netkey_index = NRF_MESH_GLOBAL_KEY_INDEX_MAX + 1,
        .iv_index = 0,
        .address = 0,
        .flags = {
            .iv_update = 0,
            .key_refresh = 0
        }
    };

    /* Invalid address and netkey_index. */
    TEST_ASSERT_FALSE(prov_data_is_valid(&prov_data));

    /* Invalid address */
    prov_data.netkey_index = NRF_MESH_GLOBAL_KEY_INDEX_MAX;
    TEST_ASSERT_FALSE(prov_data_is_valid(&prov_data));

    /* Valid address and netkey_index => success! */
    prov_data.address = 1;
    TEST_ASSERT_TRUE(prov_data_is_valid(&prov_data));
}

void test_prov_address_check(void)
{
    nrf_mesh_prov_provisioning_data_t prov_data = {
        .netkey = {0},
        .netkey_index = NRF_MESH_GLOBAL_KEY_INDEX_MAX,
        .iv_index = 0,
        .address = 0,
        .flags = {
            .iv_update = 0,
            .key_refresh = 0
        }
    };

    /* Invalid address. */
    TEST_ASSERT_FALSE(prov_address_is_valid(&prov_data, 1));

    /* Largest possible unicast address */
    prov_data.address = 0x7FFF;
    TEST_ASSERT_TRUE(prov_address_is_valid(&prov_data, 1));

    /* Overflow */
    prov_data.address = 0x7FFE;
    TEST_ASSERT_FALSE(prov_address_is_valid(&prov_data, 3));
}
