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
#include "transport.h"

#include "unity.h"
#include "cmock.h"

#include "utils.h"
#include "packet_mesh.h"

#include "bearer_event_mock.h"
#include "network_mock.h"
#include "nrf_mesh_mock.h"
#include "enc_mock.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "event_mock.h"
#include "replay_cache_mock.h"
#include "nrf_mesh_externs_mock.h"
#include "core_tx_mock.h"
#include "net_state_mock.h"
#include "mesh_mem_mock.h"

#define BEARER_FLAG 0x12345678
#define TX_TOKEN    (nrf_mesh_tx_token_t) 0xABCDEF

static nrf_mesh_rx_metadata_t m_rx_meta;
static nrf_mesh_network_secmat_t m_net_secmat;
static bool is_network_allocation_count_checked;

void setUp(void)
{
    bearer_event_mock_Init();
    network_mock_Init();
    nrf_mesh_mock_Init();
    enc_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
    event_mock_Init();
    replay_cache_mock_Init();
    nrf_mesh_externs_mock_Init();
    core_tx_mock_Init();
    net_state_mock_Init();
    mesh_mem_mock_Init();

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();

    is_network_allocation_count_checked = true;
}

void tearDown(void)
{
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();

    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    replay_cache_mock_Verify();
    replay_cache_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    mesh_mem_mock_Verify();
    mesh_mem_mock_Destroy();
}

static transport_control_packet_t m_expected_control_packet;
static uint32_t m_expected_control_packet_handler;
static void control_packet_handler(const transport_control_packet_t * p_rx_packet, const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    TEST_ASSERT_EQUAL_PTR(&m_rx_meta, p_rx_metadata);
    TEST_ASSERT_EQUAL_HEX8(m_expected_control_packet.opcode, p_rx_packet->opcode);
    TEST_ASSERT_EQUAL_PTR(m_expected_control_packet.p_net_secmat, p_rx_packet->p_net_secmat);
    TEST_ASSERT_EQUAL(m_expected_control_packet.dst.type, p_rx_packet->dst.type);
    TEST_ASSERT_EQUAL(m_expected_control_packet.dst.value, p_rx_packet->dst.value);
    TEST_ASSERT_EQUAL(m_expected_control_packet.dst.p_virtual_uuid, p_rx_packet->dst.p_virtual_uuid);
    TEST_ASSERT_EQUAL(m_expected_control_packet.src, p_rx_packet->src);
    TEST_ASSERT_EQUAL(m_expected_control_packet.ttl, p_rx_packet->ttl);
    TEST_ASSERT_EQUAL(m_expected_control_packet.reliable, p_rx_packet->reliable);
    TEST_ASSERT_EQUAL(m_expected_control_packet.data_len, p_rx_packet->data_len);
    TEST_ASSERT_EQUAL_PTR(m_expected_control_packet.p_data, p_rx_packet->p_data);
    TEST_ASSERT_TRUE(m_expected_control_packet_handler > 0);
    m_expected_control_packet_handler--;
}

static struct
{
    network_packet_metadata_t net_meta;
    nrf_mesh_tx_token_t tx_token;
    uint32_t payload_len;

    uint8_t * p_buffer;
    uint32_t retval;
    uint32_t calls;
    core_tx_bearer_selector_t bearer;
    network_tx_packet_buffer_t * p_tx_buffer;
} m_expect_network_packet_alloc;

static uint32_t network_packet_alloc_callback(network_tx_packet_buffer_t * p_buf, int calls)
{
    if (is_network_allocation_count_checked)
    {
        TEST_ASSERT_TRUE(m_expect_network_packet_alloc.calls > 0);
    }

    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.payload_len, p_buf->user_data.payload_len);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.tx_token, p_buf->user_data.token);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.bearer, p_buf->user_data.bearer_selector);

    TEST_ASSERT_NOT_NULL(p_buf->user_data.p_metadata);
    TEST_ASSERT_EQUAL_HEX16(m_expect_network_packet_alloc.net_meta.src, p_buf->user_data.p_metadata->src);
    TEST_ASSERT_EQUAL_HEX16(m_expect_network_packet_alloc.net_meta.dst.value, p_buf->user_data.p_metadata->dst.value);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.net_meta.dst.type, p_buf->user_data.p_metadata->dst.type);
    TEST_ASSERT_EQUAL_PTR(m_expect_network_packet_alloc.net_meta.dst.p_virtual_uuid, p_buf->user_data.p_metadata->dst.p_virtual_uuid);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.net_meta.ttl, p_buf->user_data.p_metadata->ttl);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.net_meta.control_packet, p_buf->user_data.p_metadata->control_packet);
    TEST_ASSERT_EQUAL_PTR(m_expect_network_packet_alloc.net_meta.p_security_material, p_buf->user_data.p_metadata->p_security_material);

    p_buf->user_data.role = CORE_TX_ROLE_ORIGINATOR;
    p_buf->p_payload = m_expect_network_packet_alloc.p_buffer;

    m_expect_network_packet_alloc.p_tx_buffer = p_buf;

    network_packet_send_Expect(m_expect_network_packet_alloc.p_tx_buffer);
    m_expect_network_packet_alloc.calls--;
    return m_expect_network_packet_alloc.retval;
}

static void expect_init(void)
{
    replay_cache_init_Expect();
    bearer_event_flag_add_ExpectAnyArgsAndReturn(BEARER_FLAG);
    core_tx_complete_cb_set_ExpectAnyArgs();
}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_control_handlers(void)
{
    expect_init();
    transport_init();
    replay_cache_has_elem_IgnoreAndReturn(false);
    replay_cache_add_IgnoreAndReturn(NRF_SUCCESS);
    /* register a control packet consumer */
    const transport_control_packet_handler_t handlers[] = {
        {TRANSPORT_CONTROL_OPCODE_HEARTBEAT, control_packet_handler},
        {TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE, control_packet_handler},
        {0x50, control_packet_handler}, /* outside the enum, shouldn't matter */
    };

    const uint32_t control_payload_len = 9;
    packet_mesh_trs_packet_t transport_packet;

    m_expected_control_packet.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    m_expected_control_packet.dst.value = 0x0001;
    m_expected_control_packet.dst.p_virtual_uuid = NULL;
    m_expected_control_packet.src = 0x0004;
    m_expected_control_packet.p_net_secmat = &m_net_secmat;
    m_expected_control_packet.data_len = control_payload_len;
    m_expected_control_packet.p_data = (const packet_mesh_trs_control_packet_t *) packet_mesh_trs_unseg_payload_get(&transport_packet);

    network_packet_metadata_t net_meta;
    net_meta.dst = m_expected_control_packet.dst;
    net_meta.control_packet = true;
    net_meta.src = m_expected_control_packet.src;
    net_meta.ttl = m_expected_control_packet.ttl;
    net_meta.p_security_material = m_expected_control_packet.p_net_secmat;

    packet_mesh_trs_common_seg_set(&transport_packet, false);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_control_packet_consumer_add(handlers, ARRAY_SIZE(handlers)));

    for (uint32_t i = 0; i < ARRAY_SIZE(handlers); ++i)
    {
        m_expected_control_packet_handler = 1;
        packet_mesh_trs_control_opcode_set(&transport_packet, handlers[i].opcode);
        m_expected_control_packet.opcode = handlers[i].opcode;
        nrf_mesh_rx_address_get_ExpectAndReturn(0x0001, NULL, true);
        nrf_mesh_rx_address_get_IgnoreArg_p_address();
        nrf_mesh_rx_address_get_ReturnThruPtr_p_address(&m_expected_control_packet.dst);
        TEST_ASSERT_EQUAL(NRF_SUCCESS,
                        transport_packet_in(&transport_packet,
                                            PACKET_MESH_TRS_UNSEG_PDU_OFFSET +
                                            control_payload_len,
                                            &net_meta,
                                            &m_rx_meta));
        TEST_ASSERT_EQUAL(0, m_expected_control_packet_handler);
    }

    /* run on an unregistered opcode */
    packet_mesh_trs_control_opcode_set(&transport_packet, TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR);
    nrf_mesh_rx_address_get_ExpectAndReturn(0x0001, NULL, true);
    nrf_mesh_rx_address_get_IgnoreArg_p_address();
    nrf_mesh_rx_address_get_ReturnThruPtr_p_address(&m_expected_control_packet.dst);
    TEST_ASSERT_EQUAL(NRF_SUCCESS,
                    transport_packet_in(&transport_packet,
                                        PACKET_MESH_TRS_UNSEG_PDU_OFFSET +
                                        control_payload_len,
                                        &net_meta,
                                        &m_rx_meta));
    TEST_ASSERT_EQUAL(0, m_expected_control_packet_handler);


    /* Overflow consumer count */
    transport_control_packet_handler_t overflow_handler = {
        0x60,
        control_packet_handler
    };
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, transport_control_packet_consumer_add(&overflow_handler, 1));

    /* reset handler array */
    expect_init();
    transport_init();

    /* builtin opcode, considered duplicate: */
    transport_control_packet_handler_t duplicate_handler = {
        TRANSPORT_CONTROL_OPCODE_SEGACK,
        control_packet_handler
    };
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, transport_control_packet_consumer_add(&duplicate_handler, 1));

    /* NULL pointer function */
    transport_control_packet_handler_t null_ptr_handler = {
        0x60, /* legal */
        NULL
    };
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, transport_control_packet_consumer_add(&null_ptr_handler, 1));

    /* NULL pointer array */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, transport_control_packet_consumer_add(&null_ptr_handler, 1));

    /* out of bounds opcode */
    transport_control_packet_handler_t out_of_bounds_handler = {
        0x80, /* out of bounds */
        control_packet_handler
    };
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, transport_control_packet_consumer_add(&out_of_bounds_handler, 1));
}

void test_control_tx(void)
{
    expect_init();
    transport_init();
    uint8_t control_packet_buffer[64];

    for (uint32_t i = 0; i < sizeof(control_packet_buffer); ++i)
    {
        /* fill buffer with bogus data */
        control_packet_buffer[i] = i;
    }

    uint8_t network_packet_buffer[64] = {0};
    transport_control_packet_t control_packet;
    nrf_mesh_network_secmat_t net_secmat;

    /* Send a single segment control packet */
    control_packet.data_len           = 8;
    control_packet.dst.p_virtual_uuid = NULL;
    control_packet.dst.value          = 0x0001;
    control_packet.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    control_packet.opcode             = TRANSPORT_CONTROL_OPCODE_HEARTBEAT;
    control_packet.p_data             = (const packet_mesh_trs_control_packet_t *) control_packet_buffer;
    control_packet.p_net_secmat       = &net_secmat;
    control_packet.reliable           = false;
    control_packet.src                = 0x0002;
    control_packet.ttl                = 9;
    control_packet.bearer_selector    = CORE_TX_BEARER_TYPE_LOW_POWER;

    m_expect_network_packet_alloc.calls                        = 1;
    m_expect_network_packet_alloc.net_meta.control_packet      = true;
    m_expect_network_packet_alloc.net_meta.dst                 = control_packet.dst;
    m_expect_network_packet_alloc.net_meta.src                 = control_packet.src;
    m_expect_network_packet_alloc.net_meta.ttl                 = control_packet.ttl;
    m_expect_network_packet_alloc.net_meta.p_security_material = control_packet.p_net_secmat;
    m_expect_network_packet_alloc.payload_len                  = 1 + control_packet.data_len;
    m_expect_network_packet_alloc.tx_token                     = TX_TOKEN;
    m_expect_network_packet_alloc.p_buffer                     = network_packet_buffer;
    m_expect_network_packet_alloc.bearer                       = control_packet.bearer_selector;

    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_control_tx(&control_packet, TX_TOKEN));

    TEST_ASSERT_EQUAL_HEX8(control_packet.opcode, network_packet_buffer[0]); /* opcode */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(control_packet_buffer, &network_packet_buffer[1], control_packet.data_len); /* payload */
}

void test_duplicate_sar_tx(void)
{
    expect_init();
    transport_init();
    nrf_mesh_network_secmat_t net_secmat;
    nrf_mesh_application_secmat_t app_secmat = {0};

    uint8_t buffer[PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE];

    nrf_mesh_tx_params_t tx_params;
    tx_params.data_len           = sizeof(buffer);
    tx_params.dst.p_virtual_uuid = NULL;
    tx_params.dst.value          = 0x0001;
    tx_params.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    tx_params.p_data             = buffer;
    tx_params.security_material.p_net = &net_secmat;
    tx_params.security_material.p_app = &app_secmat;
    tx_params.force_segmented    = true;
    tx_params.src                = 0x0002;
    tx_params.ttl                = 9;
    tx_params.transmic_size      = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    net_state_iv_index_lock_Ignore();
    enc_nonce_generate_Ignore();
    enc_aes_ccm_encrypt_Ignore();
    timer_now_IgnoreAndReturn(0);
    timer_sch_reschedule_Ignore();

    uint8_t ctx_payload[PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE + sizeof(buffer)];
    mesh_mem_alloc_ExpectAndReturn(PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE + sizeof(buffer), ctx_payload);

    network_tx_packet_buffer_t packet_buffer;
    packet_mesh_net_packet_t net_buffer;
    packet_buffer.p_payload = net_buffer.pdu;
    network_packet_alloc_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    network_packet_alloc_ReturnMemThruPtr_p_buffer(&packet_buffer, sizeof(packet_buffer));
    network_packet_send_Expect(&packet_buffer);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, false);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    // Allocating again with the same src+dst should result in FORBIDDEN:
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, false);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, transport_tx(&tx_params, NULL));
}

void test_unseg_tx(void)
{
    expect_init();
    transport_init();
    nrf_mesh_network_secmat_t net_secmat;
    nrf_mesh_application_secmat_t app_secmat = {};

    uint8_t buffer[PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE];

    nrf_mesh_tx_params_t tx_params;
    tx_params.data_len           = sizeof(buffer);
    tx_params.dst.p_virtual_uuid = NULL;
    tx_params.dst.value          = 0x0001;
    tx_params.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    tx_params.p_data             = buffer;
    tx_params.security_material.p_net = &net_secmat;
    tx_params.security_material.p_app = &app_secmat;
    tx_params.force_segmented    = false;
    tx_params.src                = 0x0002;
    tx_params.ttl                = 9;
    tx_params.tx_token           = TX_TOKEN;
    tx_params.transmic_size      = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    net_state_iv_index_lock_Ignore();
    enc_nonce_generate_Ignore();
    enc_aes_ccm_encrypt_Ignore();
    timer_now_IgnoreAndReturn(0);
    timer_sch_reschedule_Ignore();

    network_tx_packet_buffer_t packet_buffer;
    packet_mesh_net_packet_t net_buffer;
    packet_buffer.p_payload = net_buffer.pdu;

    m_expect_network_packet_alloc.calls                        = 1;
    m_expect_network_packet_alloc.net_meta.control_packet      = false;
    m_expect_network_packet_alloc.net_meta.dst                 = tx_params.dst;
    m_expect_network_packet_alloc.net_meta.src                 = tx_params.src;
    m_expect_network_packet_alloc.net_meta.ttl                 = tx_params.ttl;
    m_expect_network_packet_alloc.net_meta.p_security_material = &net_secmat;
    m_expect_network_packet_alloc.payload_len                  = PACKET_MESH_TRS_UNSEG_PDU_OFFSET + tx_params.data_len + PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE;
    m_expect_network_packet_alloc.tx_token                     = TX_TOKEN;
    m_expect_network_packet_alloc.p_buffer                     = (uint8_t *) &packet_buffer;
    m_expect_network_packet_alloc.bearer                       = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;

    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, false);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));
}

void test_core_tx_selection(void)
{
    // this tests selection of core_tx for access data considering dst address
    printf("The test checks selection of core_tx for access data considering dst address\n");

    expect_init();
    transport_init();
    nrf_mesh_network_secmat_t net_secmat;
    nrf_mesh_application_secmat_t app_secmat = {};

    uint8_t buffer[PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE];
    uint8_t uuid[16];
    is_network_allocation_count_checked = false;

    nrf_mesh_tx_params_t tx_params;
    tx_params.data_len           = sizeof(buffer);
    tx_params.dst.p_virtual_uuid = NULL;
    tx_params.dst.value          = 0x0001;
    tx_params.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    tx_params.p_data             = buffer;
    tx_params.security_material.p_net = &net_secmat;
    tx_params.security_material.p_app = &app_secmat;
    tx_params.force_segmented    = false;
    tx_params.src                = 0x0002;
    tx_params.ttl                = 9;
    tx_params.tx_token           = TX_TOKEN;
    tx_params.transmic_size      = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    net_state_iv_index_lock_Ignore();
    enc_nonce_generate_Ignore();
    enc_aes_ccm_encrypt_Ignore();
    timer_now_IgnoreAndReturn(0);
    timer_sch_reschedule_Ignore();

    network_tx_packet_buffer_t packet_buffer;
    packet_mesh_net_packet_t net_buffer;
    packet_buffer.p_payload = net_buffer.pdu;

    m_expect_network_packet_alloc.calls                        = 1;
    m_expect_network_packet_alloc.net_meta.control_packet      = false;
    m_expect_network_packet_alloc.net_meta.dst                 = tx_params.dst;
    m_expect_network_packet_alloc.net_meta.src                 = tx_params.src;
    m_expect_network_packet_alloc.net_meta.ttl                 = tx_params.ttl;
    m_expect_network_packet_alloc.net_meta.p_security_material = &net_secmat;
    m_expect_network_packet_alloc.payload_len                  = PACKET_MESH_TRS_UNSEG_PDU_OFFSET + tx_params.data_len + PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE;
    m_expect_network_packet_alloc.tx_token                     = TX_TOKEN;
    m_expect_network_packet_alloc.p_buffer                     = (uint8_t *) &packet_buffer;
    m_expect_network_packet_alloc.bearer                       = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;

    printf("1. Not own unicast\n");
    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, false);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    printf("2. Own unicast\n");
    m_expect_network_packet_alloc.bearer = CORE_TX_BEARER_TYPE_LOCAL;
    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    printf("3. Not own virtual\n");
    tx_params.dst.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    tx_params.dst.p_virtual_uuid = uuid;
    m_expect_network_packet_alloc.net_meta.dst = tx_params.dst;
    m_expect_network_packet_alloc.bearer = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;
    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, false);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    printf("4. Own virtual\n");
    tx_params.dst.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    tx_params.dst.p_virtual_uuid = uuid;
    m_expect_network_packet_alloc.net_meta.dst = tx_params.dst;
    m_expect_network_packet_alloc.bearer = CORE_TX_BEARER_TYPE_ALLOW_ALL;
    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    printf("5. Not own group\n");
    tx_params.dst.type = NRF_MESH_ADDRESS_TYPE_GROUP;
    m_expect_network_packet_alloc.net_meta.dst = tx_params.dst;
    m_expect_network_packet_alloc.bearer = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;
    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, false);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    printf("6. Own group\n");
    tx_params.dst.type = NRF_MESH_ADDRESS_TYPE_GROUP;
    m_expect_network_packet_alloc.bearer = CORE_TX_BEARER_TYPE_ALLOW_ALL;
    m_expect_network_packet_alloc.net_meta.dst = tx_params.dst;
    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    nrf_mesh_is_address_rx_ExpectAndReturn(&tx_params.dst, true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));
}

/**
 * Test the various micsize configurations and parameter values, and ensure that the packet
 * segmentation and micsize is done according to the documentation.
 */
void test_segmentation_and_micsize_rules(void)
{
    nrf_mesh_network_secmat_t net_secmat;
    nrf_mesh_application_secmat_t app_secmat = {};
    uint8_t * p_ctx_payload = NULL;

    uint8_t buffer[NRF_MESH_SEG_PAYLOAD_SIZE_MAX];
    is_network_allocation_count_checked = false;

    nrf_mesh_tx_params_t tx_params;

    tx_params.dst.p_virtual_uuid = NULL;
    tx_params.dst.value          = 0x0001;
    tx_params.dst.type           = NRF_MESH_ADDRESS_TYPE_UNICAST;
    tx_params.p_data             = buffer;
    tx_params.security_material.p_net = &net_secmat;
    tx_params.security_material.p_app = &app_secmat;
    tx_params.src                = 0x0002;
    tx_params.ttl                = 9;
    tx_params.tx_token           = TX_TOKEN;


    net_state_iv_index_lock_Ignore();
    enc_nonce_generate_Ignore();
    enc_aes_ccm_encrypt_Ignore();
    timer_now_IgnoreAndReturn(0);
    timer_sch_reschedule_Ignore();
    nrf_mesh_is_address_rx_IgnoreAndReturn(false);
    network_packet_send_Ignore();

    uint8_t net_payload[PACKET_MESH_NET_MAX_SIZE];
    network_packet_metadata_t net_meta; // dummy
    network_tx_packet_buffer_t net_buf;
    net_buf.user_data.role = CORE_TX_ROLE_ORIGINATOR;
    net_buf.user_data.token = TX_TOKEN;
    net_buf.user_data.p_metadata = &net_meta;
    net_buf.user_data.bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;
    net_buf.p_payload = net_payload;

    /* Run test vectors for various combinations.
     *
     * @note The expected mic size should take the micsize_param if it's a segmented message, and
     * small micsize for unsegmented messages. If the param is DEFAULT, we should rely on the
     * configured value.
     */
    static const struct
    {
        nrf_mesh_transmic_size_t micsize_config;
        bool force_segmented;
        nrf_mesh_transmic_size_t micsize_param;
        uint32_t data_len;

        struct
        {
            uint8_t mic_size;
            bool segmented;
        } expected;
    } test_vector[] = {
        /* No conditions triggering: */
        {NRF_MESH_TRANSMIC_SIZE_SMALL, false, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 5,    .expected = {4, false}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, false, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 5,    .expected = {4, false}},
        {NRF_MESH_TRANSMIC_SIZE_SMALL, false, NRF_MESH_TRANSMIC_SIZE_SMALL, 5,      .expected = {4, false}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, false, NRF_MESH_TRANSMIC_SIZE_SMALL, 5,      .expected = {4, false}},
        /* Explicitly require large micsize: */
        {NRF_MESH_TRANSMIC_SIZE_SMALL, false, NRF_MESH_TRANSMIC_SIZE_LARGE, 5,      .expected = {8, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, false, NRF_MESH_TRANSMIC_SIZE_LARGE, 5,      .expected = {8, true}},
        /* Packet can't fit: */
        {NRF_MESH_TRANSMIC_SIZE_SMALL, false, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 15,   .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, false, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 15,   .expected = {8, true}},
        {NRF_MESH_TRANSMIC_SIZE_SMALL, false, NRF_MESH_TRANSMIC_SIZE_SMALL, 15,     .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, false, NRF_MESH_TRANSMIC_SIZE_SMALL, 15,     .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_SMALL, false, NRF_MESH_TRANSMIC_SIZE_LARGE, 15,     .expected = {8, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, false, NRF_MESH_TRANSMIC_SIZE_LARGE, 15,     .expected = {8, true}},
        /* Forced segmentation: */
        {NRF_MESH_TRANSMIC_SIZE_SMALL, true, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 5,    .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, true, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 5,    .expected = {8, true}},
        {NRF_MESH_TRANSMIC_SIZE_SMALL, true, NRF_MESH_TRANSMIC_SIZE_SMALL, 5,      .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, true, NRF_MESH_TRANSMIC_SIZE_SMALL, 5,      .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_SMALL, true, NRF_MESH_TRANSMIC_SIZE_LARGE, 5,      .expected = {8, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, true, NRF_MESH_TRANSMIC_SIZE_LARGE, 5,      .expected = {8, true}},

        {NRF_MESH_TRANSMIC_SIZE_SMALL, true, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 15,   .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, true, NRF_MESH_TRANSMIC_SIZE_DEFAULT, 15,   .expected = {8, true}},
        {NRF_MESH_TRANSMIC_SIZE_SMALL, true, NRF_MESH_TRANSMIC_SIZE_SMALL, 15,     .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, true, NRF_MESH_TRANSMIC_SIZE_SMALL, 15,     .expected = {4, true}},
        {NRF_MESH_TRANSMIC_SIZE_SMALL, true, NRF_MESH_TRANSMIC_SIZE_LARGE, 15,     .expected = {8, true}},
        {NRF_MESH_TRANSMIC_SIZE_LARGE, true, NRF_MESH_TRANSMIC_SIZE_LARGE, 15,     .expected = {8, true}},
    };

    for (uint32_t i = 0; i < ARRAY_SIZE(test_vector); ++i)
    {
        expect_init();
        transport_init();

        nrf_mesh_opt_t opt;
        opt.len = 1;
        opt.opt.val = test_vector[i].micsize_config;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_opt_set(NRF_MESH_OPT_TRS_SZMIC, &opt));

        tx_params.transmic_size = test_vector[i].micsize_param;
        tx_params.data_len = test_vector[i].data_len;
        tx_params.force_segmented = test_vector[i].force_segmented;

        network_tx_packet_buffer_t segment_buffers[10]; // We need to separate the memory for each network alloc call, as CMock only stores the pointer.

        if (test_vector[i].expected.segmented)
        {
            net_buf.user_data.token = NRF_MESH_SAR_TOKEN;
            const uint32_t total_len = test_vector[i].data_len + test_vector[i].expected.mic_size;

            p_ctx_payload = malloc(total_len);
            TEST_ASSERT_NOT_NULL(p_ctx_payload);
            mesh_mem_alloc_ExpectAndReturn(total_len, p_ctx_payload);

            network_tx_packet_buffer_t * p_net_buf = &segment_buffers[0];

            for (uint32_t len = 0; len < total_len; len += PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE)
            {
                net_buf.user_data.payload_len = PACKET_MESH_TRS_SEG_PDU_OFFSET + MIN(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, total_len - len);
                *p_net_buf = net_buf;
                network_packet_alloc_ExpectAndReturn(p_net_buf, NRF_SUCCESS);
                network_packet_alloc_ReturnThruPtr_p_buffer(p_net_buf); // CMock stores the pointer only
                p_net_buf++;
                TEST_ASSERT_NOT_EQUAL(&segment_buffers[ARRAY_SIZE(segment_buffers)], p_net_buf); // just make sure we don't go out of bounds
            }
        }
        else
        {
            net_buf.user_data.token = TX_TOKEN;
            net_buf.user_data.payload_len = PACKET_MESH_TRS_UNSEG_PDU_OFFSET + test_vector[i].data_len + test_vector[i].expected.mic_size;
            network_packet_alloc_ExpectAndReturn(&net_buf, NRF_SUCCESS);
            network_packet_alloc_ReturnThruPtr_p_buffer(&net_buf);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));
        network_mock_Verify();

        if (p_ctx_payload != NULL)
        {
            free(p_ctx_payload);
            p_ctx_payload = NULL;
        }
    }
}
