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

#define BEARER_FLAG 0x12345678
#define TX_TOKEN    (nrf_mesh_tx_token_t) 0xABCDEF

static nrf_mesh_rx_metadata_t m_rx_meta;
static nrf_mesh_network_secmat_t m_net_secmat;

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

    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
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
    network_tx_packet_buffer_t * p_tx_buffer;
} m_expect_network_packet_alloc;
static uint32_t network_packet_alloc_callback(network_tx_packet_buffer_t * p_buf, int calls)
{
    TEST_ASSERT_TRUE(m_expect_network_packet_alloc.calls > 0);

    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.payload_len, p_buf->user_data.payload_len);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.tx_token, p_buf->user_data.token);

    TEST_ASSERT_NOT_NULL(p_buf->user_data.p_metadata);
    TEST_ASSERT_EQUAL_HEX16(m_expect_network_packet_alloc.net_meta.src, p_buf->user_data.p_metadata->src);
    TEST_ASSERT_EQUAL_HEX16(m_expect_network_packet_alloc.net_meta.dst.value, p_buf->user_data.p_metadata->dst.value);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.net_meta.dst.type, p_buf->user_data.p_metadata->dst.type);
    TEST_ASSERT_EQUAL_PTR(m_expect_network_packet_alloc.net_meta.dst.p_virtual_uuid, p_buf->user_data.p_metadata->dst.p_virtual_uuid);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.net_meta.ttl, p_buf->user_data.p_metadata->ttl);
    TEST_ASSERT_EQUAL(m_expect_network_packet_alloc.net_meta.control_packet, p_buf->user_data.p_metadata->control_packet);
    TEST_ASSERT_EQUAL_PTR(m_expect_network_packet_alloc.net_meta.p_security_material, p_buf->user_data.p_metadata->p_security_material);

    p_buf->role      = CORE_TX_ROLE_ORIGINATOR;
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
    transport_init(NULL);
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
    transport_init(NULL);

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
    transport_init(NULL);
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

    m_expect_network_packet_alloc.calls                        = 1;
    m_expect_network_packet_alloc.net_meta.control_packet      = true;
    m_expect_network_packet_alloc.net_meta.dst                 = control_packet.dst;
    m_expect_network_packet_alloc.net_meta.src                 = control_packet.src;
    m_expect_network_packet_alloc.net_meta.ttl                 = control_packet.ttl;
    m_expect_network_packet_alloc.net_meta.p_security_material = control_packet.p_net_secmat;
    m_expect_network_packet_alloc.payload_len                  = 1 + control_packet.data_len;
    m_expect_network_packet_alloc.tx_token                     = TX_TOKEN;
    m_expect_network_packet_alloc.p_buffer                     = network_packet_buffer;

    network_packet_alloc_StubWithCallback(network_packet_alloc_callback);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_control_tx(&control_packet, TX_TOKEN));

    TEST_ASSERT_EQUAL_HEX8(control_packet.opcode, network_packet_buffer[0]); /* opcode */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(control_packet_buffer, &network_packet_buffer[1], control_packet.data_len); /* payload */
}
