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

#include "core_tx_local.h"

#include "stdint.h"
#include "stdbool.h"

#include "unity.h"
#include "cmock.h"
#include "test_assert.h"
#include "core_tx_mock.h"
#include "bearer_event_mock.h"
#include "mesh_mem_mock.h"
#include "timer_mock.h"
#include "network_mock.h"

#define BE_LOOPBACK_FLAG   0x2aul
#define TX_TOKEN    0x12345678
#define TX_TIMESTAMP    0x56781234
#define NET_PACKET_LEN  29

#define ALLOCATED_PACKET_NUMBER 3

// it has to be sync with loopback_data_item_t in core_tx_local_t
typedef struct
{
    queue_elem_t        node;
    core_tx_role_t      role;
    nrf_mesh_tx_token_t token;
    uint32_t            length;
    uint8_t             data[];
} replica_loopback_data_item_t;

static bearer_event_flag_callback_t m_loopback_process;
static core_tx_bearer_t * mp_bearer;
static uint8_t m_net_packet_buf[NET_PACKET_LEN];
static uint8_t m_memory_leakage_checker;

static const network_packet_metadata_t m_net_meta =
{
    .control_packet = true,
    .ttl = 0,
    .src = 0x0001,
    .dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST,
    .dst.value = 0x0002,
};

static const core_tx_alloc_params_t m_alloc_params =
{
    .net_packet_len = NET_PACKET_LEN,
    .p_metadata = &m_net_meta,
    .token = TX_TOKEN,
    .role = CORE_TX_ROLE_ORIGINATOR,
    .bearer_selector = CORE_TX_BEARER_TYPE_LOCAL,
};

static bearer_event_flag_t bearer_event_flag_add_cb(bearer_event_flag_callback_t callback, int num_calls)
{
    m_loopback_process = callback;

    return BE_LOOPBACK_FLAG;
}

static void core_tx_bearer_add_cb(core_tx_bearer_t * p_bearer,
                                  const core_tx_bearer_interface_t * p_if,
                                  core_tx_bearer_type_t type,
                                  int num_calls)
{
    TEST_ASSERT_NOT_NULL(p_bearer);
    TEST_ASSERT_NOT_NULL(p_if);
    TEST_ASSERT_NOT_NULL(p_if->packet_alloc);
    TEST_ASSERT_NOT_NULL(p_if->packet_send);
    TEST_ASSERT_NOT_NULL(p_if->packet_discard);
    TEST_ASSERT_EQUAL(CORE_TX_BEARER_TYPE_LOCAL, type);
    TEST_ASSERT_EQUAL(0, num_calls);
    p_bearer->p_interface = p_if;
    mp_bearer = p_bearer;
}

static void* mesh_mem_alloc_success_cb(size_t size, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_EQUAL(NET_PACKET_LEN + sizeof(replica_loopback_data_item_t), size);
    m_memory_leakage_checker++;
    return malloc(size);
}

static void* mesh_mem_alloc_fail_cb(size_t size, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_EQUAL(NET_PACKET_LEN + sizeof(replica_loopback_data_item_t), size);

    return NULL;
}

static void mesh_mem_free_cb(void* ptr, int num_calls)
{
    (void)num_calls;
    m_memory_leakage_checker--;
    free(ptr);
}

static uint32_t network_packet_in_cb(const uint8_t * p_packet,
                                     uint32_t net_packet_len,
                                     const nrf_mesh_rx_metadata_t * p_rx_metadata,
                                     int num_calls)
{
    TEST_ASSERT_NOT_NULL(p_packet);
    TEST_ASSERT_NOT_NULL(p_rx_metadata);
    TEST_ASSERT_EQUAL(NET_PACKET_LEN, net_packet_len);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_net_packet_buf, p_packet, net_packet_len);
    TEST_ASSERT_EQUAL(NRF_MESH_RX_SOURCE_LOOPBACK, p_rx_metadata->source);
    TEST_ASSERT_EQUAL(TX_TOKEN, p_rx_metadata->params.loopback.tx_token);
    TEST_ASSERT_LESS_THAN(ALLOCATED_PACKET_NUMBER, num_calls);

    return NRF_SUCCESS;
}

void setUp(void)
{
    core_tx_mock_Init();
    bearer_event_mock_Init();
    mesh_mem_mock_Init();
    timer_mock_Init();
    network_mock_Init();
}

void tearDown(void)
{
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    mesh_mem_mock_Verify();
    mesh_mem_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
}

void test_init(void)
{
    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_cb);
    core_tx_bearer_add_StubWithCallback(core_tx_bearer_add_cb);

    core_tx_local_init();
    TEST_ASSERT_NOT_NULL(mp_bearer);
    TEST_ASSERT_NOT_NULL(m_loopback_process);
    TEST_ASSERT_EQUAL(0, m_memory_leakage_checker);
}

void test_alloc(void)
{
    test_init();
    core_tx_alloc_params_t alloc_params = m_alloc_params;

    // Allocate a packet on any bearers bearers except CORE_TX_BEARER_TYPE_LOCAL, should reject it
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL ^ CORE_TX_BEARER_TYPE_LOCAL;
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_REJECTED, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // lack of internal memory
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_LOCAL;
    mesh_mem_alloc_StubWithCallback(mesh_mem_alloc_fail_cb);
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_NO_MEM, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // should accept packet for local only
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_LOCAL;
    mesh_mem_alloc_StubWithCallback(mesh_mem_alloc_success_cb);
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // Can only allocate one packet at the time
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_REJECTED, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // stub to prevent test memory leakage
    mesh_mem_free_StubWithCallback(mesh_mem_free_cb);
    mp_bearer->p_interface->packet_discard(mp_bearer);
    TEST_ASSERT_EQUAL(0, m_memory_leakage_checker);
}

void test_dealloc(void)
{
    test_init();
    core_tx_alloc_params_t alloc_params = m_alloc_params;

    // allocate packet
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_LOCAL;
    mesh_mem_alloc_StubWithCallback(mesh_mem_alloc_success_cb);
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // successfully discard the current packet
    mesh_mem_free_StubWithCallback(mesh_mem_free_cb);
    mp_bearer->p_interface->packet_discard(mp_bearer);

    // Should assert if we try it again
    TEST_NRF_MESH_ASSERT_EXPECT(mp_bearer->p_interface->packet_discard(mp_bearer));
    TEST_ASSERT_EQUAL(0, m_memory_leakage_checker);
}

void test_send(void)
{
    test_init();
    core_tx_alloc_params_t alloc_params = m_alloc_params;

    for (uint8_t number = 0; number < ALLOCATED_PACKET_NUMBER; number++)
    {
        // allocate packet
        alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_LOCAL;
        mesh_mem_alloc_StubWithCallback(mesh_mem_alloc_success_cb);
        TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

        for (uint16_t i = 0; i < sizeof(m_net_packet_buf); i++)
        {
            m_net_packet_buf[i] = i;
        }

        bearer_event_flag_set_Expect(BE_LOOPBACK_FLAG);
        mp_bearer->p_interface->packet_send(mp_bearer, m_net_packet_buf, sizeof(m_net_packet_buf));

        // emulate bearer event behavior and run loopback process
        timer_now_ExpectAndReturn(TX_TIMESTAMP + number);
        core_tx_complete_Expect(mp_bearer, CORE_TX_ROLE_ORIGINATOR, TX_TIMESTAMP + number, TX_TOKEN);
    }
    network_packet_in_StubWithCallback(network_packet_in_cb);
    mesh_mem_free_StubWithCallback(mesh_mem_free_cb);
    m_loopback_process();

    // Should assert if we try to discard handled packet
    TEST_NRF_MESH_ASSERT_EXPECT(mp_bearer->p_interface->packet_discard(mp_bearer));
    TEST_ASSERT_EQUAL(0, m_memory_leakage_checker);
}
