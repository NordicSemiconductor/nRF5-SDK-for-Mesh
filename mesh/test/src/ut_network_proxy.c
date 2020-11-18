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

#include <stdlib.h>

#include "network.h"

#include "unity.h"
#include "cmock.h"

#include "utils.h"
#include "test_assert.h"

#include "log.h"
#include "packet_mesh.h"

#include "core_tx_mock.h"
#include "core_tx_adv_mock.h"
#include "msg_cache_mock.h"
#include "transport_mock.h"
#include "net_beacon_mock.h"
#include "net_packet_mock.h"
#include "nrf_mesh_externs_mock.h"
#include "manual_mock_queue.h"
#include "proxy_mock.h"

#define IV_INDEX 0x87654321U
#define SEQNUM 0xabcdefU

typedef struct
{
   network_packet_metadata_t * p_net_metadata_expected;
   uint32_t net_packet_len;
   packet_mesh_net_packet_t * p_net_encrypted_packet;
   packet_mesh_net_packet_t * p_net_decrypted_packet;
} net_packet_decrypt_expect_t;

MOCK_QUEUE_DEF(net_packet_decrypt_mock_queue, net_packet_decrypt_expect_t, NULL);
MOCK_QUEUE_DEF(net_metadata_mock_queue, network_packet_metadata_t*, NULL);
MOCK_QUEUE_DEF(bearer_selector_mock_queue, core_tx_bearer_selector_t, NULL);

/*****************************************************************************
* Mocked functions
*****************************************************************************/
static core_tx_bearer_bitmap_t core_tx_packet_alloc_mock(const core_tx_alloc_params_t * p_params,
                                                         uint8_t ** pp_packet, int num_calls)
{
    UNUSED_PARAMETER(num_calls);

    core_tx_bearer_selector_t bearer_selector_expected;
    bearer_selector_mock_queue_Consume(&bearer_selector_expected);
    TEST_ASSERT_EQUAL(bearer_selector_expected, p_params->bearer_selector);

    static uint8_t payload[PACKET_MESH_NET_MAX_SIZE];
    *pp_packet = payload;

    /* For the purpose of this test, we don't care what bearer bitmap was returned, as
     * return value of `core_tx_packet_alloc()` is only compared against 0 and not really
     * used by the network layer */
    return 1;
}

static void proxy_net_packet_processed_mock(const network_packet_metadata_t * p_net_metadata,
                                            const nrf_mesh_rx_metadata_t * p_rx_meta,
                                            int num_calls)
{
    UNUSED_PARAMETER(p_rx_meta);
    UNUSED_PARAMETER(num_calls);

    network_packet_metadata_t *p_net_metadata_expected;
    net_metadata_mock_queue_Consume(&p_net_metadata_expected);
    TEST_ASSERT_EQUAL_PTR(p_net_metadata_expected, p_net_metadata);
}

static uint32_t net_packet_decrypt_mock(network_packet_metadata_t * p_net_metadata,
                                        uint32_t net_packet_len,
                                        const packet_mesh_net_packet_t * p_net_encrypted_packet,
                                        packet_mesh_net_packet_t * p_net_decrypted_packet,
                                        net_packet_kind_t packet_kind,
                                        int num_calls)
{
    UNUSED_PARAMETER(num_calls);

    net_packet_decrypt_expect_t npd_expect;
    net_packet_decrypt_mock_queue_Consume(&npd_expect);

    TEST_ASSERT_NOT_NULL(p_net_metadata);
    TEST_ASSERT_EQUAL(npd_expect.net_packet_len, net_packet_len);
    TEST_ASSERT_EQUAL_PTR(npd_expect.p_net_encrypted_packet, p_net_encrypted_packet);
    TEST_ASSERT_NOT_NULL(p_net_decrypted_packet);
    TEST_ASSERT_EQUAL(NET_PACKET_KIND_TRANSPORT, packet_kind);

    /* Store p_net_metadata and return metadata */
    *p_net_metadata = *npd_expect.p_net_metadata_expected;
    net_metadata_mock_queue_Expect(&p_net_metadata);

    /* Return decrypted packet */
    p_net_decrypted_packet = npd_expect.p_net_decrypted_packet;

    return NRF_SUCCESS;
}

/** Copy of the implementation from nrf_mesh_utils.c. Pulled in to avoid build problems with the other functions in that module */
nrf_mesh_address_type_t nrf_mesh_address_type_get(uint16_t address)
{
    if (address == NRF_MESH_ADDR_UNASSIGNED)
    {
        return NRF_MESH_ADDRESS_TYPE_INVALID;
    }
    else
    {
        static const nrf_mesh_address_type_t types_lookup[] =
        {
            NRF_MESH_ADDRESS_TYPE_UNICAST, /* 0b00 */
            NRF_MESH_ADDRESS_TYPE_UNICAST, /* 0b01 */
            NRF_MESH_ADDRESS_TYPE_VIRTUAL, /* 0b10 */
            NRF_MESH_ADDRESS_TYPE_GROUP,   /* 0b11 */
        };
        return types_lookup[(address & NRF_MESH_ADDR_TYPE_BITS_MASK) >> NRF_MESH_ADDR_TYPE_BITS_OFFSET];
    }
}

/*****************************************************************************
* Setup functions
*****************************************************************************/
void setUp(void)
{
    __LOG_INIT(LOG_SRC_NETWORK, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    core_tx_mock_Init();
    core_tx_adv_mock_Init();
    msg_cache_mock_Init();
    transport_mock_Init();
    net_beacon_mock_Init();
    net_packet_mock_Init();
    nrf_mesh_externs_mock_Init();
    proxy_mock_Init();
    bearer_selector_mock_queue_Init();
    net_metadata_mock_queue_Init();
    net_packet_decrypt_mock_queue_Init();

    core_tx_packet_alloc_StubWithCallback(core_tx_packet_alloc_mock);
}

void tearDown(void)
{
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    core_tx_adv_mock_Verify();
    core_tx_adv_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    net_packet_mock_Verify();
    net_packet_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
    proxy_mock_Verify();
    proxy_mock_Destroy();
    bearer_selector_mock_queue_Verify();
    bearer_selector_mock_queue_Destroy();
    net_metadata_mock_queue_Verify();
    net_metadata_mock_queue_Destroy();
    net_packet_decrypt_mock_queue_Verify();
    net_packet_decrypt_mock_queue_Destroy();
}

/*****************************************************************************
* Helper functions
*****************************************************************************/
static void relay_Expect(core_tx_bearer_selector_t bearer_selector)
{
    bearer_selector_mock_queue_Expect(&bearer_selector);
    static packet_mesh_net_packet_t relay_packet;
    net_packet_from_payload_ExpectAnyArgsAndReturn(&relay_packet);
    net_packet_header_set_ExpectAnyArgs();
    net_packet_encrypt_ExpectAnyArgs();
    core_tx_packet_send_Expect();
}

static void network_packet_in_Trigger(nrf_mesh_rx_source_t source)
{
    proxy_net_packet_processed_StubWithCallback(proxy_net_packet_processed_mock);

    nrf_mesh_network_secmat_t secmat;
    secmat.nid = 0xAF;

    network_packet_metadata_t meta = {
        {NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat
    };

    packet_mesh_net_packet_t net_packet;
    memset(&net_packet, 0xAB, sizeof(net_packet));

    net_packet_decrypt_StubWithCallback(net_packet_decrypt_mock);
    net_packet_decrypt_expect_t npd_expect;
    npd_expect.p_net_metadata_expected = &meta;
    npd_expect.net_packet_len = 18;
    npd_expect.p_net_encrypted_packet = &net_packet;
    npd_expect.p_net_decrypted_packet = &net_packet;

    net_packet_decrypt_mock_queue_Expect(&npd_expect);

    net_packet_obfuscation_start_get_ExpectAndReturn(&net_packet, &net_packet.pdu[1]);

    transport_packet_in_ExpectAnyArgsAndReturn(NRF_SUCCESS);

    nrf_mesh_rx_metadata_t rx_meta;
    rx_meta.source = source;

    net_packet_payload_len_get_ExpectAnyArgsAndReturn(1);
    core_tx_adv_is_enabled_ExpectAndReturn(CORE_TX_ROLE_RELAY, true);
    nrf_mesh_rx_address_get_ExpectAnyArgsAndReturn(false); // for src
    nrf_mesh_rx_address_get_ExpectAnyArgsAndReturn(false); // for dst

    msg_cache_entry_add_ExpectAnyArgs();

    network_packet_in(net_packet.pdu, 18, &rx_meta);

    net_packet_decrypt_mock_queue_Verify();
    net_metadata_mock_queue_Verify();
    bearer_selector_mock_queue_Verify();
}

/*****************************************************************************
* Test functions
*****************************************************************************/
void test_relay_to_proxy(void)
{
    core_tx_bearer_selector_t bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL & ~CORE_TX_BEARER_TYPE_LOCAL;

    /* All bearers should be used if Proxy enabled */
    proxy_is_enabled_IgnoreAndReturn(true);
    relay_Expect(bearer_selector);
    network_packet_in_Trigger(NRF_MESH_RX_SOURCE_SCANNER);

    /* GATT Bearer should be excluded if Proxy disabled */
    proxy_is_enabled_IgnoreAndReturn(false);
    bearer_selector ^= CORE_TX_BEARER_TYPE_GATT_SERVER;
    relay_Expect(bearer_selector);
    network_packet_in_Trigger(NRF_MESH_RX_SOURCE_SCANNER);
}

void test_relay_from_proxy(void)
{
    core_tx_bearer_selector_t bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL & ~CORE_TX_BEARER_TYPE_LOCAL;

    /* Packet from GATT should be relayed if Proxy is enabled */
    proxy_is_enabled_IgnoreAndReturn(true);
    relay_Expect(bearer_selector);
    network_packet_in_Trigger(NRF_MESH_RX_SOURCE_GATT);

    /* Packet should NOT be relayed if Proxy disabled */
    proxy_is_enabled_IgnoreAndReturn(false);
    network_packet_in_Trigger(NRF_MESH_RX_SOURCE_GATT);
}
