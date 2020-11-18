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
#include "heartbeat_mock.h"
#include "msg_cache_mock.h"
#include "transport_mock.h"
#include "net_beacon_mock.h"
#include "net_state_mock.h"
#include "net_packet_mock.h"
#include "nrf_mesh_externs_mock.h"
#include "friend_internal_mock.h"
#include "mesh_lpn_mock.h"
#include "manual_mock_queue.h"

#define TOKEN 0x12345678U
#define IV_INDEX 0x87654321U
#define SEQNUM 0xabcdefU

typedef struct
{
    core_tx_alloc_params_t params;
    network_packet_metadata_t metadata;
    uint8_t *p_packet;
    bool success;
} alloc_packet_mock_t;
MOCK_QUEUE_DEF(alloc_packet_mock, alloc_packet_mock_t, NULL);

static core_tx_bearer_bitmap_t core_tx_packet_alloc_mock(const core_tx_alloc_params_t * p_params,
                                                         uint8_t ** pp_packet, int num_calls)
{
    alloc_packet_mock_t expected_alloc;
    alloc_packet_mock_Consume(&expected_alloc);

    UNUSED_PARAMETER(pp_packet);
    UNUSED_PARAMETER(num_calls);

    TEST_ASSERT_EQUAL(expected_alloc.params.role, p_params->role);
    TEST_ASSERT_EQUAL(expected_alloc.params.net_packet_len, p_params->net_packet_len);
    TEST_ASSERT_EQUAL_MEMORY(&expected_alloc.metadata, p_params->p_metadata, sizeof(network_packet_metadata_t));
    TEST_ASSERT_EQUAL(expected_alloc.params.token, p_params->token);
    TEST_ASSERT_EQUAL(expected_alloc.params.bearer_selector, p_params->bearer_selector);

    *pp_packet = expected_alloc.p_packet;

    return expected_alloc.success ? expected_alloc.params.bearer_selector : 0;
}

void setUp(void)
{
    __LOG_INIT(LOG_SRC_NETWORK, LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    core_tx_mock_Init();
    core_tx_adv_mock_Init();
    heartbeat_mock_Init();
    msg_cache_mock_Init();
    transport_mock_Init();
    net_beacon_mock_Init();
    net_state_mock_Init();
    net_packet_mock_Init();
    nrf_mesh_externs_mock_Init();
    friend_internal_mock_Init();
    alloc_packet_mock_Init();
    mesh_lpn_mock_Init();

    core_tx_packet_alloc_StubWithCallback(core_tx_packet_alloc_mock);
}

void tearDown(void)
{
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    core_tx_adv_mock_Verify();
    core_tx_adv_mock_Destroy();
    heartbeat_mock_Verify();
    heartbeat_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    net_packet_mock_Verify();
    net_packet_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
    friend_internal_mock_Verify();
    friend_internal_mock_Destroy();
    alloc_packet_mock_Verify();
    alloc_packet_mock_Destroy();
    mesh_lpn_mock_Verify();
    mesh_lpn_mock_Destroy();
}
/*****************************************************************************
* Helper functions
*****************************************************************************/
struct
{
    uint16_t src;
    uint16_t dst;
    uint8_t ttl;
    uint32_t calls;
    bool retval;
} m_relay_callback_expect;

static bool relay_callback(uint16_t src, uint16_t dst, uint8_t ttl)
{
    TEST_ASSERT_TRUE(m_relay_callback_expect.calls > 0);
    TEST_ASSERT_EQUAL(m_relay_callback_expect.src, src);
    TEST_ASSERT_EQUAL(m_relay_callback_expect.dst, dst);
    TEST_ASSERT_EQUAL(m_relay_callback_expect.ttl, ttl);
    m_relay_callback_expect.calls--;

    return m_relay_callback_expect.retval;
}

static void packet_alloc_Expect(network_packet_metadata_t * p_metadata,
                                uint32_t packet_len, uint8_t * p_packet,
                                core_tx_role_t role, bool success,
                                core_tx_bearer_selector_t bearer_selector)
{
    alloc_packet_mock_t expect_alloc;
    expect_alloc.params.role               = role;
    expect_alloc.params.net_packet_len     = packet_len;
    expect_alloc.params.token              = (role == CORE_TX_ROLE_ORIGINATOR ? TOKEN : NRF_MESH_RELAY_TOKEN);
    expect_alloc.params.bearer_selector    = bearer_selector;
    expect_alloc.p_packet                  = p_packet;
    expect_alloc.success                   = success;
    expect_alloc.metadata = *p_metadata;

    alloc_packet_mock_Expect(&expect_alloc);
}

static void relay_Expect(network_packet_metadata_t * p_metadata, uint32_t packet_len,
                         packet_mesh_net_packet_t ** pp_relay_packet,
                         core_tx_bearer_selector_t bearer_bitmap, bool alloc_fail)
{
    m_relay_callback_expect.calls  = 1;
    m_relay_callback_expect.dst    = p_metadata->dst.value;
    m_relay_callback_expect.retval = true;
    m_relay_callback_expect.src    = p_metadata->src;
    m_relay_callback_expect.ttl    = p_metadata->ttl;
    uint8_t mic_size               = (p_metadata->control_packet ? 8 : 4);

    static network_packet_metadata_t relay_meta;
    memcpy(&relay_meta, p_metadata, sizeof(relay_meta));
    relay_meta.ttl--;

    packet_alloc_Expect(&relay_meta, packet_len, (uint8_t *) *pp_relay_packet,
                        CORE_TX_ROLE_RELAY, !alloc_fail, bearer_bitmap);
    if (!alloc_fail)
    {
        net_packet_from_payload_ExpectAndReturn(&(*pp_relay_packet)->pdu[9], *pp_relay_packet);
        net_packet_header_set_Expect(*pp_relay_packet, &relay_meta);
        net_packet_encrypt_Expect(&relay_meta, packet_len - 9 - mic_size, *pp_relay_packet, NET_PACKET_KIND_TRANSPORT);
        core_tx_packet_send_Expect();
    }
}

struct
{
    const packet_mesh_trs_packet_t * p_packet;
    uint32_t trs_packet_len;
    network_packet_metadata_t * p_net_metadata;
    const nrf_mesh_rx_metadata_t * p_rx_metadata;
    uint32_t calls;
} m_transport_packet_in_expect;

static uint32_t transport_packet_in_callback(const packet_mesh_trs_packet_t * p_packet,
                                             uint32_t trs_packet_len,
                                             const network_packet_metadata_t * p_net_metadata,
                                             const nrf_mesh_rx_metadata_t * p_rx_metadata,
                                             int calls)
{
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.trs_packet_len, trs_packet_len);
    if (trs_packet_len > 0)
    {
        TEST_ASSERT_EQUAL_HEX8_ARRAY(m_transport_packet_in_expect.p_packet, p_packet, trs_packet_len);
    }
    /* Metadata has padding, so we have to compare the fields individually */
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->control_packet, p_net_metadata->control_packet);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->dst.type, p_net_metadata->dst.type);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->dst.value, p_net_metadata->dst.value);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->internal.iv_index, p_net_metadata->internal.iv_index);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->internal.sequence_number, p_net_metadata->internal.sequence_number);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->p_security_material, p_net_metadata->p_security_material);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->src, p_net_metadata->src);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_net_metadata->ttl, p_net_metadata->ttl);
    TEST_ASSERT_EQUAL(m_transport_packet_in_expect.p_rx_metadata, p_rx_metadata);
    TEST_ASSERT_TRUE(m_transport_packet_in_expect.calls > 0);
    m_transport_packet_in_expect.calls--;
    return 0x12345678;
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
* Test functions
*****************************************************************************/
void test_init(void)
{
    net_beacon_init_Expect();
    net_state_init_Expect();
    nrf_mesh_init_params_t init_params = {{0}};
    network_init(&init_params);

    net_beacon_init_Expect();
    net_state_init_Expect();
    init_params.relay_cb = relay_callback;
    network_init(&init_params);
}

void test_enable(void)
{
    net_state_enable_Expect();
#if !MESH_FEATURE_LPN_ENABLED
    net_beacon_enable_Expect();
#endif
    network_enable();
}

void test_alloc(void)
{
    struct
    {
        uint16_t src;
        uint16_t dst;
        uint8_t ttl;
        bool control;
        uint8_t payload_len;
        bool core_tx_alloc_ok;
        bool seqnum_alloc_ok;
    } vector[] = {
        {0x0001, 0x0002, 8, false, 10, true, true},
        {0x0001, 0x0002, 8, false, 10, false, true}, /* No packets available */
        {0x0001, 0x0002, 8, false, 10, true, false}, /* No seqnum available */
    };
    nrf_mesh_network_secmat_t secmat;
    network_packet_metadata_t metadata;
    network_tx_packet_buffer_t buffer;
    uint8_t payload[PACKET_MESH_NET_MAX_SIZE];
    uint8_t * p_packet = payload;
    memset(secmat.encryption_key, 0xEC, NRF_MESH_KEY_SIZE);
    memset(secmat.privacy_key, 0x93, NRF_MESH_KEY_SIZE);
    secmat.nid = 0xAA;
    buffer.user_data.p_metadata         = &metadata;
    buffer.user_data.token              = TOKEN;
    buffer.user_data.role               = CORE_TX_ROLE_ORIGINATOR;
    buffer.user_data.bearer_selector    = CORE_TX_BEARER_TYPE_ALLOW_ALL;
    metadata.p_security_material = &secmat;

    for (volatile uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        metadata.control_packet      = vector[i].control;
        metadata.dst.value           = vector[i].dst;
        metadata.dst.type =
            (vector[i].dst < 0x8000) ? NRF_MESH_ADDRESS_TYPE_UNICAST : NRF_MESH_ADDRESS_TYPE_GROUP;
        metadata.src                 = vector[i].src;
        metadata.ttl                 = vector[i].ttl;
        buffer.user_data.payload_len = vector[i].payload_len;

        packet_alloc_Expect(&metadata,
                            vector[i].payload_len + 9 + (vector[i].control ? 8 : 4),
                            p_packet,
                            CORE_TX_ROLE_ORIGINATOR,
                            vector[i].core_tx_alloc_ok,
                            CORE_TX_BEARER_TYPE_ALLOW_ALL);

        if (vector[i].core_tx_alloc_ok)
        {
            uint32_t iv_index = IV_INDEX;
            uint32_t seqnum = SEQNUM;
            net_state_iv_index_and_seqnum_alloc_ExpectAndReturn(&metadata.internal.iv_index,
                                                            &metadata.internal.sequence_number,
                                                            vector[i].seqnum_alloc_ok ? NRF_SUCCESS
                                                                                      : NRF_ERROR_FORBIDDEN);
            net_state_iv_index_and_seqnum_alloc_ReturnMemThruPtr_p_iv_index(&iv_index, sizeof(iv_index));
            net_state_iv_index_and_seqnum_alloc_ReturnMemThruPtr_p_seqnum(&seqnum, sizeof(seqnum));


            if (vector[i].seqnum_alloc_ok)
            {
                net_packet_header_set_Expect((packet_mesh_net_packet_t *) p_packet, &metadata);
                TEST_ASSERT_EQUAL(NRF_SUCCESS, network_packet_alloc(&buffer));
                TEST_ASSERT_EQUAL_PTR(&payload[9], buffer.p_payload);
            }
            else
            {
                core_tx_packet_discard_Expect();
                TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, network_packet_alloc(&buffer));
            }
        }
        else
        {
            TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, network_packet_alloc(&buffer));
        }
        net_state_mock_Verify();
        core_tx_mock_Verify();
    }
    /* Invalid params not covered by vectors */
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(NULL));
    metadata.p_security_material = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(&buffer));
    buffer.user_data.p_metadata = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(&buffer));
    buffer.user_data.p_metadata = &metadata;
    metadata.p_security_material = &secmat;
    buffer.user_data.role = CORE_TX_ROLE_COUNT; // too high
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_alloc(&buffer));

}

void test_send(void)
{
    nrf_mesh_network_secmat_t secmat;
    network_packet_metadata_t metadata;
    network_tx_packet_buffer_t buffer;
    packet_mesh_net_packet_t packet;
    uint16_t dst = 0x1234;
    uint8_t len  = 10;

    for (uint32_t control = 0; control < 2U; ++control)
    {
        metadata.dst.value                = dst;
        metadata.p_security_material      = &secmat;
        metadata.internal.iv_index        = IV_INDEX;
        metadata.internal.sequence_number = SEQNUM;
        metadata.control_packet           = control;

        net_packet_from_payload_ExpectAndReturn(&packet.pdu[9], &packet);
        net_packet_encrypt_Expect(&metadata, len, &packet, NET_PACKET_KIND_TRANSPORT);

        buffer.user_data.p_metadata  = &metadata;
        buffer.user_data.token       = TOKEN;
        buffer.user_data.payload_len = len;
        buffer.user_data.role        = CORE_TX_ROLE_ORIGINATOR;
        buffer.p_payload             = &packet.pdu[9];
        core_tx_packet_send_Expect();

        network_packet_send(&buffer);
    }
    /* Invalid params */
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(NULL));
    buffer.p_payload = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(&buffer));
    buffer.p_payload = &packet.pdu[9];
    buffer.user_data.p_metadata->p_security_material = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(&buffer));
    buffer.user_data.p_metadata = NULL;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(&buffer));
    buffer.user_data.p_metadata = &metadata;
    buffer.user_data.bearer_selector = CORE_TX_BEARER_TYPE_INVALID;
    TEST_NRF_MESH_ASSERT_EXPECT(network_packet_send(&buffer));
}

void test_discard(void)
{
    uint8_t payload[PACKET_MESH_NET_MAX_SIZE];
    network_packet_metadata_t metadata;
    network_tx_packet_buffer_t buffer;
    metadata.control_packet      = false;
    buffer.user_data.p_metadata  = &metadata;
    buffer.user_data.token       = TOKEN;
    buffer.user_data.payload_len = 10;
    buffer.user_data.role        = CORE_TX_ROLE_ORIGINATOR;
    buffer.p_payload             = &payload[9];

    for (uint32_t control = 0; control < 2U; ++control)
    {
        metadata.control_packet    = control;
        core_tx_packet_discard_Expect();
        network_packet_discard(&buffer);
    }
}

/**
 * Test packet in function, to make sure it parses the header and calls the encryption module
 * correctly, as well as discarding packets correctly.
 *
 * The packet in procedure works like this:
 * 1: Decrypt the packet
 * 2: Send to transport
 * 3: add to message cache
 * 4: translate friendship secmat
 * 5: Relay if possible
 *
 * Note: some test cases may emulate failure scenarios that can cause early return.
 */
void test_packet_in(void)
{
    nrf_mesh_network_secmat_t secmat;
    secmat.nid = 0xAF;
    typedef enum {
        STEP_DECRYPTION,
        STEP_SELF_SENDING,
        STEP_DO_RELAY,
        STEP_PACKET_ALLOC,
        STEP_SUCCESS,
    } step_t;
    struct
    {
        network_packet_metadata_t meta;
        uint32_t length;
        step_t fail_step; /**< The step where the packet processing stops, or STEP_SUCCESS if it goes through all the steps */
    } vector[] = {
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS}, /* access packet */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_DECRYPTION}, /* access packet */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS}, /* Control packet */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS}, /* Unicast DST */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+4, STEP_SUCCESS}, /* just long enough */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 9+7, STEP_SUCCESS}, /* long enough for a data packet */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 9+8, STEP_SUCCESS}, /* just long enough */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_DO_RELAY}, /* Packet is for this device, shouldn't relay */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 1, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_DO_RELAY}, /* TTL too low to relay */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS},
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SELF_SENDING}, /* It is own src address. Stop handling to prevent relay loops */
        {{{NRF_MESH_ADDRESS_TYPE_VIRTUAL, 0x8000}, 0x0001, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_DECRYPTION}, /* Virual address used in control packet */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_PACKET_ALLOC}, /* Allocating packet fails, should not add to cache */
    };
    nrf_mesh_rx_metadata_t rx_meta;

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        packet_mesh_net_packet_t relay_packet;
        packet_mesh_net_packet_t * p_relay_packet = &relay_packet;
        packet_mesh_net_packet_t net_packet;
        uint8_t mic_len = vector[i].meta.control_packet ? 8 : 4;
        memset(&net_packet, 0xAB, sizeof(net_packet));
        memset(&m_transport_packet_in_expect, 0, sizeof(m_transport_packet_in_expect));

        net_packet_obfuscation_start_get_ExpectAndReturn(&net_packet, &net_packet.pdu[1]);

        /* 1: Decrypt */
        net_packet_decrypt_ExpectAndReturn(NULL,
                                           vector[i].length,
                                           &net_packet,
                                           NULL,
                                           NET_PACKET_KIND_TRANSPORT,
                                           ((vector[i].fail_step > STEP_DECRYPTION)
                                                ? NRF_SUCCESS
                                                : NRF_ERROR_NOT_FOUND));

        net_packet_decrypt_IgnoreArg_p_net_decrypted_packet();
        net_packet_decrypt_ReturnMemThruPtr_p_net_decrypted_packet(&net_packet, vector[i].length);

        net_packet_decrypt_IgnoreArg_p_net_metadata();
        net_packet_decrypt_ReturnThruPtr_p_net_metadata(&vector[i].meta);

        if (vector[i].fail_step > STEP_DECRYPTION)
        {
            /* 2: Send to transport */
            net_packet_payload_len_get_ExpectAndReturn(&vector[i].meta,
                                                       vector[i].length,
                                                       vector[i].length - 9 - mic_len);
            transport_packet_in_StubWithCallback(transport_packet_in_callback);

            m_transport_packet_in_expect.p_packet = (const packet_mesh_trs_packet_t *) &net_packet.pdu[9];
            m_transport_packet_in_expect.trs_packet_len =
                vector[i].length - 9 - mic_len;
            m_transport_packet_in_expect.p_net_metadata = &vector[i].meta;
            m_transport_packet_in_expect.p_rx_metadata  = &rx_meta;
            m_transport_packet_in_expect.calls          = 1;

            /* 3: Check friendship secmat translation. */
            friend_friendship_established_ExpectAnyArgsAndReturn(false);

            /* 4: Relay if needed: */
            nrf_mesh_rx_address_get_ExpectAndReturn(vector[i].meta.src, NULL, (vector[i].fail_step == STEP_SELF_SENDING));
            nrf_mesh_rx_address_get_IgnoreArg_p_address();

            if (vector[i].fail_step > STEP_SELF_SENDING)
            {
                core_tx_adv_is_enabled_ExpectAndReturn(CORE_TX_ROLE_RELAY, true);
            }

            if (vector[i].meta.ttl >= 2 && vector[i].fail_step > STEP_SELF_SENDING)
            {
                core_tx_bearer_type_t bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL & ~CORE_TX_BEARER_TYPE_LOCAL;

                if (vector[i].meta.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
                {
                    nrf_mesh_rx_address_get_ExpectAndReturn(vector[i].meta.dst.value,
                                                            NULL,
                                                            (vector[i].fail_step == STEP_DO_RELAY));
                    nrf_mesh_rx_address_get_IgnoreArg_p_address();
                }
                if (vector[i].fail_step > STEP_DO_RELAY)
                {
#if MESH_FEATURE_LPN_ENABLED
                    mesh_lpn_is_in_friendship_ExpectAndReturn(false);
#endif
                    relay_Expect(&vector[i].meta, vector[i].length, &p_relay_packet,
                                 bearer_selector, vector[i].fail_step == STEP_PACKET_ALLOC);
                }
            }

            /* 5: Add to message cache */
            if (vector[i].fail_step != STEP_PACKET_ALLOC)
            {
                msg_cache_entry_add_Expect(vector[i].meta.src, vector[i].meta.internal.sequence_number);
            }
        }

        network_packet_in(net_packet.pdu, vector[i].length, &rx_meta);

        TEST_ASSERT_EQUAL(0, m_transport_packet_in_expect.calls);
        TEST_ASSERT_EQUAL(0, m_relay_callback_expect.calls);
        core_tx_mock_Verify();
        transport_mock_Verify();
        nrf_mesh_externs_mock_Verify();
        net_packet_mock_Verify();
        friend_internal_mock_Verify();
    }
}

/* This is slightly modified version of test_packet_in, to explicitly test secmat translation */
void test_packet_in_friendship_secmat_translation(void)
{
    TEST_ASSERT_EQUAL(1, MESH_FEATURE_FRIEND_ENABLED);
    TEST_ASSERT_EQUAL(1, MESH_FEATURE_RELAY_ENABLED);

    nrf_mesh_network_secmat_t secmat;
    nrf_mesh_network_secmat_t secmat_new;
    network_packet_metadata_t relay_expect_meta;

    secmat.nid = 0x70;
    secmat_new.nid = 0x07;
    typedef enum {
        STEP_DECRYPTION,
        STEP_DO_RELAY,
        STEP_SUCCESS,
    } step_t;
    struct
    {
        network_packet_metadata_t meta;
        uint32_t length;
        step_t fail_step; /**< The step where the packet processing stops, or STEP_SUCCESS if it goes through all the steps */
    } vector[] = {
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS}, /* access packet */
        {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xC001}, 0x0003, 5, true, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS},  /* Control packet */
        {{{NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0002}, 0x0004, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18, STEP_SUCCESS} /* Unicast DST */
    };
    nrf_mesh_rx_metadata_t rx_meta;

    for (uint32_t i = 0; i < ARRAY_SIZE(vector); ++i)
    {
        packet_mesh_net_packet_t relay_packet;
        packet_mesh_net_packet_t * p_relay_packet = &relay_packet;
        packet_mesh_net_packet_t net_packet;
        uint8_t mic_len = vector[i].meta.control_packet ? 8 : 4;
        memset(&net_packet, 0xAB, sizeof(net_packet));

        net_packet_obfuscation_start_get_ExpectAndReturn(&net_packet, &net_packet.pdu[1]);

        /* 1: Decrypt */
        net_packet_decrypt_ExpectAndReturn(NULL,
                                           vector[i].length,
                                           &net_packet,
                                           NULL,
                                           NET_PACKET_KIND_TRANSPORT,
                                           ((vector[i].fail_step > STEP_DECRYPTION)
                                                ? NRF_SUCCESS
                                                : NRF_ERROR_NOT_FOUND));

        net_packet_decrypt_IgnoreArg_p_net_decrypted_packet();
        net_packet_decrypt_ReturnMemThruPtr_p_net_decrypted_packet(&net_packet, vector[i].length);

        net_packet_decrypt_IgnoreArg_p_net_metadata();
        net_packet_decrypt_ReturnThruPtr_p_net_metadata(&vector[i].meta);

        TEST_ASSERT_TRUE(vector[i].fail_step > STEP_DECRYPTION);

        /* 2: Send to transport */
        net_packet_payload_len_get_ExpectAndReturn(&vector[i].meta,
                                                    vector[i].length,
                                                    vector[i].length - 9 - mic_len);
        transport_packet_in_StubWithCallback(transport_packet_in_callback);

        m_transport_packet_in_expect.p_packet = (const packet_mesh_trs_packet_t *) &net_packet.pdu[9];
        m_transport_packet_in_expect.trs_packet_len =
            vector[i].length - 9 - mic_len;
        m_transport_packet_in_expect.p_net_metadata = &vector[i].meta;
        m_transport_packet_in_expect.p_rx_metadata  = &rx_meta;
        m_transport_packet_in_expect.calls          = 1;
        core_tx_adv_is_enabled_ExpectAndReturn(CORE_TX_ROLE_RELAY, true);

        /* 3: Add to message cache */
        msg_cache_entry_add_Expect(vector[i].meta.src, vector[i].meta.internal.sequence_number);

        /* 4: Check friendship secmat translation. Each switch case checks different test case */
        TEST_ASSERT_EQUAL(3, ARRAY_SIZE(vector));
        relay_expect_meta = vector[i].meta;
        switch (i)
        {
            /* Test: Friendship established returns false */
            case 0:
                friend_friendship_established_ExpectAndReturn(vector[i].meta.src, false);
                break;

            /* Test: Friendship established returns true, secmat get returns NULL. */
            case 1:
                friend_friendship_established_ExpectAndReturn(vector[i].meta.src, true);
                nrf_mesh_net_master_secmat_get_ExpectAndReturn(vector[i].meta.p_security_material, NULL);
                break;

            /* Test: Friendship established returns true, secmat get returns some other secmat */
            case 2:
                friend_friendship_established_ExpectAndReturn(vector[i].meta.src, true);
                nrf_mesh_net_master_secmat_get_ExpectAndReturn(vector[i].meta.p_security_material, &secmat_new);
                relay_expect_meta.p_security_material = &secmat_new;
                break;

            /* There are no more than 3 test cases. Vector size is larger than the test scenarios. */
            default:
                TEST_ASSERT_TRUE(false);
        }

        /* 5: Always expect relaying, so that we can check the secmat. */
        nrf_mesh_rx_address_get_ExpectAndReturn(vector[i].meta.src, NULL, false);
        nrf_mesh_rx_address_get_IgnoreArg_p_address();
        TEST_ASSERT_TRUE(vector[i].meta.ttl >= 2);
        if (vector[i].meta.dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST)
        {
            nrf_mesh_rx_address_get_ExpectAndReturn(vector[i].meta.dst.value,
                                                    NULL,
                                                    (vector[i].fail_step == STEP_DO_RELAY));
            nrf_mesh_rx_address_get_IgnoreArg_p_address();
        }

        TEST_ASSERT_TRUE(vector[i].fail_step > STEP_DO_RELAY);
#if MESH_FEATURE_LPN_ENABLED
        mesh_lpn_is_in_friendship_ExpectAndReturn(false);
#endif
        relay_Expect(&relay_expect_meta, vector[i].length, &p_relay_packet, CORE_TX_BEARER_TYPE_ALLOW_ALL & ~CORE_TX_BEARER_TYPE_LOCAL, false);

        network_packet_in(net_packet.pdu, vector[i].length, &rx_meta);

        TEST_ASSERT_EQUAL(0, m_transport_packet_in_expect.calls);
        TEST_ASSERT_EQUAL(0, m_relay_callback_expect.calls);
        core_tx_mock_Verify();
        transport_mock_Verify();
        nrf_mesh_externs_mock_Verify();
        net_packet_mock_Verify();
        friend_internal_mock_Verify();
    }
}

/* This is slightly modified version of test_packet_in, to explicitly test bypassing relay in lpn mode */
void test_relay_in_lpn(void)
{
#if MESH_FEATURE_LPN_ENABLED
    net_beacon_init_Expect();
    net_state_init_Expect();
    nrf_mesh_init_params_t init_params = {.relay_cb = NULL};
    network_init(&init_params);

    nrf_mesh_network_secmat_t secmat;
    secmat.nid = 0xAF;
    struct
    {
        network_packet_metadata_t meta;
        uint32_t length;
    } vector = {{{NRF_MESH_ADDRESS_TYPE_GROUP, 0xFFFF}, 0x0001, 5, false, {SEQNUM, IV_INDEX}, &secmat}, 18};

    nrf_mesh_rx_metadata_t rx_meta;

    packet_mesh_net_packet_t net_packet;
    uint8_t mic_len = 4;
    memset(&net_packet, 0xAB, sizeof(net_packet));
    memset(&m_transport_packet_in_expect, 0, sizeof(m_transport_packet_in_expect));

    net_packet_obfuscation_start_get_ExpectAndReturn(&net_packet, &net_packet.pdu[1]);

    /* 1: Decrypt */
    net_packet_decrypt_ExpectAndReturn(NULL,
                                       vector.length,
                                       &net_packet,
                                       NULL,
                                       NET_PACKET_KIND_TRANSPORT,
                                       NRF_SUCCESS);

    net_packet_decrypt_IgnoreArg_p_net_decrypted_packet();
    net_packet_decrypt_ReturnMemThruPtr_p_net_decrypted_packet(&net_packet, vector.length);

    net_packet_decrypt_IgnoreArg_p_net_metadata();
    net_packet_decrypt_ReturnThruPtr_p_net_metadata(&vector.meta);

    /* 2: Send to transport */
    net_packet_payload_len_get_ExpectAndReturn(&vector.meta,
                                               vector.length,
                                               vector.length - 9 - mic_len);
    transport_packet_in_StubWithCallback(transport_packet_in_callback);

    m_transport_packet_in_expect.p_packet = (const packet_mesh_trs_packet_t *) &net_packet.pdu[9];
    m_transport_packet_in_expect.trs_packet_len = vector.length - 9 - mic_len;
    m_transport_packet_in_expect.p_net_metadata = &vector.meta;
    m_transport_packet_in_expect.p_rx_metadata  = &rx_meta;
    m_transport_packet_in_expect.calls          = 1;

    /* 3: Check friendship secmat translation. */
    friend_friendship_established_ExpectAnyArgsAndReturn(false);

    /* 4: Relay if needed: */
    nrf_mesh_rx_address_get_ExpectAndReturn(vector.meta.src, NULL, false);
    nrf_mesh_rx_address_get_IgnoreArg_p_address();

    core_tx_adv_is_enabled_ExpectAndReturn(CORE_TX_ROLE_RELAY, true);

    if (vector.meta.ttl >= 2)
    {
        mesh_lpn_is_in_friendship_ExpectAndReturn(true);
    }

    msg_cache_entry_add_Expect(vector.meta.src, vector.meta.internal.sequence_number);

    network_packet_in(net_packet.pdu, vector.length, &rx_meta);

    TEST_ASSERT_EQUAL(0, m_transport_packet_in_expect.calls);
    TEST_ASSERT_EQUAL(0, m_relay_callback_expect.calls);
    core_tx_mock_Verify();
    transport_mock_Verify();
    nrf_mesh_externs_mock_Verify();
    net_packet_mock_Verify();
    friend_internal_mock_Verify();
#endif
}
