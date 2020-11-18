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
#include "unity.h"
#include "cmock.h"

#include "transport.h"
#include "transport_test_common.h"
#include "mesh_opt.h"

#include "friend_internal_mock.h"
#include "mesh_config_entry_mock.h"
#include "mesh_config_mock.h"

/* These are the copy of definitions from replay_cache.c. They shall be synchronized. */
/** Replay cache start ID of the item range */
#define MESH_OPT_REPLY_CACHE_RECORD      0x0001
/** SeqZero cache start ID of the item range */
#define MESH_OPT_SEQZERO_CACHE_RECORD    (MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES)

/*****************************************************************************
* Extern stub
*****************************************************************************/
extern const mesh_config_entry_params_t m_replay_cache_params;
extern const mesh_config_entry_params_t m_seqzero_cache_params;

static uint32_t entry_set_cb(mesh_config_entry_id_t id, const void* p_entry, int num_calls)
{
    (void)num_calls;

    TEST_ASSERT_EQUAL(MESH_OPT_REPLAY_CACHE_FILE_ID, id.file);

    if (IS_IN_RANGE(id.record, MESH_OPT_REPLY_CACHE_RECORD,
                    MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1))
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_replay_cache_params.callbacks.setter(id, p_entry));
        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_SEQZERO_CACHE_RECORD,
                    MESH_OPT_SEQZERO_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1))
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_seqzero_cache_params.callbacks.setter(id, p_entry));
        return NRF_SUCCESS;
    }

    TEST_FAIL();
    return NRF_ERROR_INTERNAL;
}

void setUp(void)
{
    friend_internal_mock_Init();
    mesh_config_entry_mock_Init();
    mesh_config_mock_Init();
    mesh_config_entry_set_StubWithCallback(entry_set_cb);
    mesh_config_entry_delete_IgnoreAndReturn(NRF_SUCCESS);
    transport_test_common_setup();
}

void tearDown(void)
{
    friend_internal_mock_Verify();
    friend_internal_mock_Destroy();
    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
    transport_test_common_teardown();
}

static void expect_friend_needs_packet(bool retval)
{
    friend_needs_packet_ExpectAnyArgsAndReturn(retval);
}

static void expect_unseg_friend_packet_in(uint32_t length, core_tx_role_t role)
{
    friend_packet_in_Expect(NULL, length + PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE, NULL, role);
    friend_packet_in_IgnoreArg_p_packet();
    friend_packet_in_IgnoreArg_p_metadata();
}

static void expect_sar_friend_packet_in(uint32_t length, core_tx_role_t role)
{
    friend_packet_in_Expect(NULL, length + PACKET_MESH_TRS_SEG_PDU_OFFSET, NULL, role);
    friend_packet_in_IgnoreArg_p_packet();
    friend_packet_in_IgnoreArg_p_metadata();
}

void test_friend_packet_rx_unseg(void)
{
    network_packet_metadata_t meta;
    uint16_t src = 1;
    uint32_t packet_len = 10;
    uint32_t seq = 0;

    // receive a unicast packet destined for ourself, but not the friend. Should go right up to the app, and be added to RPL:
    expect_friend_needs_packet(false);
    expect_access_rx(packet_len, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, packet_len);
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq-1, m_iv_index));

    // receive a group packet destined for ourself, but not the friend. Should go right up to the app, and be added to RPL:
    expect_friend_needs_packet(false);
    expect_access_rx(packet_len, src, NRF_MESH_ADDRESS_TYPE_GROUP);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_unseg_rx(&meta, packet_len);
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq-1, m_iv_index));

    // receive a packet destined for ourself and the friend. Should go right up to the app, and be added to RPL:
    expect_friend_needs_packet(true);
    expect_unseg_friend_packet_in(packet_len, CORE_TX_ROLE_RELAY);
    expect_access_rx(packet_len, src, NRF_MESH_ADDRESS_TYPE_GROUP);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_unseg_rx(&meta, packet_len);
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq-1, m_iv_index));

    // receive a group packet destined for the friend but not ourself. Should not go up to the app, and not be added to RPL:
    m_rx_addr_ok = false;
    expect_friend_needs_packet(true);
    expect_unseg_friend_packet_in(packet_len, CORE_TX_ROLE_RELAY);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_unseg_rx(&meta, packet_len);
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq-1, m_iv_index));

    // receive a unicast packet destined for the friend but not ourself. Should not go up to the app and not be added to RPL:
    m_rx_addr_ok = false;
    expect_friend_needs_packet(true);
    expect_unseg_friend_packet_in(packet_len, CORE_TX_ROLE_RELAY);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, packet_len);
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq-1, m_iv_index));

    // receive a unicast packet destined for the friend with SEQ in RPL. Should avoid RPL and go to the friend.
    expect_friend_needs_packet(true);
    expect_unseg_friend_packet_in(packet_len, CORE_TX_ROLE_RELAY);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(src, seq, m_iv_index));
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, packet_len);
}

void test_friend_packet_rx_sar_single_segment(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;
    uint32_t seq = 9001; // Start high to have all values represented in seqauth

    sar_session_t sar_session = {
        .seqzero = 0,
        .segment_count = 1,
        .total_len = PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    uint64_t seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);

    // receive a unicast packet for ourself, but not the friend. Should go to app, but friend_sar_complete shouldn't be called
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(false);
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_friend_needs_packet(false);
    expect_sar_ack(&sar_session, 1);
    friend_sar_exists_ExpectAndReturn(src, seqauth, false);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq-1, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq-1, m_iv_index));
    event_mock_Verify();

    // receive a group packet for ourself, but not the friend. Should go to app, but friend_sar_complete shouldn't be called
    sar_session.seqzero = seq & TRANSPORT_SAR_SEQZERO_MASK;
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(false);
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);
    friend_sar_exists_ExpectAndReturn(src, seqauth, false);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq-1, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq-1, m_iv_index));
    event_mock_Verify();

    // receive a packet for ourself and the friend. Should go to app, and friend_sar_complete should be called with a success-value
    sar_session.seqzero = seq & TRANSPORT_SAR_SEQZERO_MASK;
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);
    friend_sar_exists_ExpectAndReturn(src, seqauth, false);
    friend_sar_complete_Expect(src, sar_session.seqzero, true);
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq-1, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq-1, m_iv_index));
    event_mock_Verify();

    // receive a unicast packet for the friend. Should not go to app, and friend_sar_complete should be called with a success-value. Ack should have OBO set
    m_rx_addr_ok = false;
    m_ack_on_behalf_of_friend = true;
    sar_session.seqzero = seq & TRANSPORT_SAR_SEQZERO_MASK;
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);
    friend_sar_exists_ExpectAndReturn(src, seqauth, false);
    friend_sar_complete_Expect(src, sar_session.seqzero, true);
    expect_friend_needs_packet(true);
    expect_sar_ack(&sar_session, 1);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq-1, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq-1, m_iv_index)); // friend packets aren't added to the replay list

    // The sender didn't get our ack, and sends the segment again. This should be acked with OBO set, and the packet should NOT be added to replay (just like the rest of the session)
    m_ack_on_behalf_of_friend = true;
    expect_friend_needs_packet(true);
    seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);
    friend_sar_exists_ExpectAndReturn(src, seqauth, true);
    expect_friend_needs_packet(true);
    expect_sar_ack(&sar_session, 1);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq-2, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq-2, m_iv_index));

    // receive a group packet for the friend. Should not go to app, and friend_sar_complete should be called with a success-value.
    m_rx_addr_ok = false;
    m_ack_on_behalf_of_friend = true;
    sar_session.seqzero = seq & TRANSPORT_SAR_SEQZERO_MASK;
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);
    friend_sar_exists_ExpectAndReturn(src, seqauth, false);
    friend_sar_complete_Expect(src, sar_session.seqzero, true);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq-1, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq-1, m_iv_index));

    // The sender sends the segment again, but we've already added the packet to our own RX list.
    // This still shouldn't make it go to the app, as it has been processed already, and the packet
    // should NOT be added to replay (just like the rest of the session)
    m_rx_addr_ok = true;
    expect_friend_needs_packet(true);
    seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);
    friend_sar_exists_ExpectAndReturn(src, seqauth, true);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    // we don't add messages for the friend to replay
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq-2, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq-2, m_iv_index));
}

void test_sar_multisegment(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;
    uint32_t seq = 9001; // Start high to have all values represented in seqauth

    sar_session_t sar_session = {
        .seqzero = 0,
        .segment_count = 4,
        .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    uint64_t seqauth = transport_sar_seqauth_get(m_iv_index, seq, sar_session.seqzero);

    // receive a packet for ourself and the friend. Should go to app, and friend_sar_complete should be called with a success-value
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(true);
    friend_sar_exists_ExpectAndReturn(src, seqauth, false);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    // segment 2
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_session);
    // segment 3
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_session);

    // final segment
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    friend_sar_complete_Expect(src, sar_session.seqzero, true);
    expect_sar_ctx_free();
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_session);

}

/**
 * Receive sar packets from two different sources at the same time. Shouldn't affect the interaction
 * with friend_internal.
 */
void test_sar_concurrent_multisegment(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src[2] = {1, 2};
    uint32_t seq[2] = {9001, 9001}; // Start high to have all values represented in seqauth

    sar_session_t sar_session[2] = {
        {
            .seqzero = 0,
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        },
        {
            .seqzero = 0,
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        },
    };

    uint64_t seqauth[2] = {
        transport_sar_seqauth_get(m_iv_index, seq[0], sar_session[0].seqzero),
        transport_sar_seqauth_get(m_iv_index, seq[1], sar_session[1].seqzero)
    };

    // device A, segment 1
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(true);
    friend_sar_exists_ExpectAndReturn(src[0], seqauth[0], false);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src[0], seq[0]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session[0]);

    // device B, segment 1
    expect_sar_ctx_alloc();
    expect_friend_needs_packet(true);
    friend_sar_exists_ExpectAndReturn(src[1], seqauth[1], false);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src[1], seq[1]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session[1]);

    // device A, segment 2
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src[0], seq[0]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_session[0]);
    // device A, segment 3
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src[0], seq[0]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_session[0]);

    // device B, segment 2
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src[1], seq[1]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_session[1]);
    // device B, segment 3
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    net_meta_build(src[1], seq[1]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_session[1]);

    // device A, final segment
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    friend_sar_complete_Expect(src[0], sar_session[0].seqzero, true);
    expect_access_rx(sar_session[0].total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src[0],
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    expect_sar_ctx_free();
    net_meta_build(src[0], seq[0]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_session[0]);

    // device B, final segment
    expect_friend_needs_packet(true);
    expect_sar_friend_packet_in(PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, CORE_TX_ROLE_RELAY);
    friend_sar_complete_Expect(src[1], sar_session[1].seqzero, true);
    expect_access_rx(sar_session[1].total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src[1],
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    expect_sar_ctx_free();
    net_meta_build(src[1], seq[1]++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_session[1]);
}

