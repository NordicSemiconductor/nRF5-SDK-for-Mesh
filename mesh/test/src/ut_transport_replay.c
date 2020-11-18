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

#include "mesh_opt.h"
#include "transport_test_common.h"
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

/**************************************************************************************************
 * This unit test tests all aspects of transport layer replay protection and redundancy checks.
 *************************************************************************************************/
static timer_event_t* mp_incomplete_timer_evt;

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

void timer_sch_reschedule_cb(timer_event_t* p_timer_evt, timestamp_t new_timestamp, int cmock_num_calls)
{
    UNUSED_VARIABLE(cmock_num_calls);

    // Store incomplete timer pointer
    if (new_timestamp == timer_now() + TRANSPORT_SAR_RX_TIMEOUT_DEFAULT_US)
    {
        mp_incomplete_timer_evt = p_timer_evt;
    }
}

void setUp(void)
{
    transport_test_common_setup();
    mesh_config_entry_mock_Init();
    mesh_config_mock_Init();
    mesh_config_entry_set_StubWithCallback(entry_set_cb);
    mesh_config_entry_delete_IgnoreAndReturn(NRF_SUCCESS);
}

void tearDown(void)
{
    transport_test_common_teardown();
    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
}

/*****************************************************************************
* Test functions
*****************************************************************************/

void test_replay_unseg(void)
{
    network_packet_metadata_t meta;
    uint16_t src = 1;
    do_iv_update(10);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(src, 1, m_iv_index));

    // receive a packet with lower seqnum, expect it to fail immediately:
    net_meta_build(src, 0, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // receive a packet with same seqnum, expect it to fail immediately:
    net_meta_build(src, 1, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // receive a packet with lower IV index, expect it to fail immediately:
    net_meta_build(src, 2, m_iv_index - 1, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // receive a packet with higher seqnum on current IV index, should go through:
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, 2, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // should have added the previous packet to replay protection:
    net_meta_build(src, 2, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    do_iv_update(m_iv_index + 1);

    // receive a packet with new, higher IV index, should go through:
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, 0, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // should have added the previous packet to replay protection:
    net_meta_build(src, 0, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // receive a packet with lower IV index, expect it to fail immediately:
    net_meta_build(src, 1, m_iv_index - 1, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // packet with different source, should go through, even on the old IV index:
    src++;
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, 0, m_iv_index-1, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 0, m_iv_index-1));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, 0, m_iv_index)); // doesn't have it on the new index

    // packet that fails decryption, shouldn't be added to replay protection:
    m_decrypt_ok = false;
    net_meta_build(src, 1, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, 1, m_iv_index));

    // send it again with successful decryption, this time it should go through:
    m_decrypt_ok = true;
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, 1, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 1, m_iv_index));

    // check that it didn't affect seqzero cache for the lowest seqzero,
    // since we haven't received any segmented message yet
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 0, m_iv_index - 1, 0));
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 0, m_iv_index, 0));
}

void test_replay_reject_full_list(void)
{
    network_packet_metadata_t meta;
    uint16_t src = 1;
    do_iv_update(10);

    // fill replay cache with different source addresses
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(src++, 1, m_iv_index - 1)); // one on the previous iv index
    for (uint32_t i = 0; i < REPLAY_CACHE_ENTRIES - 1; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(src++, 1, m_iv_index));
    }

    // receive a packet from a new src, should fail and generate replay full evt:
    expect_replay_cache_full(src, m_iv_index & 1, NRF_MESH_RX_FAILED_REASON_REPLAY_CACHE_FULL);
    net_meta_build(src, 1, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // after an update, the entry that was on the previous IV index will be discared
    do_iv_update(m_iv_index + 1);

    // Now, the previous packet will go through, as there's space again:
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, 0, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);
}

void test_sar_frame_skipping_with_seqno_less_seqauth(void)
{
    network_packet_metadata_t meta;
    uint16_t src = 1;
    do_iv_update(10);

    sar_session_t sar_session = {
        .seqzero = 1,
        .segment_count = 1,
        .total_len = PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    // receive a packet with sequence number 0 for seqzero 1
    net_meta_build(src, 0, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // frame should be silently skipped. There will not be further handling.
}

void test_replay_sar_single_segment(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;
    do_iv_update(10);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(src, 1, m_iv_index));

    sar_session_t sar_session = {
        .seqzero = 0, // lower than first entry in the cache!
        .segment_count = 1,
        .total_len = PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    // Receive first segment, should be rejected, as it's already in the cache
    net_meta_build(src, 0, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // Receive first segment again with a new seqnum, but old seqzero.
    // Packet should go through and should be accepted because seqnum is new
    // and seqzero is not in the replay cache yet.
    expect_sar_ctx_alloc();
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_session, 1);
    expect_sar_ctx_free();
    net_meta_build(src, 2, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // the packet should be added to replay, as it was successfully processed:
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, 2, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 2, m_iv_index));

    // Now it should be dropped again since seqnum is in the replay cache
    sar_session.seqzero = 2;
    net_meta_build(src, 2, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    sar_session.seqzero = 3; // new value that isn't present in replay list

    // this time, we'll actually accept the packet
    expect_sar_ctx_alloc();
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_session, 1);
    expect_sar_ctx_free();
    net_meta_build(src, 4, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta); // note: seq is higher than seqzero
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // the packet was added to replay, as it was successfully processed:
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, 4, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 4, m_iv_index));

    // Receive a new packet with the same seqzero, should send a block ack, and
    // the packet should get added to replay protection
    expect_sar_ack(&sar_session, 1);
    net_meta_build(src, 5, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta); // note: seq is higher than seqzero
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // the packet was added to replay, as it was successfully processed:
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, 5, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 5, m_iv_index));
}

/**
 * Test replay protection for SAR with multiple segments. Should add the highest seqnum when
 * the packet is successfully decrypted.
 */
void test_replay_sar_multisegment(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_session = {
        .seqzero = 2,
        .segment_count = 4,
        .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    // Accept the packet, starting a new session:
    expect_sar_ctx_alloc();
    net_meta_build(src, 3, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta); // note: seq is higher than seqzero
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    // shouldn't have added anything to replay yet:
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 2, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, 2, m_iv_index));

    // send the rest of the segments in a shuffled order
    net_meta_build(src, 6, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_session);
    // shouldn't have added anything to replay yet:
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 2, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, 2, m_iv_index));

    net_meta_build(src, 4, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta); // seq is lower than previous segment, shouldn't matter
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_session);
    // shouldn't have added anything to replay yet:
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 2, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, 2, m_iv_index));

    // final segment, gets added to the replay list and pushed to access
    // This is not the highest seq in the sar, so, although this is the last packet to be received,
    // we'll report the seqnum of the packet that was sent last to upper transport (and the replay list)
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_session, 0x0f);
    expect_sar_ctx_free();
    net_meta_build(src, 5, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_session);
    // The highest seq should have been added to replay:
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, 6, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 6, m_iv_index));

    // resend the last segment with a new seqnum, should cause a repeat of the block ack and adding to replay
    expect_sar_ack(&sar_session, 0x0f);
    net_meta_build(src, 8, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_session);
    // should have added this packet to replay:
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, 8, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 8, m_iv_index));

    // Receiving a new sar session with seqzero that's NOT in the replay, but lower than last seqnum in the replay
    // The packet should go through
    sar_session.seqzero = 7;
    sar_session.segment_count = 1;
    sar_session.total_len = 1 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE;
    expect_sar_ctx_alloc();
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_session, 0x01);
    expect_sar_ctx_free();
    net_meta_build(src, 9, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    // should add this packet to replay, since seqzero is new:
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, 9, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 9, m_iv_index));
}

/**
 * Test receiving of an unsegmented message in the middle of a SAR session. Should force all later
 * segments of the sar to come on higher seqnums, but it shouldn't block the SAR from completing.
 *
 * Likewise, the SAR packet shouldn't affect the replay protection on the unsegmented packets until
 * the full SAR is completed.
 */
void test_replay_concurrent_sar_and_unseg(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_session = {
        .seqzero = 0,
        .segment_count = 4,
        .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    // Start a new SAR session:
    expect_sar_ctx_alloc();
    net_meta_build(src, 2, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // Receive all segments except one:
    net_meta_build(src, 3, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_session);
    net_meta_build(src, 4, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_session);

    // receive an unsegmented packet with a lower seq than the sar segment, shouldn't matter:
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, 6, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 1, m_iv_index));
    // Should NOT affect seqzero cache
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 1, m_iv_index, 0));

    // Receive the final sar segment with a lower seq than the unsegmented message. Should be ignored.
    net_meta_build(src, 5, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_session);

    // Receive the final sar segment with a higher seq than the unsegmented message. Should cause full SAR to go through.
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_session, 0x0f);
    expect_sar_ctx_free();
    net_meta_build(src, 8, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_session);
    // seqzero and seqnum are in the cache
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 8, m_iv_index));
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, 8, m_iv_index, sar_session.seqzero));

    // If we receive an unsegmented packet with a lower seq than the last sar segment now, it'll be ignored:
    net_meta_build(src, 7, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);

    // receive an unsegmented packet with a higher seq than the sar segment, should go through
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, 9, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, 9, m_iv_index));
    // Should not affect seqzero cache
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 9, m_iv_index, sar_session.seqzero + 1));
}

/**
 * Test replay protection when there are two sar sessions from the same device at the same time.
 * Only the one with the highest seqauth should go through.
 *
 * Scenario: Session A starts, then session B comes in and cancels it.
 */
void test_replay_concurrent_sars_AB(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_sessions[2] = {
        {
            .seqzero = 0,
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        },
        {
            .seqzero = 1,
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        }
    };

    uint32_t seq = 2; // higher than both seqzeros to avoid invalid seqzero calculations

    // Start session A:
    expect_sar_ctx_alloc();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_sessions[0]);

    // Start session B, should cancel session A
    expect_sar_cancel(NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION);
    expect_sar_ctx_free();
    expect_sar_ctx_alloc();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta); // with a different destination address, doesn't matter.
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_sessions[1]);
    // The highest seqzero should NOT be in the replay cache
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[1].seqzero));

    // Receive all segments except one:
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_sessions[1]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[1]);

    // Receiving packets on session A should have no effect, as its seqauth is lower:
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_sessions[0]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[0]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_sessions[0]);

    // Receive last segment of session B: No ack, since it's a group address
    expect_access_rx(sar_sessions[1].total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_sessions[1]);
    // seqzero and seqnum are in the replay cache
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[1].seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq - 1, m_iv_index));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq, m_iv_index));
}

/**
 * Test replay protection when there are two sar sessions from the same device at the same time.
 * Only the one with the highest seqauth should go through.
 *
 * Scenario: Session B starts, then session A gets blocked.
 */
void test_replay_concurrent_sars_BA(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_sessions[2] = {
        {
            .seqzero = 0,
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        },
        {
            .seqzero = 1,
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        }
    };

    uint32_t seq = 2; // higher than both seqzeros to avoid invalid seqzero calculations

    // Start session B:
    expect_sar_ctx_alloc();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_sessions[1]);
    // Should not add seqzero to the replay cache
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[1].seqzero));

    // Start session A, should be ignored:
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_sessions[0]);
    // Should not add seqzero to the replay cache
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[0].seqzero));

    // Receive all segments except one:
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_sessions[1]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[1]);

    // Receive the rest of session A, should have no effect
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_sessions[0]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[0]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_sessions[0]);

    // Receive last segment of session B: No ack, since it's a group address
    expect_access_rx(sar_sessions[1].total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_sessions[1]);
    // seqzero and seqnum are in the cache
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[1].seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq - 1, m_iv_index));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq, m_iv_index));
}

/**
 * Check that transport rejects a segmented message and doesn't send an ack
 * if we overflow the SAR cache by sending a segmented message from the same source.
 */
void test_replay_overflow_sar_cache(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_session = {
        .seqzero = 0,
        .segment_count = 1,
        .total_len = PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };
    uint32_t seq = 0;
    // Receive enough SAR packets to fill the cache and verify that they generate acks on duplicate RX
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        sar_session.seqzero = seq;
        expect_sar_ctx_alloc();
        expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                        src,
                        NRF_MESH_ADDRESS_TYPE_UNICAST);
        expect_sar_ack(&sar_session, 1);
        expect_sar_ctx_free();
        net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
        packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

        // receive again, should cause ACK, as the packet is in cache:
        expect_sar_ack(&sar_session, 1);
        net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
        packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    }

    // Send one more to pop the first session out of cache
    sar_session.seqzero = seq;
    expect_sar_ctx_alloc();
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                    src,
                    NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_session, 1);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // receive first session again, should not cause ACK, as the packet isn't in cache. Also shouldn't cause duplicate RX.
    sar_session.seqzero = 0;
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
}

/**
 * Check that transport rejects last received segmented message from the same source
 * but sends an ack to it, if we overflow the SAR cache by sending
 * a segmented message from different sources.
 */
void test_replay_overflow_sar_cache_different_sources(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    do_iv_update(10);

    network_packet_metadata_t meta;

    struct
    {
        sar_session_t sar_session;
        uint16_t src;
        uint16_t seq;
    } elements[TRANSPORT_SAR_SESSIONS_MAX];

    for (size_t i = 0 ; i < ARRAY_SIZE(elements); i++)
    {
        elements[i].sar_session.seqzero = 0;
        elements[i].sar_session.segment_count = 1;
        elements[i].sar_session.total_len = 1 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE;

        elements[i].src = i + 1;
        elements[i].seq = 0;
    }

    // Fill SAR RX cache
    for (size_t i = 0; i < ARRAY_SIZE(elements); i++)
    {
        expect_sar_ctx_alloc();
        expect_access_rx(elements[i].sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                        elements[i].src,
                        NRF_MESH_ADDRESS_TYPE_UNICAST);
        expect_sar_ack(&elements[i].sar_session, 1);
        expect_sar_ctx_free();
        net_meta_build(elements[i].src, elements[i].seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
        packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &elements[i].sar_session);
    }

    // Assuming that sender didn't receive ack, retransmit the first segmsg.
    // Node should send ack and reject the packet because it was received.
    expect_sar_ack(&elements[0].sar_session, 1);
    net_meta_build(elements[0].src, elements[0].seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &elements[0].sar_session);
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(elements[0].src, elements[0].seq - 1, m_iv_index, elements[0].sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(elements[0].src, elements[0].seq - 1, m_iv_index));
}

/**
 * Test that the device is able to send an ack on an old session even after starting a new session.
 */
void test_ack_on_completed_session_after_new_session_started(void)
{

    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_sessions[2] = {
        {
            .seqzero = 0,
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        },
        {
            .seqzero = 0, // will be adjusted before the session is used
            .segment_count = 4,
            .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
        }
    };

    uint32_t seq = 2; // higher than both seqzeros to avoid invalid seqzero calculations

    // Start session A
    expect_sar_ctx_alloc();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_sessions[0]);
    // seqzero is not in the replay cache
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[0].seqzero));

    // Receive all segments except one:
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_sessions[0]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[0]);

    // Receive last segment of session A: should go to access
    expect_access_rx(sar_sessions[0].total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_sessions[0], 0x0f);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_sessions[0]);
    // seqzero and seqnum are in the replay cache
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[0].seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq - 1, m_iv_index));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq, m_iv_index));

    // Start session B:
    sar_sessions[1].seqzero = seq;
    expect_sar_ctx_alloc();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_sessions[1]);
    // seqzero is not in the replay cache
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[1].seqzero));

    /* Now, receive a repeated segment of A again. Should NOT cause a block ack, since we started a
     * different session between. */
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[0]);
    // seqzero is in the replay cache, seqnum NOT
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[0].seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq - 1, m_iv_index));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq, m_iv_index));

    // Receive all segments except one:
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_sessions[1]);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[1]);

    // Receive last segment of session B: No ack, since it's a group address
    expect_access_rx(sar_sessions[1].total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_GROUP);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_GROUP, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 3, &sar_sessions[1]);
    // seqzero and seqnum are in the replay cache
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[1].seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq - 1, m_iv_index));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq, m_iv_index));

    /* Now, receive a repeated segment of A again. Should not cause a block ack, since we received a
     * different session between. */
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 2, &sar_sessions[0]);
    // seqzero is in the replay cache, seqnum NOT
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_sessions[0].seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq - 1, m_iv_index));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, seq, m_iv_index));

}

/**
 * Test that the replay protection is able to receive a segmented message
 * even if it got lost the first time, and the source sends an unsegmented message
 * with higher seqnum from before it retransmits the segmented message.
 */
void test_sar_unseg_msg_before_first_segment(void)
{
    timer_sch_reschedule_Ignore();
    timer_sch_abort_Ignore();

    do_iv_update(10);

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_session = {
        .seqzero = 0,
        .segment_count = 1,
        .total_len = 1 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    uint32_t seq = 0;

    // The segmented message with seqauth = 0 is transmitted, but got lost
    // The sender increments seq
    seq++;

    // The sender sends an unsegmented packet before retransmitting the segmented message
    // Transport receives the message and adds seq to the replay cache
    expect_access_rx(10, src, NRF_MESH_ADDRESS_TYPE_UNICAST);
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_unseg_rx(&meta, 10);
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq - 1, m_iv_index));

    // The sender retransmits the segmented message where seqzero is old, but seqnum is new
    // Transport receives the message and adds seq to the replay cache
    expect_sar_ctx_alloc();
    expect_access_rx(sar_session.total_len - PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE,
                     src,
                     NRF_MESH_ADDRESS_TYPE_UNICAST);
    expect_sar_ack(&sar_session, 1);
    expect_sar_ctx_free();
    net_meta_build(src, seq++, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(src, seq - 1, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_TRUE(replay_cache_has_elem(src, seq - 1, m_iv_index));
}

/**
 * Test that the device ignores the SAR session if it was canceled due to incomplete timer timeout.
 */
void test_sar_canceled_rx_session(void)
{
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_cb);
    timer_sch_abort_Ignore();

    network_packet_metadata_t meta;
    uint16_t src = 1;

    sar_session_t sar_session = {
        .seqzero = 2,
        .segment_count = 4,
        .total_len = 4 * PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE,
    };

    // Accept the packet, starting a new session.
    expect_sar_ctx_alloc();
    net_meta_build(src, 2, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta); // note: seq is higher than seqzero
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 0, &sar_session);

    // Check that incomplete timer has been scheduled. */
    TEST_ASSERT_NOT_NULL(mp_incomplete_timer_evt);

    // Send the next segment.
    net_meta_build(src, 3, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta);
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_session);

    // Trigger incomplete timer to time out the SAR session.
    expect_sar_cancel(NRF_MESH_SAR_CANCEL_REASON_TIMEOUT);
    expect_sar_ctx_free();
    mp_incomplete_timer_evt->cb(timer_now() + TRANSPORT_SAR_RX_TIMEOUT_DEFAULT_US, mp_incomplete_timer_evt->p_context);
    // Shouldn't have added anything to replay.
    TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 2, m_iv_index, sar_session.seqzero));
    TEST_ASSERT_FALSE(replay_cache_has_elem(src, 2, m_iv_index));

    // Send the missing fragment. Don't expect SAR session allocation.
    net_meta_build(src, 4, m_iv_index, NRF_MESH_ADDRESS_TYPE_UNICAST, &meta); // note: seq is higher than seqzero
    packet_seg_rx(&meta, PACKET_MESH_TRS_SEG_ACCESS_PDU_MAX_SIZE, 1, &sar_session);
}
