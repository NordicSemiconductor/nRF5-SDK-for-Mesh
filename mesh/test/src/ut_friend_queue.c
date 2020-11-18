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

#include "friend_queue.h"
#include "bitfield.h"
#include "test_assert.h"

static friend_queue_t m_queue;
static const transport_packet_metadata_t m_initial_metadata = {
    .net = {
        .src = 0x1234,
        .dst.value = 0xfefe,
        .control_packet = false,
        .internal = {
            .sequence_number = 10000,
            .iv_index = 12345678,
        },
        .ttl = 43,
    },
    .segmented = false,
    .segmentation = {
        .seq_zero = 4000,
        .segment_offset = 0,
        .last_segment = 8,
    },
};

/** Check that all packets are present in one and exactly one queue. */
static void verify_packets_in_queues(void)
{
    uint32_t packets_found[BITFIELD_BLOCK_COUNT(MESH_FRIEND_QUEUE_SIZE)] = {0};

    queue_t * p_queues[2 + ARRAY_SIZE(m_queue.sar_sessions)] = {
        &m_queue.committed_packets,
        &m_queue.free_packets
    };
    for (uint32_t i = 0; i < ARRAY_SIZE(m_queue.sar_sessions); ++i)
    {
        p_queues[i + 2] = &m_queue.sar_sessions[i].segments;
    }

    for (uint32_t i = 0; i < ARRAY_SIZE(p_queues); ++i)
    {
        for (queue_elem_t * p_elem = p_queues[i]->p_front; p_elem != NULL; p_elem = p_elem->p_next)
        {
            uint32_t index = (PARENT_BY_FIELD_GET(friend_packet_t, queue_elem, p_elem) - &m_queue.buffer[0]);
            TEST_ASSERT_FALSE_MESSAGE(bitfield_get(packets_found, index), "Packet exists in two queues");
            bitfield_set(packets_found, index);
        }
    }
    TEST_ASSERT_TRUE(bitfield_is_all_set(packets_found, MESH_FRIEND_QUEUE_SIZE));
}

static void friend_packet_has_net_metadata(const friend_packet_t * p_packet, const network_packet_metadata_t * p_metadata)
{
    TEST_ASSERT_EQUAL(p_packet->net_metadata.ttl, p_metadata->ttl);
    TEST_ASSERT_EQUAL(p_packet->net_metadata.seqnum, p_metadata->internal.sequence_number);
    TEST_ASSERT_EQUAL(p_packet->net_metadata.iv_index, p_metadata->internal.iv_index);
    TEST_ASSERT_EQUAL(p_packet->net_metadata.dst, p_metadata->dst.value);
    TEST_ASSERT_EQUAL(p_packet->net_metadata.control_packet, p_metadata->control_packet);
    TEST_ASSERT_EQUAL(p_packet->net_metadata.src, p_metadata->src);
}

void setUp(void)
{
    friend_queue_init(&m_queue);
}

void tearDown(void)
{
    // Check that all packets are still accounted for:
    verify_packets_in_queues();
}
/*****************************************************************************
* Test functions
*****************************************************************************/
void test_unseg_add(void)
{
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));
    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    friend_packet_has_net_metadata(p_packet, &metadata.net);
    TEST_ASSERT_EQUAL(CORE_TX_ROLE_RELAY, p_packet->role);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&packet, &p_packet->packet, 8);

    // Should pop the same packet again, as long as we haven't freed it:
    TEST_ASSERT_EQUAL_PTR(p_packet, friend_queue_packet_get(&m_queue));

    // Fill the queue:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE - 1; ++i)
    {
        metadata.net.internal.sequence_number++;
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    }

    // Check that all packets are still accounted for:
    verify_packets_in_queues();

    // should still pop the same first packet:
    TEST_ASSERT_EQUAL_PTR(p_packet, friend_queue_packet_get(&m_queue));

    // Add one more to overflow the queue:
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // This made us delete the first packet, and a new one should now appear when we pop:
    TEST_ASSERT_NOT_EQUAL(p_packet, friend_queue_packet_get(&m_queue));
    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_EQUAL(m_initial_metadata.net.internal.sequence_number + 1, p_packet->net_metadata.seqnum); // This is the second packet we pushed.

    // Check that all packets are still accounted for:
    verify_packets_in_queues();

    // release the popped packet:
    friend_queue_packet_release(&m_queue);

    // pop all the packets:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE - 1; ++i)
    {
        p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        // should come in order:
        TEST_ASSERT_EQUAL(m_initial_metadata.net.internal.sequence_number + 2 + i, p_packet->net_metadata.seqnum);
        friend_queue_packet_release(&m_queue);
    }

    // no more packets to pop:
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));
}

void test_sar_add(void)
{
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));
    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;
    metadata.segmented = true;

    // push a segmented packet:
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // Shouldn't be able to pop it, it's not actually added to the queue yet:
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Finalize the SAR session, should make it poppable:
    friend_queue_sar_complete(&m_queue, metadata.net.src, true);
    const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    friend_packet_has_net_metadata(p_packet, &metadata.net);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&packet, &p_packet->packet, 8);
    friend_queue_packet_release(&m_queue);

    // start and fail a SAR transmission:
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    friend_queue_sar_complete(&m_queue, metadata.net.src, false);

    // Shouldn't be able to pop it, it's not actually added to the queue when it fails:
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Start concurrent SAR transmissions from different senders:
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        metadata.net.src = 0x1000 + i;
        metadata.net.internal.sequence_number = 0;
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
        metadata.net.internal.sequence_number++;
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    }
    // Shouldn't be able to pop any packets, they're not actually added to the queue yet:
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Should ignore attempt to add another session, as the list is full.
    metadata.net.src = 0x1000 + TRANSPORT_SAR_SESSIONS_MAX;
    metadata.net.internal.sequence_number = 0;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // End all the sessions in reverse order:
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        uint16_t src = 0x1000 + TRANSPORT_SAR_SESSIONS_MAX - i - 1;
        friend_queue_sar_complete(&m_queue, src, true);
    }

    // Should pop the sessions in reverse order, but the packets in the session should be in the received order:
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        metadata.net.src = 0x1000 + TRANSPORT_SAR_SESSIONS_MAX - i - 1;

        metadata.net.internal.sequence_number = 0;
        p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        friend_packet_has_net_metadata(p_packet, &metadata.net);
        friend_queue_packet_release(&m_queue);

        metadata.net.internal.sequence_number = 1;
        p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        friend_packet_has_net_metadata(p_packet, &metadata.net);
        friend_queue_packet_release(&m_queue);
    }

    // Must reset the friend queue before tearDown, as we dropped a packet in the assert:
    friend_queue_init(&m_queue);
}

void test_free(void)
{
    // Releasing a packet from an empty queue shouldn't have any effect:
    friend_queue_packet_release(&m_queue);

    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // Releasing before getting shouldn't have any effect:
    friend_queue_packet_release(&m_queue);

    // get and release the pushed packet:
    TEST_ASSERT_NOT_NULL(friend_queue_packet_get(&m_queue));
    friend_queue_packet_release(&m_queue);

    // Now, there's nothing to get.
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Fill the queue:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        metadata.net.internal.sequence_number++;
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    }

    // Releasing before getting shouldn't have any effect:
    friend_queue_packet_release(&m_queue);

    // get a packet
    const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);

    // should still pop the same first packet:
    TEST_ASSERT_EQUAL_PTR(p_packet, friend_queue_packet_get(&m_queue));

    // Overflow the queue, should make us release the packet automatically:
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // should return a new packet this time:
    TEST_ASSERT_NOT_EQUAL(p_packet, friend_queue_packet_get(&m_queue));

    // pop all the packets:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        // release multiple times, only the first should have any effect:
        friend_queue_packet_release(&m_queue);
        friend_queue_packet_release(&m_queue);
        friend_queue_packet_release(&m_queue);
        friend_queue_packet_release(&m_queue);
    }

    // Now, there's nothing to get.
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));
}

void test_overflow(void)
{
    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;

    // Fill the queue with regular packets:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
        metadata.net.internal.sequence_number++;
    }

    // overflow by 1:
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    metadata.net.internal.sequence_number++;

    // Should have removed the first packet. When we pop, we should get initial seqnum + 1, and the rest in order:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        TEST_ASSERT_EQUAL(m_initial_metadata.net.internal.sequence_number + 1 + i, p_packet->net_metadata.seqnum);
        friend_queue_packet_release(&m_queue);
    }

    // Fill the queue with packets from a SAR session:
    metadata.segmented = true;
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
        metadata.net.internal.sequence_number++;
    }
    // Complete the session to move the packets to the queue:
    friend_queue_sar_complete(&m_queue, metadata.net.src, true);

    verify_packets_in_queues();

    // If we overflow now, we'll delete all the packets from the SAR session
    metadata.segmented = false;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    verify_packets_in_queues();

    // The only poppable packet should be the last one:
    const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    friend_packet_has_net_metadata(p_packet, &metadata.net);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&packet, &p_packet->packet, 8);
    friend_queue_packet_release(&m_queue);
    // Now, there's nothing to get.
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Fill the queue with packets from a SAR session:
    metadata.segmented = true;
    metadata.segmentation.last_segment = MESH_FRIEND_QUEUE_SIZE;
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
        metadata.net.internal.sequence_number++;
    }

    // Push a packet from a different SAR session, should just be ignored, as there's no memory left for it:
    metadata.net.src++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    friend_queue_sar_complete(&m_queue, metadata.net.src, true);

    // Now, there's nothing to get, as the long SAR session hasn't been pushed yet.
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));
}

void test_segack(void)
{
    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;
    metadata.net.control_packet = true;
    packet_mesh_trs_control_opcode_set(&packet, TRANSPORT_CONTROL_OPCODE_SEGACK);
    packet_mesh_trs_common_seg_set(&packet, false);
    packet_mesh_trs_control_packet_t * p_segack = (packet_mesh_trs_control_packet_t *) packet_mesh_trs_unseg_payload_get(&packet);
    packet_mesh_trs_control_segack_seqzero_set(p_segack, 1234);
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0x00000001);

    // Push a segack
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // Push segack for the same SAR session again
    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0xABABABAB);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // We should only be able to pop the last one, as the first one was removed as a duplicate:
    const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    friend_packet_has_net_metadata(p_packet, &metadata.net);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&packet, &p_packet->packet, p_packet->length);
    friend_queue_packet_release(&m_queue);

    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Test that the same thing works in the beginning of the queue too:

    // push a segack
    metadata.net.control_packet = true;
    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0x00000001);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // push a normal packet
    metadata.net.control_packet = false;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // push a normal packet
    metadata.net.control_packet = false;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // push a duplicate segack
    metadata.net.control_packet = true;
    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0xABABABAB);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // Should pop two normal packets, then the last segack:
    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    TEST_ASSERT_FALSE(p_packet->net_metadata.control_packet);
    friend_queue_packet_release(&m_queue);

    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    TEST_ASSERT_FALSE(p_packet->net_metadata.control_packet);
    friend_queue_packet_release(&m_queue);

    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    friend_packet_has_net_metadata(p_packet, &metadata.net);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&packet, &p_packet->packet, p_packet->length);
    friend_queue_packet_release(&m_queue);

    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Test that the same thing works in the middle of the queue too:

    // push a segack
    metadata.net.control_packet = true;
    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0x00000001);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // push a normal packet
    metadata.net.control_packet = false;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // push a normal packet
    metadata.net.control_packet = false;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // push a duplicate segack
    metadata.net.control_packet = true;
    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0xABABABAB);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // Should pop two normal packets, then the last segack:
    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    TEST_ASSERT_FALSE(p_packet->net_metadata.control_packet);
    friend_queue_packet_release(&m_queue);

    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    TEST_ASSERT_FALSE(p_packet->net_metadata.control_packet);
    friend_queue_packet_release(&m_queue);

    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    friend_packet_has_net_metadata(p_packet, &metadata.net);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&packet, &p_packet->packet, p_packet->length);
    friend_queue_packet_release(&m_queue);

    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Test that the same thing works in the end of the queue too:

    // push a normal packet
    metadata.net.control_packet = false;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // push a normal packet
    metadata.net.control_packet = false;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // push a segack
    metadata.net.control_packet = true;
    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0x00000001);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // push a duplicate segack
    metadata.net.control_packet = true;
    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0xABABABAB);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // Should pop two normal packets, then the last segack:
    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    TEST_ASSERT_FALSE(p_packet->net_metadata.control_packet);
    friend_queue_packet_release(&m_queue);

    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    TEST_ASSERT_FALSE(p_packet->net_metadata.control_packet);
    friend_queue_packet_release(&m_queue);

    p_packet = friend_queue_packet_get(&m_queue);
    TEST_ASSERT_NOT_NULL(p_packet);
    friend_packet_has_net_metadata(p_packet, &metadata.net);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&packet, &p_packet->packet, p_packet->length);
    friend_queue_packet_release(&m_queue);

    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));

    // Push segacks for different sessions, shouldn't remove any:

    metadata.net.control_packet = true;
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0x00000001);

    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    metadata.net.src++;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    metadata.net.dst.value++;
    metadata.net.internal.sequence_number++;
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    metadata.net.internal.sequence_number++;
    packet_mesh_trs_control_segack_seqzero_set(p_segack, 1234 + 1);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    for (uint32_t i = 0; i < 4; ++i)
    {
        p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        TEST_ASSERT_TRUE(p_packet->net_metadata.control_packet);
        friend_queue_packet_release(&m_queue);
    }
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));
}

void test_stats(void)
{
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE, m_queue.stats.packets.min_free);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE, m_queue.stats.packets.free);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.discarded);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.sent);

    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;

    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 1, m_queue.stats.packets.min_free);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 1, m_queue.stats.packets.free);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.discarded);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.sent);

    // Fill the queue with regular packets:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE - 1; ++i)
    {
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
        metadata.net.internal.sequence_number++;
    }

    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.min_free);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.free);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.discarded);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.sent);

    // overflow by 1:
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    metadata.net.internal.sequence_number++;

    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.min_free);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.free);
    TEST_ASSERT_EQUAL(1, m_queue.stats.packets.discarded);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.sent);


    // pop all the packets:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        TEST_ASSERT_NOT_NULL(friend_queue_packet_get(&m_queue));
        friend_queue_packet_release(&m_queue);

        TEST_ASSERT_EQUAL(0, m_queue.stats.packets.min_free);
        TEST_ASSERT_EQUAL(i + 1, m_queue.stats.packets.free);
        TEST_ASSERT_EQUAL(1, m_queue.stats.packets.discarded);
        TEST_ASSERT_EQUAL(i + 1, m_queue.stats.packets.sent);
    }

    // Fill the queue with SAR packets of the same src+seqzero
    metadata.segmented = true;
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE; ++i)
    {
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    }
    TEST_ASSERT_EQUAL(1, m_queue.stats.sar_sessions.pending);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.free);
    // commit the sar session:
    friend_queue_sar_complete(&m_queue, metadata.net.src, true);
    TEST_ASSERT_EQUAL(0, m_queue.stats.sar_sessions.pending);
    TEST_ASSERT_EQUAL(0, m_queue.stats.packets.free);
    TEST_ASSERT_EQUAL(1, m_queue.stats.sar_sessions.successful);

    // Overflow by 1. Should cause the queue to discard all the committed sar packets, as they have the same set_id:
    metadata.segmented = false;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 1, m_queue.stats.packets.free);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE + 1, m_queue.stats.packets.discarded);
    (void) friend_queue_packet_get(&m_queue);
    friend_queue_packet_release(&m_queue);


    // push a segmented packet:
    metadata.segmented = true;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    TEST_ASSERT_EQUAL(1, m_queue.stats.sar_sessions.pending);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 1, m_queue.stats.packets.free);

    friend_queue_sar_complete(&m_queue, metadata.net.src, true);
    TEST_ASSERT_EQUAL(0, m_queue.stats.sar_sessions.pending);
    TEST_ASSERT_EQUAL(2, m_queue.stats.sar_sessions.successful);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 1, m_queue.stats.packets.free);

    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    TEST_ASSERT_EQUAL(1, m_queue.stats.sar_sessions.pending);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 2, m_queue.stats.packets.free);

    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    TEST_ASSERT_EQUAL(1, m_queue.stats.sar_sessions.pending);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 3, m_queue.stats.packets.free);

    friend_queue_sar_complete(&m_queue, metadata.net.src, false);
    TEST_ASSERT_EQUAL(0, m_queue.stats.sar_sessions.pending);
    TEST_ASSERT_EQUAL(2, m_queue.stats.sar_sessions.successful);
    TEST_ASSERT_EQUAL(1, m_queue.stats.sar_sessions.failed);
    TEST_ASSERT_EQUAL(MESH_FRIEND_QUEUE_SIZE - 1, m_queue.stats.packets.free);

    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        metadata.net.src = 1000 + i;
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
        TEST_ASSERT_EQUAL(i + 1, m_queue.stats.sar_sessions.pending);
    }

    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX; ++i)
    {
        metadata.net.src = 1000 + i;
        friend_queue_sar_complete(&m_queue, metadata.net.src, true);
        TEST_ASSERT_EQUAL(TRANSPORT_SAR_SESSIONS_MAX - i - 1, m_queue.stats.sar_sessions.pending);
    }
}

void test_dont_discard_friend_update(void)
{

    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;
    metadata.net.control_packet = true;
    packet_mesh_trs_control_opcode_set(&packet, TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE);
    packet_mesh_trs_common_seg_set(&packet, false);
    packet_mesh_trs_control_packet_t * p_segack = (packet_mesh_trs_control_packet_t *) packet_mesh_trs_unseg_payload_get(&packet);
    packet_mesh_trs_control_segack_seqzero_set(p_segack, 1234);
    packet_mesh_trs_control_segack_block_ack_set(p_segack, 0x00000001);

    // Push two friend updates
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // one normal packet
    metadata.net.control_packet = false;
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // and another friend update
    metadata.net.control_packet = true;
    friend_queue_packet_push(&m_queue,
                      &packet,
                      PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE,
                      &metadata,
                      CORE_TX_ROLE_RELAY);

    // Fill the rest of the queue with other packets:
    metadata.net.control_packet = false;

    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE - 4; ++i)
    {
        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    }

    // Overflow by 1:
    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);

    // Should have discarded the first non-update packet, so we'll get the three update packets when we pop:
    for (uint32_t i = 0; i < 3; ++i)
    {
        const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        TEST_ASSERT_TRUE(p_packet->net_metadata.control_packet);
        TEST_ASSERT_EQUAL(TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE, packet_mesh_trs_control_opcode_get(&p_packet->packet));
        friend_queue_packet_release(&m_queue);
    }

    // the rest will be normal packets:
    for (uint32_t i = 0; i < MESH_FRIEND_QUEUE_SIZE - 3; ++i)
    {
        const friend_packet_t * p_packet = friend_queue_packet_get(&m_queue);
        TEST_ASSERT_NOT_NULL(p_packet);
        TEST_ASSERT_FALSE(p_packet->net_metadata.control_packet);
        friend_queue_packet_release(&m_queue);
    }

    // no more packets:
    TEST_ASSERT_NULL(friend_queue_packet_get(&m_queue));
}

void test_check_sar_seqauth_exists(void)
{
    packet_mesh_trs_packet_t packet = {.pdu = {1, 2, 3, 4, 5, 6, 7, 8, 9}};
    transport_packet_metadata_t metadata = m_initial_metadata;
    metadata.segmented = true;
    metadata.net.internal.iv_index = 0;
    metadata.net.internal.sequence_number = 1;
    packet_mesh_trs_common_seg_set(&packet, true);
    packet_mesh_trs_seg_seqzero_set(&packet, 1);

    friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
    // The packet is not added yet
    TEST_ASSERT_FALSE(friend_queue_sar_exists(&m_queue, metadata.net.src, 1));

    // Now the packet should be in the queue
    friend_queue_sar_complete(&m_queue, metadata.net.src, true);
    TEST_ASSERT_TRUE(friend_queue_sar_exists(&m_queue, metadata.net.src, 1));

    // Overflow the queue so that the first packet will be dropped from the queue
    uint16_t seqzero = 1;
    for (size_t i = 0; i < MESH_FRIEND_QUEUE_SIZE + 1; i++)
    {
        metadata.net.internal.sequence_number++;
        packet_mesh_trs_seg_seqzero_set(&packet, seqzero++);

        friend_queue_packet_push(&m_queue, &packet, 8, &metadata, CORE_TX_ROLE_RELAY);
        friend_queue_sar_complete(&m_queue, metadata.net.src, true);

        // seqauth in this case is equal to seqzero of the added packet
        TEST_ASSERT_TRUE(friend_queue_sar_exists(&m_queue, metadata.net.src, seqzero - 1));
    }

    // Check that packet doesn't exist anymore in the queue
    TEST_ASSERT_FALSE(friend_queue_sar_exists(&m_queue, metadata.net.src, 1));
}
