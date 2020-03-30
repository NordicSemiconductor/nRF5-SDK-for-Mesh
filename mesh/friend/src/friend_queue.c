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
#include "friend_queue.h"

/*****************************************************************************
* Static functions
*****************************************************************************/

static friend_packet_t * packet_from_queue_elem(queue_elem_t * p_elem)
{
    return (p_elem ? PARENT_BY_FIELD_GET(friend_packet_t, queue_elem, p_elem) : NULL);
}

static friend_packet_t * queue_pop_packet(queue_t * p_queue)
{
    return packet_from_queue_elem(queue_pop(p_queue));
}

static void queue_push_packet(queue_t * p_queue, friend_packet_t * p_packet)
{
    queue_push(p_queue, &p_packet->queue_elem);
}

static void packet_free(friend_queue_t * p_queue, friend_packet_t * p_packet)
{
    queue_push_packet(&p_queue->free_packets, p_packet);
#if FRIEND_DEBUG
    p_queue->stats.packets.free++;
#endif
}

static void sar_session_finalize(friend_queue_t * p_queue, friend_queue_sar_session_t * p_session, bool success)
{
#if FRIEND_DEBUG
    if (success)
    {
        p_queue->stats.sar_sessions.successful++;
    }
    else
    {
        QUEUE_FOREACH(&p_session->segments, it)
        {
            p_queue->stats.packets.free++;
        }
        p_queue->stats.sar_sessions.failed++;
    }
    p_queue->stats.sar_sessions.pending--;
#endif

    queue_t * p_target_queue = (success ? &p_queue->committed_packets : &p_queue->free_packets);
    queue_merge(p_target_queue, &p_session->segments);
    p_session->src = NRF_MESH_ADDR_UNASSIGNED;
}

static friend_queue_sar_session_t * sar_session_find(friend_queue_t * p_queue, uint16_t src)
{
    for (uint32_t i = 0; i < ARRAY_SIZE(p_queue->sar_sessions); ++i)
    {
        if (p_queue->sar_sessions[i].src == src)
        {
            return &p_queue->sar_sessions[i];
        }
    }
    return NULL;
}

static friend_packet_t * packet_alloc(friend_queue_t * p_queue, const transport_packet_metadata_t * p_metadata)
{
    friend_packet_t * p_packet = queue_pop_packet(&p_queue->free_packets);

    if (p_packet != NULL)
    {
#if FRIEND_DEBUG
        p_queue->stats.packets.free--;
        p_queue->stats.packets.min_free = MIN(p_queue->stats.packets.min_free, p_queue->stats.packets.free);
#endif
        return p_packet;
    }

    // There are no free packets left, so we need to start discarding:
    QUEUE_FOREACH(&p_queue->committed_packets, it)
    {
        /* According to @tagMeshSp section 3.5.5, we should discard the
         * oldest packet that isn't an update packet. To avoid leaving partial SAR packets in the
         * queue, we'll remove consecutive packets with the same set_id. */
        friend_packet_t * p_committed_packet = packet_from_queue_elem(*it.pp_elem);
        NRF_MESH_ASSERT_DEBUG(p_committed_packet != NULL);

        // No update packets:
        if (p_committed_packet->net_metadata.control_packet &&
            packet_mesh_trs_control_opcode_get(&p_committed_packet->packet) == TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE)
        {
            continue;
        }

        if (p_packet != NULL)
        {
            // Stop removing packets when a different set_id is encountered:
            if (p_packet->set_id != p_committed_packet->set_id)
            {
                break;
            }

            // we're about to pop another packet from the queue, free the current choice:
            packet_free(p_queue, p_packet);
        }

        p_packet = p_committed_packet;
        queue_iterator_elem_remove(&it);
        p_queue->user_has_packet = false;

#if FRIEND_DEBUG
        p_queue->stats.packets.discarded++;
#endif
    }

    return p_packet;
}

static bool is_segack(const packet_mesh_trs_packet_t * p_packet, bool control_packet, uint16_t * p_seqzero)
{
    if (control_packet &&
        packet_mesh_trs_control_opcode_get(p_packet) == TRANSPORT_CONTROL_OPCODE_SEGACK)
    {
        const packet_mesh_trs_control_packet_t * p_segack = (const packet_mesh_trs_control_packet_t *) packet_mesh_trs_unseg_payload_get(p_packet);
        *p_seqzero = packet_mesh_trs_control_segack_seqzero_get(p_segack);
        return true;
    }
    return false;
}

static void remove_duplicate_segack(friend_queue_t * p_queue,
                                    const packet_mesh_trs_packet_t * p_packet,
                                    const transport_packet_metadata_t * p_metadata)
{
    uint16_t seqzero;
    if (is_segack(p_packet, p_metadata->net.control_packet, &seqzero))
    {
        // Look for a segack packet with the same src, dst and seqzero parameters:
        QUEUE_FOREACH(&p_queue->committed_packets, it)
        {
            friend_packet_t * p_queue_packet = PARENT_BY_FIELD_GET(friend_packet_t, queue_elem, *it.pp_elem);

            uint16_t queue_packet_seqzero;
            bool queue_packet_is_segack = is_segack(&p_queue_packet->packet,
                                                    p_queue_packet->net_metadata.control_packet,
                                                    &queue_packet_seqzero);

            if (queue_packet_is_segack &&
                p_metadata->net.src       == p_queue_packet->net_metadata.src &&
                p_metadata->net.dst.value == p_queue_packet->net_metadata.dst &&
                seqzero == queue_packet_seqzero)
            {
                // free the old segack:
                queue_iterator_elem_remove(&it);
                packet_free(p_queue, p_queue_packet);
                return;
            }
        }
    }
}

static bool is_segmented(const packet_mesh_trs_packet_t * p_packet)
{
    return packet_mesh_trs_common_seg_get(p_packet);
}

static uint64_t get_seqauth(const friend_packet_t *p_packet)
{
    uint16_t seqzero = packet_mesh_trs_seg_seqzero_get(&p_packet->packet);
    uint64_t seqauth = transport_sar_seqauth_get(p_packet->net_metadata.iv_index,
                                                 p_packet->net_metadata.seqnum,
                                                 seqzero);
    return seqauth;
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void friend_queue_init(friend_queue_t * p_queue)
{
    memset(p_queue, 0, sizeof(friend_queue_t));

    queue_init(&p_queue->committed_packets);
    queue_init(&p_queue->free_packets);

    for (uint32_t i = 0; i < ARRAY_SIZE(p_queue->sar_sessions); ++i)
    {
        queue_init(&p_queue->sar_sessions[i].segments);
    }

    // All packets are free:
    for (uint32_t i = 0; i < ARRAY_SIZE(p_queue->buffer); ++i)
    {
        queue_push_packet(&p_queue->free_packets, &p_queue->buffer[i]);
    }

#if FRIEND_DEBUG
        p_queue->stats.packets.free = MESH_FRIEND_QUEUE_SIZE;
        p_queue->stats.packets.min_free = MESH_FRIEND_QUEUE_SIZE;
#endif
}

const friend_packet_t * friend_queue_packet_get(friend_queue_t * p_queue)
{
    queue_elem_t * p_elem = queue_peek(&p_queue->committed_packets);
    if (p_elem == NULL)
    {
        return NULL;
    }

    p_queue->user_has_packet = true;

#if FRIEND_DEBUG
    p_queue->stats.packets.sent++;
#endif

    return PARENT_BY_FIELD_GET(friend_packet_t, queue_elem, p_elem);
}

void friend_queue_packet_release(friend_queue_t * p_queue)
{
    if (p_queue->user_has_packet)
    {
        friend_packet_t * p_packet = queue_pop_packet(&p_queue->committed_packets);
        NRF_MESH_ASSERT_DEBUG(p_packet != NULL);
        packet_free(p_queue, p_packet);
    }
    p_queue->user_has_packet = false;
}

void friend_queue_packet_push(friend_queue_t * p_queue,
                       const packet_mesh_trs_packet_t * p_packet,
                       uint8_t length,
                       const transport_packet_metadata_t * p_metadata,
                       core_tx_role_t role)
{
    remove_duplicate_segack(p_queue, p_packet, p_metadata);

    friend_packet_t * p_friend_packet = packet_alloc(p_queue, p_metadata);
    if (p_friend_packet == NULL)
    {
        return;
    }

    p_friend_packet->net_metadata.ttl            = p_metadata->net.ttl;
    p_friend_packet->net_metadata.control_packet = p_metadata->net.control_packet;
    p_friend_packet->net_metadata.src            = p_metadata->net.src;
    p_friend_packet->net_metadata.dst            = p_metadata->net.dst.value;
    p_friend_packet->net_metadata.seqnum         = p_metadata->net.internal.sequence_number;
    p_friend_packet->net_metadata.iv_index       = p_metadata->net.internal.iv_index;
    p_friend_packet->role = role;
    p_friend_packet->length = length;
    NRF_MESH_ASSERT_DEBUG(length <= sizeof(p_friend_packet->packet));
    memcpy(&p_friend_packet->packet, p_packet, length);

    if (p_metadata->segmented)
    {
        friend_queue_sar_session_t * p_session = sar_session_find(p_queue, p_metadata->net.src);
        if (p_session == NULL)
        {
            // Need to allocate new session:
            p_session = sar_session_find(p_queue, NRF_MESH_ADDR_UNASSIGNED);

            if (p_session == NULL)
            {
                return;
            }

            p_session->set_id = p_queue->next_set_id++;
            p_session->src = p_metadata->net.src;

#if FRIEND_DEBUG
            p_queue->stats.sar_sessions.pending++;
#endif
        }

        p_friend_packet->set_id = p_session->set_id;
        queue_push_packet(&p_session->segments, p_friend_packet);
    }
    else
    {
        p_friend_packet->set_id = p_queue->next_set_id++;
        queue_push_packet(&p_queue->committed_packets, p_friend_packet);
    }
}

void friend_queue_sar_complete(friend_queue_t * p_queue, uint16_t src, bool success)
{
    friend_queue_sar_session_t * p_session = sar_session_find(p_queue, src);
    if (p_session != NULL)
    {
        sar_session_finalize(p_queue, p_session, success);
    }
}

bool friend_queue_sar_exists(friend_queue_t * p_queue, uint16_t src, uint64_t seqauth)
{
    QUEUE_FOREACH(&p_queue->committed_packets, it)
    {
        friend_packet_t * p_queue_packet = PARENT_BY_FIELD_GET(friend_packet_t, queue_elem, *it.pp_elem);

        if (p_queue_packet->net_metadata.src == src
            && is_segmented(&p_queue_packet->packet)
            && (get_seqauth(p_queue_packet) == seqauth))
        {
            return true;
        }
    }

    return false;
}

bool friend_queue_is_empty(const friend_queue_t * p_queue)
{
    return (queue_peek(&p_queue->committed_packets) == NULL);
}

void friend_queue_clear(friend_queue_t * p_queue)
{
    friend_queue_init(p_queue);
}

uint32_t friend_queue_packet_counter_get(friend_queue_t * p_queue)
{
    uint32_t pkt_counter = 0;

    QUEUE_FOREACH(&p_queue->committed_packets, it)
    {
        pkt_counter++;
    }

    return pkt_counter;
}
