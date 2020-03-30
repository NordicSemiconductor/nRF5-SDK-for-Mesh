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
#ifndef TRANSPORT_INTERNAL_H__
#define TRANSPORT_INTERNAL_H__

#include <stdint.h>
#include <stdbool.h>
#include "transport.h"

/**
 * @defgroup TRANSPORT_INTERNAL Internal Transport declarations, types, and functions
 * @internal
 * @{
 */

/** Offset to the IV Index in SeqAuth. */
#define TRANSPORT_SAR_SEQAUTH_IV_INDEX_OFFSET (NETWORK_SEQNUM_BITS)

/** Mask for SeqZero in SeqAuth. */
#define TRANSPORT_SAR_SEQZERO_MASK (TRANSPORT_SAR_SEQNUM_DIFF_MAX)

/** Receivers for incoming transport packets. */
typedef enum
{
    TRANSPORT_PACKET_RECEIVER_NONE      = 0,        /**< No one is receiving this packet. */
    TRANSPORT_PACKET_RECEIVER_SELF      = (1 << 0), /**< This node is receiving this packet. */
    TRANSPORT_PACKET_RECEIVER_FRIEND    = (1 << 1), /**< A friendship device is receiving this packet. */
} transport_packet_receiver_t;

typedef struct
{
    uint16_t seq_zero;
    uint8_t segment_offset;
    uint8_t last_segment;
} transport_segmentation_metadata_t;

typedef struct
{
    bool segmented;
    transport_packet_receiver_t receivers;
    uint8_t mic_size;
    union
    {
        struct
        {
            bool using_app_key;
            uint8_t app_key_id;
        } access;
        struct
        {
            transport_control_opcode_t opcode;
        } control;
    } type;
    transport_segmentation_metadata_t segmentation;
    network_packet_metadata_t net;
    const nrf_mesh_application_secmat_t * p_security_material;
    nrf_mesh_tx_token_t token;
    core_tx_bearer_selector_t tx_bearer_selector; /**< The bearer on which the outgoing packets are to be sent on. */
} transport_packet_metadata_t;

/**
 * Derives the sequence number (SEQ) of the first segment.
 *
 * @param[in] sequence_number   SEQ of any of the segments.
 * @param[in] seq_zero          SeqZero derived from the segmented message.
 *
 * @returns SEQ of the first SAR segment.
 */
static inline uint32_t transport_sar_first_seq_num_get(uint32_t sequence_number, uint16_t seq_zero)
{
    if ((sequence_number & TRANSPORT_SAR_SEQZERO_MASK) < seq_zero)
    {
        return ((sequence_number - ((sequence_number & TRANSPORT_SAR_SEQZERO_MASK) - seq_zero) - (TRANSPORT_SAR_SEQZERO_MASK + 1)));
    }
    else
    {
        return ((sequence_number - ((sequence_number & TRANSPORT_SAR_SEQZERO_MASK) - seq_zero)));
    }
}

/**
 * Derives SeqAuth from the IV Index, SeqZero, and the sequence number (SEQ) of
 * any of the segments.
 *
 * The SeqAuth is a 56-bit value, composed of the IV index and the sequence
 * number (SEQ) of the first segment:
 *
 * |  |  |  |  |  |  |  |
 * |56       24|23     0|
 * | IV Index  |  SEQ   |
 *
 * The SeqZero is the least significant 13 bits of the SeqAuth, included in
 * the Segmented message and the Segment Acknowledgment message.
 *
 * @param[in] iv_index          The current IV Index.
 * @param[in] sequence_number   SEQ of any of the segments.
 * @param[in] seq_zero          SeqZero derived from the segmented message.
 */
static inline uint64_t transport_sar_seqauth_get(uint32_t iv_index, uint32_t sequence_number, uint16_t seq_zero)
{
    return ((uint64_t) iv_index << TRANSPORT_SAR_SEQAUTH_IV_INDEX_OFFSET)
            + transport_sar_first_seq_num_get(sequence_number, seq_zero);
}

/** @} */

#endif /* TRANSPORT_INTERNAL_H__ */
