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
#ifndef FRIEND_INTERNAL_H__
#define FRIEND_INTERNAL_H__

#include <stdbool.h>
#include <stdint.h>
#include "net_packet.h"
#include "packet_mesh.h"
#include "core_tx.h"
#include "transport_internal.h"

/**
 * @defgroup FRIEND_INTERNAL Friend internal API
 * @internal
 * @{
 */

/**
 * Passes a transport packet to the Friend module.
 *
 * Adds the packet to the appropriate Friend Queue or Queues.
 *
 * @param[in] p_packet 		Packet to process.
 * @param[in] length 		Length of the packet.
 * @param[in] p_metadata 	Transport packet metadata tied to the packet.
 * @param[in] role 			Role this device has for the packet.
 */
void friend_packet_in(const packet_mesh_trs_packet_t * p_packet,
                      uint8_t length,
                      const transport_packet_metadata_t * p_metadata,
                      core_tx_role_t role);

/**
 * Checks whether the Friend module needs packets with the given metadata.
 *
 * @param[in] p_metadata Metadata tied to the packet.
 *
 * @retval true Friend module needs the packet.
 * @retval false Friend module doesn't need the packet.
 */
bool friend_needs_packet(const transport_packet_metadata_t * p_metadata);

/**
 * Notifies the Friend module that a SAR session is complete.
 *
 * The Friend needs this information to know when it can push the SAR packets to the Friend Queue.
 *
 * @param[in] src 		Source address for the SAR transaction.
 * @param[in] seqzero 	SeqZero value used in the SAR transaction.
 * @param[in] success 	Whether the SAR transaction was successful.
 */
void friend_sar_complete(uint16_t src, uint32_t seqzero, bool success);

/**
 * Check whether the Friend module has a packet with the specified SeqAuth.
 *
 * @param[in] src       Source address for the SAR transaction.
 * @param[in] seqauth   SeqAuth for the SAR transaction to check.
 *
 * @retval true Friend module has the packet.
 * @retval false Friend module does not have the packet.
 */
bool friend_sar_exists(uint16_t src, uint64_t seqauth);

/**
 * Checks if the given LPN address has an active friendship established.
 *
 * @param[in] src       Source address of the LPN.
 *
 * @retval true   Friendship is established with the given LPN.
 * @retval false  Friendship is not established with the given LPN.
 */
bool friend_friendship_established(uint16_t src);

/**
 * Gets the remaining Poll Timeout time.
 *
 * @param[in] src       Source address of the LPN.
 *
 * @retval PollTimeoutTime Value of the PollTimeout timer in 100 ms.
 * @retval 0               Address is either unknown or the PollTimeout has
 *                         expired for the given address (ref. @tagMeshSp
 *                         section 4.3.2.68). In practice, this means the same.
 */
uint32_t friend_remaining_poll_timeout_time_get(uint16_t src);

#if FRIEND_TEST_HOOK
/**
 * Sets TX delay for any message sent by the friend.
 *
 * @param[in] tx_delay_ms TX delay in ms.
 */
void friend_tx_delay_set(uint16_t tx_delay_ms);
#endif

/** @} */

#endif /* FRIEND_INTERNAL_H__ */
