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

#ifndef MESH_LPN_INTERNAL_H__
#define MESH_LPN_INTERNAL_H__

#include <stdint.h>
#include "transport.h"
#include "net_packet.h"

/**
 * @internal
 * @defgroup MESH_LPN_INTERNAL Mesh LPN internal API
 *
 * This API is intended for the @ref DEVICE_STATE_MANAGER to use whenever new addresses are added to
 * the global subscription list of the node or when the node enters a new friendship.
 * @{
 */

/**
 * @defgroup MESH_LPN_INTERNAL_SUBMAN Subscription Manager API
 * @{
 */

/**
 * Initializes the LPN subscription manager (LSM) module.
 */
void mesh_lpn_subman_init(void);

/**
 * Adds the given address to the subscription list of the Friend node.
 *
 * This function accepts addresses only after friendship is established.
 *
 * @param[in] address            A 16-bit raw mesh address.
 *
 * @retval  NRF_SUCCESS                 Address has been accepted for propagation to the Friend's
 *                                      subscription list.
 * @retval  NRF_ERROR_NO_MEM            The address value cannot be accepted due to resource constraints.
 * @retval  NRF_ERROR_INVALID_PARAM     The given address is not a group or virtual address.
 * @retval  NRF_ERROR_INVALID_STATE     The friendship is not yet established. Cannot accept address.
 */
uint32_t mesh_lpn_subman_add(uint16_t address);

/**
 * Removes the given address from the subscription list of the Friend node.
 *
 * This function accepts addresses only after friendship is established.
 *
 * @param[in] address           A 16-bit raw mesh address.
 *
 * @retval  NRF_SUCCESS                 Address has been accepted for deletion from the Friend's
 *                                      subscription list.
 * @retval  NRF_ERROR_INVALID_DATA      The address value for removal cannot be accepted, as it was never added.
 * @retval  NRF_ERROR_INVALID_PARAM     The given address is not a group or virtual address.
 * @retval  NRF_ERROR_INVALID_STATE     The friendship is not yet established. Cannot accept address.
 */
uint32_t mesh_lpn_subman_remove(uint16_t address);

/**
 * @} end of MESH_LPN_INTERNAL_SUBMAN
 */

/**
 * @defgroup MESH_LPN_INTERNAL_FSM FSM module API
 * @{
 */

/**
 * Send the friendship message using LPN Finite State Machine (FSM).
 *
 * If this API is called before the previous message has been finished or cancelled, the previous
 * message is over-written.
 *
 * @note The buffer holding the transport control PDU must be kept allocated until the response to this
 * message has been received or the friendship has been terminated.
 *
 * @note The PDU size must fit in the unsegmented message. Otherwise this API
 * will assert.
 *
 * @param[in] p_trs_ctrl_pkt    Pointer to the transport_control_packet_t structure with the following
 *                              members populated: @ref transport_control_packet_t::opcode,
 *                              @ref transport_control_packet_t::p_data, and
 *                              @ref transport_control_packet_t::data_len
 *
 */
void mesh_lpn_subman_data_push(transport_control_packet_t * p_trs_ctrl_pkt);

/**
 * Stops retrying the currently queued message, if any.
 */
void mesh_lpn_subman_data_clear(void);

/**
 * Gets the size of the Friend subscription list for the selected Friend.
 *
 * @returns Size of the Friend's subscription list for the accepted offer.
 */
uint16_t mesh_lpn_friend_subscription_size_get(void);

/**
 * @} end of MESH_LPN_INTERNAL_FSM
 */

/**
 * Notify the LPN that a Network protocol data unit (PDU) was successfully received.
 *
 * By notifying the LPN about a received Network PDU, the LPN can cancel any ongoing scanning
 * activity and save power.
 */
void mesh_lpn_rx_notify(const network_packet_metadata_t * p_net_metadata);

/** @} end of MESH_LPN_INTERNAL */

#endif /* MESH_LPN_INTERNAL_H__ */
