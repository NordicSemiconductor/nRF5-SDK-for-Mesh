/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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
#ifndef CORE_TX_H__
#define CORE_TX_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh.h"

/**
 * @defgroup CORE_TX Core Transmission Handler
 * @ingroup MESH_CORE
 * Handler for transmission of core mesh packets.
 * @{
 */

/** Available roles each packet can be transmitted for. */
typedef enum
{
    CORE_TX_ROLE_ORIGINATOR, /**< The packet originated in this device. */
    CORE_TX_ROLE_RELAY, /**< The packet came from a different device and is being relayed. */

    CORE_TX_ROLE_COUNT /**< Number of roles available, not a valid role itself. */
} core_tx_role_t;

typedef enum
{
    CORE_TX_BEARER_ADV = (1 << 0),
} core_tx_bearer_t;

typedef struct
{
    core_tx_bearer_t bearer;
    core_tx_role_t role;
} core_tx_metadata_t;

/**
 * @defgroup CORE_TX_BEARER_API Core TX bearer API
 * API for the Core TX bearers to interact with the Core TX module.
 * @{
 */

/**
 * Packet allocation function type for Core TX bearer interfaces.
 * @param[in] net_packet_len Length of the network packet
 * @param[in] p_metadata Core TX metadata for the packet
 * @param[in] token TX token that shall be presented in the TX complete callback for the packet.
 *
 * @returns A pointer to a network packet buffer ready to be populated or NULL if the allocation
 * failed.
 */
typedef uint8_t * (*core_tx_bearer_packet_alloc_t)(uint32_t net_packet_len,
                                                   const core_tx_metadata_t * p_metadata,
                                                   nrf_mesh_tx_token_t token);
/**
 * Packet send function type for Core TX bearer interfaces.
 *
 * @param[in] p_metadata Core TX metadata for the packet, as passed into the alloc function
 * @param[in,out] p_packet Pointer to the packet to send. Comes from the alloc callback in the same interface.
 */
typedef void (*core_tx_bearer_packet_send_t)(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);

/**
 * Packet discard function type for Core TX bearer interfaces.
 *
 * @param[in] p_metadata Core TX metadata for the packet, as passed into the alloc function.
 * @param[in,out] p_packet Pointer to the packet to discard. Comes from the alloc callback in the same interface.
 */
typedef void (*core_tx_bearer_packet_discard_t)(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);

/**
 * Core TX interface definition, providing packet sending functionality for a single Core TX bearer type.
 */
typedef struct
{
    core_tx_bearer_packet_alloc_t packet_alloc;
    core_tx_bearer_packet_send_t packet_send;
    core_tx_bearer_packet_discard_t packet_discard;
} core_tx_bearer_interface_t;

/** @} */

/**
 * Transmission complete callback type used to notify the user of ended transmissions.
 *
 * @warning The transmission complete callback executes in an interrupt, and should do as little as
 * possible before returning.
 *
 * @param[in] p_metadata Metadata for the transmission.
 * @param[in] timestamp Timestamp of the packet transmission in microseconds.
 * @param[in] token Token set in the packet alloc call.
 */
typedef void (*core_tx_complete_cb_t)(const core_tx_metadata_t * p_metadata, uint32_t timestamp, nrf_mesh_tx_token_t token);

/**
 * Set the core tx complete callback.
 *
 * @param[in] tx_complete_callback Callback to call at the end of a successful transmission, or NULL
 * if no callback is needed.
 */
void core_tx_complete_cb_set(core_tx_complete_cb_t tx_complete_callback);

/**
 * Allocate a network packet for transmission.
 *
 * @param[in] net_packet_len Desired network packet length, including header and MIC.
 * @param[in] p_metadata Metadata for the packet.
 * @param[in,out] pp_packet Packet pointer to populate.
 * @param[in] token User token that can be used to identify the packet in the TX complete callback.
 *
 * @returns A bitmask of the bearers that were able to allocate data.
 */
core_tx_bearer_t core_tx_packet_alloc(uint32_t net_packet_len,
                                      const core_tx_metadata_t * p_metadata,
                                      uint8_t ** pp_packet,
                                      nrf_mesh_tx_token_t token);

/**
 * Send a network packet acquired through the @ref core_tx_packet_alloc() function.
 *
 * @param[in] p_metadata Metadata used when allocating the packet.
 * @param[in,out] p_packet Packet pointer to send.
 */
void core_tx_packet_send(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);

/**
 * Discard a packet buffer acquired through the @ref core_tx_packet_alloc() function. The packet
 * memory will be made available for reallocation, and its contents discarded.
 *
 * @param[in] p_metadata Metadata used when allocating the packet.
 * @param[in,out] p_packet Packet pointer to send.
 */
void core_tx_packet_discard(const core_tx_metadata_t * p_metadata, uint8_t * p_packet);

/**
 * @ingroup CORE_TX_BEARER_API
 * @{
 */

/**
 * Register an interface for the specified bearer.
 *
 * Registering multiple handlers for the same bearer will result in an assert.
 *
 * @param[in] bearer Bearer to register for.
 * @param[in] p_interface Interface structure. Must be static memory, with no NULL-pointer functions.
 */
void core_tx_bearer_register(core_tx_bearer_t bearer, const core_tx_bearer_interface_t * p_interface);

/**
 * TX complete callback for bearers to call upon successful sending.
 *
 * @param[in] p_metadata Metadata for the sent packet, as passed to the bearer's alloc interface
 * for the given packet.
 * @param[in] timestamp Timestamp of the packet transmission in microseconds.
 * @param[in] token Token set in the packet alloc call.
 */
void core_tx_complete(const core_tx_metadata_t * p_metadata,
                      uint32_t timestamp,
                      nrf_mesh_tx_token_t token);
/** @} */

/** @} */

#endif /* CORE_TX_H__ */
