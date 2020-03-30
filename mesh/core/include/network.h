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
#ifndef MESH_NETWORK_H__
#define MESH_NETWORK_H__

#include "nrf_mesh.h"
#include "nrf_mesh_opt.h"
#include "core_tx.h"
#include "net_packet.h"

typedef struct
{
    /** Data set by the user prior to allocation. */
    struct
    {
        /** Network metadata, used to fill header fields. */
        network_packet_metadata_t * p_metadata;
        /** TX Token to give to the bearer. */
        nrf_mesh_tx_token_t token;
        /** Length of the payload. */
        uint32_t payload_len;
        /** The bearers on which the outgoing packets are to be sent on. Alternatively, use CORE_TX_BEARER_TYPE_ALLOW_ALL to allow allocation to all bearers. */
        core_tx_bearer_selector_t bearer_selector;
        /** Role this device has for the packet. */
        core_tx_role_t role;
    } user_data;


    /** Pointer to the network data, set in the allocation. */
    uint8_t * p_payload;
} network_tx_packet_buffer_t;

/**
 * @defgroup NETWORK Network Layer
 * @ingroup MESH_CORE
 * Processes incoming messages and decrypts packet contents using a network key.
 * @{
 */

/**
 * Initializes the network layer.
 * @param[in] p_init_params Pointer to the initialization parameters structure.
 */
void network_init(const nrf_mesh_init_params_t * p_init_params);

/**
 * Enables the network module.
 */
void network_enable(void);

/**
 * Sets a network layer option.
 * @param[in] id    Option ID.
 * @param[in] p_opt Pointer to a structure containing the new value of the option.
 * @retval NRF_SUCCESS          The value of the specified option was successfully changed.
 * @retval NRF_ERROR_NOT_FOUND  The specified option ID is not valid for this module.
 * @retval NRF_ERROR_INVALID_PARAM The value provided for the option was invalid.
 * @retval NRF_ERROR_NULL       The @c p_opt parameter was NULL.
 */
uint32_t network_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * p_opt);

/**
 * Retrieves the value of a network layer option.
 * @param[in]  id    Option ID.
 * @param[out] p_opt Pointer to where the retrieved option value will be stored.
 * @retval NRF_SUCCESS             The value of the specified option was successfully retrieved.
 * @retval NRF_ERROR_NOT_FOUND     The specified option ID is not valid for this module.
 * @retval NRF_ERROR_INVALID_PARAM The value provided for the option was invalid.
 * @retval NRF_ERROR_NULL          The @c p_opt parameter was NULL.
 */
uint32_t network_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * p_opt);

/**
 * Allocates a network packet, and populates its header fields based on the given metadata before
 * returning a pointer to the payload. Will retain internal fields in the network metadata if the
 * core tx role is @ref CORE_TX_ROLE_RELAY.
 *
 * @param[in,out] p_buffer Network packet buffer to populate. All user data must be valid.
 *
 * @retval NRF_SUCCESS The packet was allocated successfully.
 * @retval NRF_ERROR_NO_MEM There wasn't enough buffer space to allocate the packet.
 * @retval NRF_ERROR_FORBIDDEN Couldn't allocate a sequence number for the packet.
 */
uint32_t network_packet_alloc(network_tx_packet_buffer_t * p_buffer);

/**
 * Sends a packet allocated through @ref network_packet_alloc. The network layer is responsible for
 * releasing the memory after use.
 *
 * @param[in] p_buffer Network packet buffer to send.
 */
void network_packet_send(const network_tx_packet_buffer_t * p_buffer);

/**
 * Discards a packet allocated through @ref network_packet_alloc.
 *
 * @param[in] p_buffer Network packet buffer to discard.
 */
void network_packet_discard(const network_tx_packet_buffer_t * p_buffer);

/**
 * Function for processing incoming packets. Will attempt to decrypt the packet before passing it to
 * transport, along with extracted metadata.
 *
 * @note Does not perform destination address lookup, but instead fills the raw value.
 *
 * @param[in] p_packet Network packet to process.
 * @param[in] net_packet_len Length of the network packet.
 * @param[in] p_rx_metadata RX metadata for the packet the network packet came in.
 *
 * @retval NRF_SUCCESS The packet was successfully processed.
 * @retval NRF_ERROR_INVALID_ADDR The destination address is not valid.
 * @retval NRF_ERROR_NOT_FOUND    The packet could not be decrypted.
 */
uint32_t network_packet_in(const uint8_t * p_packet, uint32_t net_packet_len, const nrf_mesh_rx_metadata_t * p_rx_metadata);

/** @} */

#endif

