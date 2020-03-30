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
#ifndef MESH_TRANSPORT_H__
#define MESH_TRANSPORT_H__

#include <stdint.h>
#include <stdbool.h>

#include "packet.h"
#include "timer.h"
#include "utils.h"

#include "nrf_mesh.h"
#include "nrf_mesh_opt.h"
#include "nrf_mesh_defines.h"
#include "network.h"
#include "packet_mesh.h"
#include "utils.h"
#include "nrf_mesh_assert.h"

#include "nrf_mesh_config_core.h"

/**
 * @defgroup TRANSPORT Transport Layer
 * @ingroup MESH_CORE
 * @{
 */

/** Default RX timeout. */
#define TRANSPORT_SAR_RX_TIMEOUT_DEFAULT_US SEC_TO_US(10)

/** Default base RX acknowledgement timeout. */
#define TRANSPORT_SAR_RX_ACK_BASE_TIMEOUT_DEFAULT_US MS_TO_US(150)

/** Default per hop RX acknowledgement timeout addition. */
#define TRANSPORT_SAR_RX_ACK_PER_HOP_ADDITION_DEFAULT_US MS_TO_US(50)

/** Default base TX retry timeout. */
#define TRANSPORT_SAR_TX_RETRY_BASE_TIMEOUT_DEFAULT_US MS_TO_US(500)

/** Default per hop TX retry timeout addition. */
#define TRANSPORT_SAR_TX_RETRY_PER_HOP_ADDITION_DEFAULT_US MS_TO_US(50)

/** Default number of retries before cancelling SAR TX session. */
#define TRANSPORT_SAR_TX_RETRIES_DEFAULT (4)

/** Maximum number of control packet consumers. Heartbeat + LPN_SM + LPN FSM + Friend. */
#if MESH_FEATURE_LPN_ENABLED && MESH_FEATURE_FRIEND_ENABLED
#define TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX   (4)
#elif MESH_FEATURE_LPN_ENABLED
#define TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX   (3)
#elif MESH_FEATURE_FRIEND_ENABLED
#define TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX   (2)
#else
#define TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX   (1)
#endif
/** The highest control packet opcode that can fit in the packet field,  */
#define TRANSPORT_CONTROL_PACKET_OPCODE_MAX (PACKET_MESH_TRS_CONTROL_OPCODE_MASK)

/** Control packet opcodes */
typedef enum
{
    /** Sent by a SAR packet receiver to a sender to report the status of the SAR packet receive process. */
    TRANSPORT_CONTROL_OPCODE_SEGACK,
    /** Sent by a Low Power node to its Friend node to request any messages that it has stored for the Low Power node. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_POLL,
    /** Sent by a Friend node to a Low Power node to inform it about security updates. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE,
    /** Sent by a Low Power node the all-friends fixed group address to start to find a friend. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_REQUEST,
    /** Sent by a Friend node to a Low Power node to offer to become its friend. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_OFFER,
    /** Sent to a Friend node to inform a previous friend of a Low Power node about the removal of a friendship. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR,
    /** Sent from a previous friend to Friend node to confirm that a prior friend relationship has been removed. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR_CONFIRM,
    /** Sent to a Friend node to add one or more addresses to the Friend Subscription List. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_ADD,
    /** Sent to a Friend node to remove one or more addresses from the Friend Subscription List. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_REMOVE,
    /** Sent by a Friend node to confirm Friend Subscription List updates. */
    TRANSPORT_CONTROL_OPCODE_FRIEND_SUBSCRIPTION_LIST_CONFIRM,
    /** Sent by a node to let other nodes determine topology of a subnet. */
    TRANSPORT_CONTROL_OPCODE_HEARTBEAT
} transport_control_opcode_t;

/** Control packet structure. */
typedef struct
{
    transport_control_opcode_t opcode; /**< Opcode of the control packet. */
    const packet_mesh_trs_control_packet_t * p_data; /**< Control packet data. */
    uint32_t data_len; /**< Length of the control packet data. */
    bool reliable; /**< Whether or not the packet is an acknowledged message. */
    uint16_t src; /**< Source address (must be a unicast address). */
    nrf_mesh_address_t dst; /**< Packet destination address. */
    const nrf_mesh_network_secmat_t * p_net_secmat; /**< Network security material used during network encryption/decryption. */
    uint8_t ttl; /**< TTL value for the control packet. This is a 7 bit value. */
    core_tx_bearer_selector_t bearer_selector; /**< The bearers on which the outgoing packets are to be sent on. Alternatively, use CORE_TX_BEARER_TYPE_ALLOW_ALL to allow allocation to all bearers. */
} transport_control_packet_t;

/**
 * Control packet handler callback function.
 *
 * @param[in] p_control_packet Control packet received.
 * @param[in] p_rx_metadata Metadata for the packet that contained the control packet (the last packet in the case of multi-segment packets).
 */
typedef void (*transport_control_packet_callback_t)(
    const transport_control_packet_t * p_control_packet,
    const nrf_mesh_rx_metadata_t * p_rx_metadata);

/* Control packet handler. */
typedef struct
{
    transport_control_opcode_t opcode; /**< Opcode to handle. */
    transport_control_packet_callback_t callback; /**< Callback function to call when a control packet with the given opcode is received. */
} transport_control_packet_handler_t;

/**
 * Initializes the transport layer.
 */
void transport_init(void);

/**
 * Enables the transport layer.
 */
void transport_enable(void);

/**
 * Function for passing packets from the network layer to the transport layer.
 *
 * @param[in] p_packet Pointer to the transport packet.
 * @param[in] trs_packet_len Length of the transport packet.
 * @param[in] p_net_metadata Pre-filled network metadata structure. Note that the transport layer
 * expects the destination address to have its value field set, and will fill in the rest of the
 * address data itself.
 * @param[in] p_rx_metadata RX metadata tied to the packet.
 *
 * @retval NRF_ERROR_INVALID_ADDR The destination address is not valid.
 * @retval NRF_ERROR_NOT_FOUND    The packet could not be decryptet with any application key.
 * @retval NRF_ERROR_NULL         One or more of the input parameters was NULL.
 * @retval NRF_SUCCESS            The packet was successfully decrypted and sent up the stack.
 */
uint32_t transport_packet_in(const packet_mesh_trs_packet_t * p_packet,
                             uint32_t trs_packet_len,
                             const network_packet_metadata_t * p_net_metadata,
                             const nrf_mesh_rx_metadata_t * p_rx_metadata);

/**
 * Convert a transport address context structure to a 16-bit short address.
 *
 * @param[in]  p_addr          Transport address context pointer.
 * @param[out] p_short_addr    16-bit little-endian short address.
 *
 * @retval NRF_SUCCESS            Successfully converted address.
 * @retval NRF_ERROR_INVALID_ADDR Invalid address.
 */
uint32_t transport_addr_to_short(nrf_mesh_address_t * p_addr, uint16_t * p_short_addr);

/**
 * Transmit a transport message.
 *
 * The transport layer processes a message from the application, encrypts it and
 * passes it on to the network layer.
 *
 * @param[in]  p_params           Message parameters.
 * @param[out] p_packet_reference Reference to SAR buffer (for TX complete event).
 *
 * @retval NRF_SUCCESS              The packet was successfully queued for transmission.
 * @retval NRF_ERROR_NULL           Null-pointer supplied.
 * @retval NRF_ERROR_INVALID_ADDR   Invalid address supplied.
 * @retval NRF_ERROR_INVALID_LENGTH The packet length was too long.
 * @retval NRF_ERROR_INVALID_PARAM  One or more of the given parameters are out of bounds.
 * @retval NRF_ERROR_NO_MEM         Insufficient amount of available memory.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t transport_tx(const nrf_mesh_tx_params_t * p_params, uint32_t * const p_packet_reference);

/**
 * Transmit a transport control message.
 *
 * The transport control message is only encrypted at the network level, and should only be used for
 * spec-defined core-control messages. All application-level messages should go through the @ref
 * transport_tx function.
 *
 * @note The destination address must be a unicast or group address.
 *
 * @param[in] p_params Transport control packet parameters.
 * @param[in] tx_token Token to use in the TX complete event.
 *
 * @retval NRF_SUCCESS              The packet was successfully queued for transmission.
 * @retval NRF_ERROR_NULL           Null-pointer supplied.
 * @retval NRF_ERROR_INVALID_ADDR   Invalid address supplied.
 * @retval NRF_ERROR_INVALID_LENGTH The packet length was too long.
 * @retval NRF_ERROR_INVALID_PARAM  One or more of the given parameters are out of bounds.
 * @retval NRF_ERROR_NO_MEM         Insufficient amount of available memory.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is
 *                                  being to sent to this destination. Wait for
 *                                  the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t transport_control_tx(const transport_control_packet_t * p_params,
                              nrf_mesh_tx_token_t tx_token);

/**
 * Set transport layer options.
 *
 * @param[in] id    Identifier for option to set.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS         Successfully set option.
 * @retval NRF_ERROR_NOT_FOUND Could not find the ID requested.
 * @retval NRF_ERROR_NULL      NULL pointer supplied.
 */
uint32_t transport_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * const p_opt);

/**
 * Get transport layer options.
 *
 * @param[in] id    Identifier for option to set.
 * @param[in] p_opt Pointer to option struct.
 *
 * @retval NRF_SUCCESS         Successfully set option.
 * @retval NRF_ERROR_NOT_FOUND Could not find the ID requested.
 * @retval NRF_ERROR_NULL      NULL pointer supplied.
 */
uint32_t transport_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * const p_opt);

/**
 * Add a control packet consumer to the list of consumers.
 *
 * @param[in] p_handlers An array of control packet handlers.
 * @param[in] handler_count The number of handlers in @p p_handlers.
 *
 * @retval NRF_SUCCESS The consumer was successfully added to the list of consumers, and all
 * incoming control packets with an opcode matching one of the handlers will result in a call to
 * the corresponding callback.
 * @retval NRF_ERROR_NO_MEM The maximum number of consumers is already reached. Increase @ref
 * TRANSPORT_CONTROL_PACKET_CONSUMERS_MAX to avoid this.
 * @retval NRF_ERROR_NULL One or more of the callbacks in the handler array is NULL.
 * @retval NRF_ERROR_FORBIDDEN One or more of the handlers have opcodes covered by existing
 * consumers.
 * @retval NRF_ERROR_INVALID_DATA One or more of the handlers have opcodes higher than @ref
 * TRANSPORT_CONTROL_PACKET_OPCODE_MAX.
 */
uint32_t transport_control_packet_consumer_add(const transport_control_packet_handler_t * p_handlers, uint32_t handler_count);

/** @} */

#endif
