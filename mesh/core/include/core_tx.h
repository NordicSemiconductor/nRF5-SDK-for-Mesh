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
#ifndef CORE_TX_H__
#define CORE_TX_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh.h"
#include "net_packet.h"
#include "list.h"

/** Maximum number of concurrent bearers. Limited by the number of bits in the
 * core_tx_bearer_bitmap_t type. */
#define CORE_TX_BEARER_COUNT_MAX 64

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

#if MESH_FEATURE_RELAY_ENABLED
    CORE_TX_ROLE_RELAY, /**< The packet came from a different device and is being relayed. */
#endif

    CORE_TX_ROLE_COUNT /**< Number of roles available, not a valid role itself. */
} core_tx_role_t;

/** Supported bearer types. */
typedef enum
{
    CORE_TX_BEARER_TYPE_INVALID     = 0x00,
    CORE_TX_BEARER_TYPE_ADV         = 0x01,
    CORE_TX_BEARER_TYPE_GATT_SERVER = 0x02,
    CORE_TX_BEARER_TYPE_GATT_CLIENT = 0x04,
    CORE_TX_BEARER_TYPE_FRIEND      = 0x08,
    CORE_TX_BEARER_TYPE_LOW_POWER   = 0x10,
    CORE_TX_BEARER_TYPE_LOCAL       = 0x20,
    CORE_TX_BEARER_TYPE_ALLOW_ALL   = 0xFF,
} core_tx_bearer_type_t;

/** Bearer selector type representing multiple bearer types. */
typedef uint32_t core_tx_bearer_selector_t;

/** Parameter structure for @ref core_tx_packet_alloc(). */
typedef struct
{
    core_tx_role_t role; /**< Packet role. */
    uint32_t net_packet_len; /**< Requited network packet length. */
    const network_packet_metadata_t * p_metadata; /**< Network packet metadata to pass to bearer for allocation decision. */
    nrf_mesh_tx_token_t token; /**< TX token used to identify the packet in the TX complete callback. */
    core_tx_bearer_selector_t bearer_selector; /**< The bearers on which the outgoing packets are to be sent on. Alternatively, use @ref CORE_TX_BEARER_TYPE_ALLOW_ALL to allow allocation to all bearers. */
} core_tx_alloc_params_t;

/**
 * Bitmap type representing multiple bearers.
 *
 * @note The bearer bitmap represents bearer instances, not bearer types. Each bearer instance has a bearer type.
 */
typedef uint64_t core_tx_bearer_bitmap_t;

/**
 * @defgroup CORE_TX_BEARER_API Core TX bearer API
 * API for the Core TX bearers to interact with the Core TX module.
 * @{
 */

/** Allocation result returned by the bearers to indicate result of allocation. */
typedef enum
{
    /** The allocation was successful. */
    CORE_TX_ALLOC_SUCCESS,
    /** The bearer rejected the packet based on its metadata. */
    CORE_TX_ALLOC_FAIL_REJECTED,
    /** The bearer didn't have enough memory to fit the packet. */
    CORE_TX_ALLOC_FAIL_NO_MEM
} core_tx_alloc_result_t;

typedef struct core_tx_bearer core_tx_bearer_t;

/**
 * Packet allocation function type for Core TX bearer interfaces.
 *
 * The bearer is expected to allocate space for a packet with the given parameters or present a
 * reason for rejecting it. The bearer may later be expected to commit the packet without failure.
 * Only one packet will be allocated at a time.
 *
 * @param[in] p_bearer Bearer to allocate on.
 * @param[in] p_params Allocation parameters to aid the bearer in allocating.
 *
 * @returns The result of the allocation.
 */
typedef core_tx_alloc_result_t (*core_tx_bearer_packet_alloc_t)(core_tx_bearer_t * p_bearer,
                                                                const core_tx_alloc_params_t * p_params);
/**
 * Packet send function type for Core TX bearer interfaces.
 *
 * The bearer is expected to copy the payload of the packet, and commit it for sending.
 *
 * @param[in] p_bearer Bearer to send on.
 * @param[in] p_packet Pointer to the packet to send. Shall be copied into the allocated bearer buffer.
 * @param[in] packet_length Length of the packet.
 */
typedef void (*core_tx_bearer_packet_send_t)(core_tx_bearer_t * p_bearer,
                                             const uint8_t * p_packet,
                                             uint32_t packet_length);

/**
 * Packet discard function type for Core TX bearer interfaces.
 *
 * The bearer is expected to discard the currently allocated packet.
 *
 * @param[in] p_bearer Bearer to discard the packet on.
 */
typedef void (*core_tx_bearer_packet_discard_t)(core_tx_bearer_t * p_bearer);

/**
 * Core TX interface definition, providing packet sending functionality for a single Core TX bearer type.
 */
typedef struct
{
    core_tx_bearer_packet_alloc_t packet_alloc;
    core_tx_bearer_packet_send_t packet_send;
    core_tx_bearer_packet_discard_t packet_discard;
} core_tx_bearer_interface_t;

struct core_tx_bearer
{
#ifdef CORE_TX_DEBUG
    struct
    {
        uint32_t alloc_count;
        uint32_t no_mem_count;
        uint32_t reject_count;
        uint32_t discard_count;
        uint32_t send_count;
    } debug;
#endif

    const core_tx_bearer_interface_t * p_interface;

    uint8_t bearer_index;
    core_tx_bearer_type_t type;

    list_node_t list_node;
};

/** @} */

/**
 * Transmission complete callback type used to notify the user of ended transmissions.
 *
 * The @p bearer_index parameter maps to an offset in the bearer bitmap. Combined with the bitmap
 * returned in the corresponding @ref core_tx_packet_alloc call, it can be used to determine whether
 * all bearers are finished sending this packet.
 *
 * @param[in] role Packet role, as set in the corresponding alloc call.
 * @param[in] bearer_index Index of the bearer reporting the TX complete event. Is always the offset
 * of one of the marked bits returned in the corresponding alloc call.
 * @param[in] timestamp Timestamp of the packet transmission in microseconds.
 * @param[in] token Token set in the packet alloc call.
 */
typedef void (*core_tx_complete_cb_t)(core_tx_role_t role,
                                      uint32_t bearer_index,
                                      uint32_t timestamp,
                                      nrf_mesh_tx_token_t token);

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
 * The packet allocation will succeed if one or more bearers accept the allocation. Every set bit
 * in the return value will yield a corresponding TX complete callback once that specific bearer
 * has completed sending. Every successful call to this function has to be followed by a send or
 * discard call before the next alloc call can be issued.
 *
 * @warning Will assert if a previous alloc call hasn't had a matching send or discard call.
 *
 * @param[in] p_params Packet allocation parameters.
 * @param[out] pp_packet Packet pointer to populate.
 *
 * @returns A bitmap of the bearers that were able to allocate data.
 */
core_tx_bearer_bitmap_t core_tx_packet_alloc(const core_tx_alloc_params_t * p_params, uint8_t ** pp_packet);

/**
 * Send the currently allocated packet.
 */
void core_tx_packet_send(void);

/**
 * Discard the currently allocated packet.
 */
void core_tx_packet_discard(void);

/**
 * Get the bearer type at the given index.
 *
 * @param[in] bearer_index Bearer index to look up.
 *
 * @returns The type of bearer the given bearer index represents.
 */
core_tx_bearer_type_t core_tx_bearer_type_get(uint32_t bearer_index);

/**
 * Get the number of registered bearers.
 *
 * @returns The number of bearers registered.
 */
uint32_t core_tx_bearer_count_get(void);

/**
 * @ingroup CORE_TX_BEARER_API
 * @{
 */

/**
 * Add a bearer.
 *
 * Once added, all fields in the bearer structure are read-only.
 *
 * @param[in,out] p_bearer Bearer to register for.
 * @param[in] p_interface Interface structure. Must be static memory, with no NULL-pointer
 * functions.
 * @param[in] type The type of bearer this is.
 */
void core_tx_bearer_add(core_tx_bearer_t * p_bearer,
                        const core_tx_bearer_interface_t * p_interface,
                        core_tx_bearer_type_t type);

/**
 * TX complete callback for bearers to call upon successful sending.
 *
 * @note Must be called in order of allocation.
 *
 * @param[in,out] p_bearer Bearer to register a TX complete for.
 * @param[in] role TX role the packet was transmitted in.
 * @param[in] timestamp Timestamp of the packet transmission in microseconds.
 * @param[in] token Token set in the packet alloc call.
 */
void core_tx_complete(core_tx_bearer_t * p_bearer,
                      core_tx_role_t role,
                      uint32_t timestamp,
                      nrf_mesh_tx_token_t token);
/** @} */

#ifdef UNIT_TEST

/**
 * @internal
 * Reset the core TX state. Unsafe outside of unit testing.
 */
void core_tx_reset(void);
#endif
/** @} */

#endif /* CORE_TX_H__ */
