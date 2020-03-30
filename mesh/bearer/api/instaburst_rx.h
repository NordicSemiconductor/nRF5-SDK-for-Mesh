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
#ifndef INSTABURST_RX_H__
#define INSTABURST_RX_H__

/**
 * @defgroup INSTABURST_RX Instaburst RX module
 * @ingroup INSTABURST
 *
 * The Instaburst RX module implements a subset of the required scanner functionality for the
 * Bluetooth 5.0 feature "Advertising Extensions". It only aims to support messages coming
 * from a Nordic Mesh device sending TX packets with the @ref INSTABURST_TX implementation.
 *
 * The Instaburst RX module hooks into the scanner module through its inline callback functionality,
 * providing a sneak peak at incoming packets. If the module detects an Advertising Extension
 * Indication packet that points to an Auxiliary Advertising packet with a configuration within the
 * bounds of the modules capabilites, it will attempt to schedule an auxiliary scanning event to
 * receive the Auxiliary advertising packet. Should the Auxiliary packet point to another packet,
 * the module will attempt to keep scheduling new RX events until it runs out of resources or
 * reaches the end of the packet chain.
 *
 * @warning Instaburst is a Nordic-specific feature that does not adhere to the Bluetooth Mesh
 * specification. It does not have the same requirements for test coverage, API stability
 * or specification compliance as the rest of Nordic's nRF5 SDK for Mesh.
 *
 * @{
 */

#include "instaburst.h"
#include "adv_ext_packet.h"

#ifndef INSTABURST_RX_DEBUG
/** Whether to enable the Instaburst RX debug mode. */
#define INSTABURST_RX_DEBUG 0
#endif

/** A single Instaburst RX packet. */
typedef struct
{
    nrf_mesh_rx_metadata_instaburst_t metadata; /**< Metadata associated with the given packet. */
    uint8_t payload_len; /**< Length of the packet payload */
    const uint8_t * p_payload; /**< A pointer to the packet payload. */
} instaburst_rx_packet_t;

/** Stats structure for Instaburst. */
typedef struct
{
    uint32_t rx_ok; /**< Number of successfully received packets. */
    uint32_t crc_fail; /**< Number of CRC failures detected. */
    uint32_t too_late; /**< Number of times the module failed to start RX on time. */
    uint32_t no_rx; /**< Number of times the module started RX on time, but failed detecting any incoming packets. */
    uint32_t invalid_offset; /**< Number of packets dropped due to unsupported offset times. */
    uint32_t busy; /**< Number of packets dropped because the scanner was busy. */
    uint32_t switched_timeslot; /**< Number of packets dropped because the timeslot ended before we could handle it. */
} instaburst_rx_stats_t;

/**
 * Initializes the Instaburst RX module.
 *
 * @param[in] packet_process_cb Function to call upon successfully receiving an Instaburst packet.
 */
void instaburst_rx_init(bearer_event_flag_callback_t packet_process_cb);

/**
 * Enables the Instaburst RX module by registering its RX callback with the scanner.
 */
void instaburst_rx_enable(void);

/**
 * Disables the Instaburst RX module.
 */
void instaburst_rx_disable(void);

/**
 * Fetches a single packet from the Instaburst packet queue.
 *
 * @returns A pointer to a received Instaburst packet, or NULL if no packet is ready.
 */
const instaburst_rx_packet_t * instaburst_rx(void);

/**
 * Checks whether Instaburst has any packets ready for being fetched by @ref instaburst_rx.
 *
 * @returns Whether the Instaburst RX module has packets ready for reading.
 */
bool instaburst_rx_pending(void);

/**
 * Releases a packet acquired through the @ref instaburst_rx function.
 *
 * @param[in] p_packet Packet to release.
 */
void instaburst_rx_packet_release(const instaburst_rx_packet_t * p_packet);

/**
 * Gets a pointer to the Instaburst RX stats structure. If @ref INSTABURST_RX_DEBUG is set, each
 * BLE channel will have a stats struct allocated for it, keeping track of the number of events that
 * occurred on that channel.
 *
 * @param[in] channel Channel to fetch the structure from. Must be lower than or equal to @ref
 * INSTABURST_CHANNEL_INDEX_MAX.
 *
 * @returns A pointer to the stats structure for the given channel, or NULL if no structure is
 * found.
 */
const instaburst_rx_stats_t * instaburst_rx_stats_get(uint8_t channel);

/** @} */

#endif /* INSTABURST_RX_H__ */
