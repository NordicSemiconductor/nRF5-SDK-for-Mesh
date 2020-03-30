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
#ifndef INSTABURST_TX_H__
#define INSTABURST_TX_H__

/**
 * @defgroup INSTABURST_TX Instaburst TX module
 * @ingroup INSTABURST
 *
 * The Instaburst TX module acts as a replacement for the regular advertiser,
 * providing higher throughput at the expense of breaking spec-compliance.
 *
 * The Instaburst TX module allocates user data in a generic buffer, then dynamically makes a
 * decision on how to transmit the user data on every advertising event. If the buffer payload can
 * fit inside a regular advertising packet when the advertising event starts,
 * the module transmits the buffer as a regular advertising packet. Otherwise, it will transmit an
 * Advertising Extension indication packet pointing to an Auxiliary advertising packet in a
 * secondary advertising channel. This Auxiliary packet will contain the actual packet data, and may
 * point to yet another Auxiliary packet if it can't fit all the user data into a single packet.
 *
 * To distinguish Auxiliary packets, Instaburst makes a CRC16 hash of the buffer contents, and uses
 * that as the Advertisement Data Identification. This way, a scanner may choose to ignore the
 * Auxiliary packet if it has seen its contents before.
 *
 * @warning Instaburst is a Nordic-specific feature that does not adhere to the Bluetooth Mesh
 * specification. It does not have the same requirements for test coverage, API stability
 * or specification compliance as the rest of Nordic's nRF5 SDK for Mesh.
 *
 * @{
 */

#include "instaburst.h"
#include "broadcast.h"
#include "adv_ext_tx.h"
#include "bearer_event.h"
#include "packet_buffer.h"
#include "bearer_handler.h"

/**
 * Smallest packet buffer size allowed.
 */
#define INSTABURST_TX_BUFFER_MIN_SIZE                                                              \
    (sizeof(packet_buffer_packet_t) + sizeof(adv_ext_tx_event_t) +                                 \
     (sizeof(adv_ext_tx_packet_t) + ADV_EXT_PACKET_LEN_MAX) * ADV_EXT_TX_CHAIN_MAX_COUNT)


typedef struct instaburst_tx instaburst_tx_t;

/**
 * TX Complete callback to be called when all repeats of a buffer have been sent.
 *
 * @param[in,out] p_tx Instaburst instance the buffer was allocated to.
 * @param[in] tx_token Token passed to the alloc call for this buffer.
 * @param[in] timestamp Timestamp of the first bit of the packet header of the last packet in the
 * chain (in device time).
 */
typedef void (*instaburst_tx_complete_t)(struct instaburst_tx * p_tx, nrf_mesh_tx_token_t tx_token, timestamp_t timestamp);

/** Instaburst TX configuration. Passed to the @ref instaburst_tx_instance_init function to
 * configure an Instaburst TX instance. */
typedef struct
{
    /** Set ID for this Instaburst instance. Cannot be higher than @ref INSTABURST_SET_ID_MAX. */
    uint8_t set_id;
    /** Array of radio channels to send on. No channel can be higher than @ref INSTABURST_CHANNEL_INDEX_MAX. */
    const uint8_t * p_channels;
    /** Number of channels in the @c p_channels array. */
    uint8_t channel_count;
    /** Radio mode to run auxiliary packets in. */
    radio_mode_t radio_mode;
    /** TX power to transmit auxiliary packets at. */
    radio_tx_power_t tx_power;
    /** Callback to call when the transmission was completed. */
    instaburst_tx_complete_t callback;
    /**
     * Transmission interval in milliseconds. Must be between @ref BEARER_ADV_INT_MIN_MS and @ref
     * BEARER_ADV_INT_MAX_MS.
     *
     * @note: As per the Bluetooth Core Specification, a 10ms randomization is added to the
     * time between transmissions.
     */
    uint32_t interval_ms;
} instaburst_tx_config_t;

#ifdef INSTABURST_TX_DEBUG
/** Structure containing debug data for an Instaburst TX instance. */
typedef struct
{
    uint32_t failed_allocs;
    uint32_t tx_regular_packet;
    uint32_t tx_adv_ext;
    uint32_t tx_skipped;
} instaburst_tx_debug_t;
#endif

/**
 * Instaburst instance structure. All fields are considered internal and should not be accessed
 * directly.
 */
struct instaburst_tx
{
    instaburst_tx_config_t config;

    uint8_t channel_index;

    packet_buffer_t packet_buffer;

    uint8_t * p_next_alloc; /**< Pointer to the next location to allocate a buffer in. */
    adv_ext_tx_packet_t * p_alloc_packet; /**< Packet being built in the allocate stage. */
    packet_buffer_packet_t * p_alloc_buf; /**< Buffer currently being used for allocating new packets. */
    packet_buffer_packet_t * p_tx_buf; /**< Buffer currently in transmission. */

    broadcast_t broadcast;
    adv_ext_tx_t adv_ext_tx;

    bearer_event_sequential_t tx_complete_event;
    timestamp_t prev_tx_timestamp;

    timer_event_t timer_event;

#ifdef INSTABURST_TX_DEBUG
    instaburst_tx_debug_t debug;
#endif
};

/**
 * Initializes the Instaburst TX module.
 *
 * @param[in] lfclk_ppm The accuracy of the low frequency clock in Parts Per Million (PPM).
 */
void instaburst_tx_init(uint32_t lfclk_ppm);

/**
 * Initializes an Instaburst TX instance.
 *
 * @param[in,out] p_instaburst Instance to initialize.
 * @param[in] p_config Configuration to use for this instance.
 * @param[in,out] p_packet_buffer Packet buffer for this instance.
 * @param[in] packet_buffer_size Size of the given packet buffer. Must be at least @ref INSTABURST_TX_BUFFER_MIN_SIZE bytes.
 */
void instaburst_tx_instance_init(instaburst_tx_t * p_instaburst,
                                 const instaburst_tx_config_t * p_config,
                                 uint8_t * p_packet_buffer,
                                 uint32_t packet_buffer_size);

/**
 * Enables the given Instaburst instance.
 *
 * @param[in,out] p_instaburst Instance to enable.
 */
void instaburst_tx_enable(instaburst_tx_t * p_instaburst);

/**
 * Disables the given Instaburst instance.
 *
 * @param[in,out] p_instaburst Instance to disable.
 */
void instaburst_tx_disable(instaburst_tx_t * p_instaburst);

/**
 * Checks if the Instaburst instance is enabled.
 *
 * @param[in] p_instaburst Instaburst instance pointer.
 *
 * @returns @c true if the Instaburst instance is enabled (scheduled), @c false otherwise.
 */
bool instaburst_tx_is_enabled(const instaburst_tx_t * p_instaburst);

/**
 * Allocates a buffer for transmission.
 *
 * @warning Only one buffer can be allocated at a time. To allocate a second buffer, free or discard
 * the previous.
 *
 * @param[in,out] p_instaburst Instaburst instance to allocate from.
 * @param[in] data_len Desired buffer length.
 * @param[in] tx_token Token to pass to the buffer to identify it on the TX complete event.
 *
 * @returns A pointer to a newly allocated TX buffer, or NULL if the allocation wasn't successful.
 */
uint8_t * instaburst_tx_buffer_alloc(instaburst_tx_t * p_instaburst, uint32_t data_len, nrf_mesh_tx_token_t tx_token);

/**
 * Commits the given buffer for transmission.
 *
 * @param[in,out] p_instaburst Instaburst instance the buffer was allocated from.
 * @param[in] p_buffer Buffer to commit. Must have been acquired from a call to @ref instaburst_tx_buffer_alloc.
 */
void instaburst_tx_buffer_commit(instaburst_tx_t * p_instaburst, const uint8_t * p_buffer);

/**
 * Discards the given buffer, freeing any memory associated with it.
 *
 * @param[in,out] p_instaburst Instaburst instance the buffer was allocated from.
 * @param[in] p_buffer Buffer to discard. Must have been acquired from a call to @ref instaburst_tx_buffer_alloc.
 */
void instaburst_tx_buffer_discard(instaburst_tx_t * p_instaburst, const uint8_t * p_buffer);

/**
 * Finalizes the TX event under construction, putting it up for transmission.
 *
 * This function is automatically called at every advertising event, and the user does not have to
 * call it for the module to operate normally.
 *
 * @note If the buffer lock is active, this function does nothing. See @ref
 * instaburst_tx_buffer_lock.
 *
 * @param[in,out] p_instaburst Instance to finalize a packet of.
 *
 * @returns Whether the current packet was successfully finalized.
 */
bool instaburst_tx_finalize(instaburst_tx_t * p_instaburst);

/**
 * Locks the current buffer to prevent it from being automatically transmitted on the next
 * advertisement.
 *
 * Use this function to ensure that all buffers committed while the lock is active end up
 * in the same event.
 *
 * @note This function holds a counter, allowing multiple locks to occur. Each lock has to
 * correspond to an unlock.
 *
 * @param[in] lock Flag indicating whether the buffer should be locked.
 */
void instaburst_tx_buffer_lock(bool lock);

/**
 * Sets the TX interval for the given Instaburst instance.
 *
 * @param[in,out] p_instaburst Instaburst instance to configure.
 * @param[in] interval_ms New transmission interval in milliseconds. Must be between @ref
 * BEARER_ADV_INT_MIN_MS and @ref BEARER_ADV_INT_MAX_MS.
 */
void instaburst_tx_interval_set(instaburst_tx_t * p_instaburst, uint32_t interval_ms);

/**
 * Gets the TX interval for the given Instaburst instance.
 *
 * @param[in] p_instaburst Instaburst instance pointer.
 *
 * @returns the current TX interval in milliseconds.
 */
static inline uint32_t instaburst_tx_interval_get(const instaburst_tx_t * p_instaburst)
{
    return p_instaburst->config.interval_ms;
}

/**
 * Sets the TX power for the given Instaburst instance.
 *
 * @param[in,out] p_instaburst Instaburst instance to configure.
 * @param[in] tx_power New TX power.
 */
void instaburst_tx_tx_power_set(instaburst_tx_t * p_instaburst, radio_tx_power_t tx_power);

/**
 * Gets the TX power for the given Instaburst instance.
 *
 * @param[in] p_instaburst Instaburst instance pointer.
 *
 * @returns the current TX power.
 */
static inline radio_tx_power_t instaburst_tx_tx_power_get(const instaburst_tx_t * p_instaburst)
{
    return p_instaburst->config.tx_power;
}

/** @} */

#endif /* INSTABURST_TX_H__ */
