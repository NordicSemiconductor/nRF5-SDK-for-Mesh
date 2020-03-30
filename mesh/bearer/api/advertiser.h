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
#ifndef ADVERTISER_H__
#define ADVERTISER_H__

#include "broadcast.h"
#include "timer_scheduler.h"
#include "packet_buffer.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_bearer.h"
#include "nrf_mesh.h"
#include "bearer_event.h"

/**
 * @defgroup ADVERTISER Advertiser
 * @ingroup MESH_API_GROUP_BEARER
 * Implements a Bluetooth LE 4.0 compliant advertiser.
 * @{
 */

/** Advertiser time randomization offset per interval, as defined by the Bluetooth Specification. */
#define ADVERTISER_INTERVAL_RANDOMIZATION_US MS_TO_US(10)
/** Define usedd with @ref adv_packet_t to make the packet transmit indefinitely. */
#define ADVERTISER_REPEAT_INFINITE 0xFF

/** Maximum length of an advertisement packet in the buffer. Can be used to create buffers that can
 * fit an exact number of packets. */
#define ADVERTISER_PACKET_BUFFER_PACKET_MAXLEN    (sizeof(packet_buffer_packet_t) + sizeof(adv_packet_t))

/** The structure defining the contents of an advertisement packet.  */
typedef struct
{
    /** TX token, set by the application. May be used to identify the packet in the TX complete callback. */
    nrf_mesh_tx_token_t token;
    struct
    {
        /** Number of times the packet should be transmitted on each channel. */
        uint8_t repeats;
    } config;
    /** Advertisement packet going on air. */
    packet_t packet __attribute__((aligned(WORD_SIZE)));
} adv_packet_t;

/** Advertiser channel configuration. */
typedef struct
{
    /** Array of radio channels to send on. */
    uint8_t channel_map[BEARER_ADV_CHANNELS_MAX];
    /** Number of channels in the @c channel_map. */
    uint8_t count;
    /** If set, the advertiser will randomize the order of the advertisement channels before each transmit. */
    bool    randomize_order;
} advertiser_channels_t;


/** Configuration of the advertiser to be used when sending the packets.*/
typedef struct
{
    /** On init the advertiser will use the address in the device FICR. */
    ble_gap_addr_t        adv_addr;
    /** On init the advertiser will use @c BEARER_ADV_INT_DEFAULT_MS. */
    uint32_t              advertisement_interval_us;
    /** On init the advertiser will use all the default BLE advertisement channels, without randomization. */
    advertiser_channels_t channels;
} advertiser_config_t;

/** Forward declaration of advertiser_t structure. */
typedef struct advertiser_t advertiser_t;

/**
 * Transmit complete callback for notifying the users after a given packet has been sent the
 * desired number of times. Packets that are repeated indefinitely will get this callback on
 * every transmission.
 *
 * @note The timestamp parameter is clocked the last time the packet goes on air (the last channel
 * in the configuration, on the last repeat), at the time when the first bit of the
 * @p p_packet->packet goes on air.
 *
 * @param[in] p_adv The advertiser instance used in sending the packet.
 * @param[in] token TX token, as set by the application.
 * @param[in] timestamp Timestamp of the last transmission of the packet, in microseconds.
 */
typedef void (*advertiser_tx_complete_cb_t)(advertiser_t * p_adv, nrf_mesh_tx_token_t token, timestamp_t timestamp);

typedef struct
{
    nrf_mesh_tx_token_t token; /**< TX token, set by the application. */
    timestamp_t timestamp; /**< Timestamp of the last transmission of the packet, in microseconds. */
} advertiser_tx_complete_params_t;

/** Single advertiser instance. */
struct advertiser_t
{
    bool                            enabled; /**< Flag indicating whether the event is enabled. */
    adv_packet_t *                  p_packet; /**< Pointer to the current packet, only for internal use. */
    broadcast_t                     broadcast; /**< Broadcast module, used as a context to send a single advertisement. */
    timer_event_t                   timer; /**< Timer event used to set up periodic advertisements. */
    packet_buffer_t                 buf; /**< Packet buffer for outgoing packets. */
    advertiser_config_t             config; /**< Advertiser configuration. */
    advertiser_tx_complete_cb_t     tx_complete_callback; /**< TX complete callback to call at the end of a completed transmission. */
    bearer_event_sequential_t       tx_complete_event; /**< Bearer event for executing the TX_COMPLETE event outside the radio interrupt. */
    advertiser_tx_complete_params_t tx_complete_params; /**< Parameters of the TX_COMPLETE event. */
};

/**
 * Initialize the advertiser module.
 */
void advertiser_init(void);

/**
 * Initialize an advertiser instance.
 *
 * @note This function can be called multiple times to initialize different advertiser
 * instances
 *
 * @param[in,out] p_adv The advertiser instance to initialize, this must be a statically
 * allocated object.
 * @param[in] tx_complete_cb The transmit complete callback function pointer, or NULL.
 * @param[in] p_buffer The raw buffer to use when sending packets, this must be a
 * statically allocated buffer that is only dedicated to the given advertiser instance @c p_adv.
 * @param[in] buffer_size The buffer size in bytes.
 */
void advertiser_instance_init(advertiser_t * p_adv,
                              advertiser_tx_complete_cb_t tx_complete_cb,
                              uint8_t * p_buffer,
                              uint32_t buffer_size);

/**
 * Enables the advertiser instance given.
 *
 * @param[in,out] p_adv Advertiser to enable.
 */
void advertiser_enable(advertiser_t * p_adv);

/**
 * Disables the advertiser instance given, so that no more packets are sent from this advertiser
 * even if @ref advertiser_packet_send is called afterwards.
 *
 * @param[in,out] p_adv Advertiser to disable.
 */
void advertiser_disable(advertiser_t * p_adv);

/**
 * Allocates a buffer, if available, from the given advertiser instance.
 *
 * @note The returned packet has all headerfields set with default values. The token is set to 0,
 * and may be altered by the application.
 *
 * @param[in, out] p_adv The advertiser instance to use, it must have been successfully
 *                               initialized via @ref advertiser_init.
 * @param[in] adv_payload_size The advertisement packet payload size.
 *
 * @return         A pointer to the allocated advertisement packet.
 */
adv_packet_t * advertiser_packet_alloc(advertiser_t * p_adv, uint32_t adv_payload_size);

/**
 * Sends a given packet using the given advertiser instance, this can be called multiple times
 * without having to wait for a tx_complete (@see advertiser_tx_complete_cb_t) on the previous
 * packets.
 *
 * @note       Once the packet is sent successfully, the @c tx_complete_callback in the @c p_adv
 *             will be called with @c p_packet and @c p_adv as parameters to the callback.
 *
 * @param[in,out] p_adv     An already initialized advertiser instance.
 * @param[in,out] p_packet  A valid packet that was allocated using @ref advertiser_packet_alloc, and
 *                          the the same advertiser instance as given to this function.
 */
void advertiser_packet_send(advertiser_t * p_adv, adv_packet_t * p_packet);

/**
 * Discards an allocated advertisement packet. The packet memory will be freed, and all contents
 * will be lost.
 *
 * @param[in,out] p_adv Advertiser owning the packet to discard.
 * @param[in,out] p_packet Packet to discard.
 */
void advertiser_packet_discard(advertiser_t * p_adv, adv_packet_t * p_packet);

/**
 * Updates the advertiser configuration.
 *
 * @note           The defaults will be already set on @ref advertiser_init, to see what the
 *                 defaults are see @ref advertiser_config_t.
 *
 * @param[in, out] p_adv     An already initialized advertiser instance.
 * @param[in]      p_config  The new configuration to use with the given advertiser instance.
 */
void advertiser_config_set(advertiser_t * p_adv, const advertiser_config_t * p_config);

/**
 * Gets the current advertiser configuration.
 *
 * @param[in]  p_adv     An already initialized advertiser instance.
 * @param[out] p_config  A configuration instance to populate.
 */
void advertiser_config_get(const advertiser_t * p_adv, advertiser_config_t * p_config);

/**
 * Sets the advertiser channels used by the given advertiser instance.
 *
 * @param[in,out] p_adv Advertiser instance to configure.
 * @param[in] p_channels New channel configuration.
 */
void advertiser_channels_set(advertiser_t * p_adv, const advertiser_channels_t * p_channels);

/**
 * Sets the advertiser address used by the given advertiser instance.
 *
 * @note Only @c BLE_GAP_ADDR_TYPE_PUBLIC and @c BLE_GAP_ADDR_TYPE_RANDOM_STATIC address types
 * are supported. The advertisement address may be altered to ensure Bluetooth Core Specification v4.0
 * compliance.
 *
 * @param[in,out] p_adv Advertiser instance to configure.
 * @param[in] p_addr New GAP advertisement address.
 */
void advertiser_address_set(advertiser_t * p_adv, const ble_gap_addr_t * p_addr);

/**
 * Gets the advertiser address used by the given advertiser instance.
 *
 * @param[in]  p_adv  Advertiser instance pointer.
 * @param[out] p_addr GAP address pointer to write the address.
 */
static inline void advertiser_address_get(const advertiser_t * p_adv, ble_gap_addr_t * p_addr)
{
    memcpy(p_addr, &p_adv->config.adv_addr, sizeof(ble_gap_addr_t));
}

/**
 * Sets the advertisement interval for the given advertiser.
 *
 * @param[in,out] p_adv Advertiser to configure.
 * @param[in] interval_ms Advertisement interval in milliseconds.
 */
void advertiser_interval_set(advertiser_t * p_adv, uint32_t interval_ms);

/**
 * Gets the given advertiser's advertisement interval.
 *
 * @param[in] p_adv Advertiser instance pointer.
 *
 * @returns Advertisement interval in milliseconds.
 */
static inline uint32_t advertiser_interval_get(const advertiser_t * p_adv)
{
    return US_TO_MS(p_adv->config.advertisement_interval_us);
}

/**
 * Sets the TX power for the given advertiser.
 *
 * @param[in,out] p_adv Advertiser to configure.
 * @param[in] tx_power New TX power.
 */
void advertiser_tx_power_set(advertiser_t * p_adv, radio_tx_power_t tx_power);

/**
 * Gets the TX power for the given advertiser.
 *
 * @param[in] p_adv Advertiser instance pointer.
 *
 * @returns the TX power value.
 */
static inline radio_tx_power_t advertiser_tx_power_get(const advertiser_t * p_adv)
{
    return p_adv->broadcast.params.radio_config.tx_power;
}

/**
 * Flushes the given advertiser's packet queue.
 *
 * @warning If called in the middle of a transmission, the ongoing transmission will finish, and
 * produce a TX-complete event, potentially after the call to this function returns.
 *
 * @warning Any packets that have been allocated, but not sent must either be sent or freed before
 * this call.
 *
 * @param[in,out] p_adv Advertiser instance to flush and disable.
 */
void advertiser_flush(advertiser_t * p_adv);

/**
 * Gets the default advertisement address from device factory information structure.
 *
 * @param[in,out] p_addr Address structure to return the address in.
 */
void advertiser_address_default_get(ble_gap_addr_t * p_addr);

/**
 * Checks if an advertiser is enabled.
 *
 * @param[in] p_adv Advertiser instance pointer.
 *
 * @returns @c true if the given advertiser is enabled.
 */
static inline bool advertiser_is_enabled(const advertiser_t * p_adv)
{
    return p_adv->enabled;
}

/** @} */

#endif /* ADVERTISER_H__ */
