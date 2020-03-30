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
#ifndef ADV_EXT_TX_H__
#define ADV_EXT_TX_H__

/**
 * @internal
 * @defgroup ADV_EXT_TX Advertising Extensions TX implementation
 * @ingroup INSTABURST_TX
 * The radio handler module for the Advertising Extensions part of @ref INSTABURST_TX.
 *
 * @note The implementation of this moduel does not conform with the ADV Extensions
 * as described in the Bluetooth specifications.
 * @{
 */

#include <stdint.h>
#include <stdbool.h>
#include "adv_ext_packet.h"
#include "bearer_handler.h"

/** Maximum number of auxiliary packets to transmit in a chain. */
#define ADV_EXT_TX_CHAIN_MAX_COUNT 2

/** Maximum time the user is allowed to spend in the TX callback function. */
#define ADV_EXT_TX_USER_CALLBACK_MAXTIME 100

/** Longest extended header. Static asserts rather than being calculated, to avoid exposing complexity. */
#define ADV_EXT_TX_ADV_EXT_HEADER_OVERHEAD_MAX 7

/** Longest auxiliary packet payload. */
#define ADV_EXT_TX_PAYLOAD_MAXLEN (ADV_EXT_PACKET_LEN_MAX - ADV_EXT_TX_ADV_EXT_HEADER_OVERHEAD_MAX)

/** Maximum space to allocate for the packet header of an AUX packet. */
#define AUX_BUFFER_OVERHEAD_MAX (sizeof(ble_packet_hdr_t) + ADV_EXT_TX_ADV_EXT_HEADER_OVERHEAD_MAX)

typedef struct
{
    nrf_mesh_instaburst_event_id_t id;
    uint8_t packet_count;
    uint8_t channel;
    uint8_t token_count;
} adv_ext_tx_event_params_t;

/** A single advertising event, containing multiple packets. */
typedef struct
{
    adv_ext_tx_event_params_t params;
    uint8_t packet_data[];
} adv_ext_tx_event_t;

typedef struct adv_ext_tx adv_ext_tx_t;

/**
 * Callback type for the TX complete callback.
 *
 * @note The timestamp parameter is clocked the last packet in the chain, at the time when the first
 * bit of the packet header goes on air.
 *
 * @param[in,out] p_tx TX instance that finished sending.
 * @param[in] p_tx_event TX event that was sent.
 * @param[in] timestamp Timestamp of the last packet in the chain in the transmitted Extended
 * advertisement event.
 */
typedef void (*adv_ext_tx_callback_t)(adv_ext_tx_t * p_tx, const adv_ext_tx_event_t * p_tx_event, timestamp_t timestamp);

/** Extended advertising instance configuration parameters. */
typedef struct
{
    radio_config_t radio_config; /**< Radio configuration to use for the auxiliary packets. */
    adv_ext_tx_callback_t callback; /**< Function to call when the transmission is complete. */
} adv_ext_tx_config_t;

/** Extended advertising packet buffer. */
typedef struct
{
    uint8_t data_len; /**< Length of the data, not including the header. */
    uint8_t header[AUX_BUFFER_OVERHEAD_MAX]; /**< Staging buffer for the header. Used to construct packet header before sending. */
    uint8_t data[]; /**< User data. */
} adv_ext_tx_packet_t;

/** Extended advertising instance. */
struct adv_ext_tx
{
    bearer_action_t bearer_action;
    adv_ext_tx_config_t config;
    const adv_ext_tx_event_t * p_tx_event; /**< Active TX event. */
};


/**
 * Gets the next TX packet after the given one.
 *
 * @warning Does not do buffer boundary checking.
 *
 * @param[in] p_packet Previous packet.
 *
 * @returns A pointer to the first packet after the given @p p_packet.
 */
static inline const adv_ext_tx_packet_t * adv_ext_tx_packet_next_get(const adv_ext_tx_packet_t * p_packet)
{
    return (const adv_ext_tx_packet_t *) &p_packet->data[p_packet->data_len];
}

/**
 * Initializes the Advertising Extensions TX module.
 *
 * @param[in] lfclk_ppm Accuracy of the low frequency clock in Parts Per Million (PPM).
 */
void adv_ext_tx_init(uint32_t lfclk_ppm);

/**
 * Initializes a single Advertising Extensions TX instance.
 *
 * @param[in,out] p_tx Instance to initialize.
 * @param[in] p_config Configuration to use for the instance.
 */
void adv_ext_tx_instance_init(adv_ext_tx_t * p_tx, const adv_ext_tx_config_t * p_config);

/**
 * Transmits a single Extended advertising event.
 *
 * @param[in,out] p_tx Instance to transmit on.
 * @param[in] p_tx_event Event to transmit.
 *
 * @retval NRF_SUCCESS The event was successfully scheduled for transmission.
 * @retval NRF_ERROR_BUSY The given instance is already in the process of transmitting an event.
 */
uint32_t adv_ext_tx(adv_ext_tx_t * p_tx, const adv_ext_tx_event_t * p_tx_event);

/** @} */

#endif /* ADV_EXT_TX_H__ */
