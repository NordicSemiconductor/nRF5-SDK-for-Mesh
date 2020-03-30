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
#ifndef BROADCAST_H__
#define BROADCAST_H__

#include <stdint.h>
#include "radio_config.h"
#include "bearer_handler.h"
#include "packet.h"

/**
 * @defgroup BROADCAST Broadcast module
 * @ingroup MESH_BEARER
 * Handles the radio in a single advertisement event.
 * @{
 */

/** Forward declaration of struct broadcast_params */
typedef struct broadcast_params broadcast_params_t;

/**
 * Broadcast complete callback for reporting to the users after the packet given has been sent.
 * Provides a timestamp of when the message was sent on the last channel in the configuration, clocked at the end of
 * the access address field, i.e. when the radio starts sending the first byte of @c p_packet.
 *
 * @note: As the broadcast instance is still active when this callback is called, a new event cannot be scheduled from
 * the callback.
 *
 * @param[in] p_broadcast The broadcast_params_t instance used in sending the packet.
 * @param[in] timestamp Timestamp of the last transmission in the event, in device time.
 */
typedef void (*broadcast_complete_cb_t) (broadcast_params_t * p_broadcast, timestamp_t timestamp);

/** Broadcast parameters used in providing the details of the packet to be sent and related
 * configuration info.
 */
struct broadcast_params
{
    /** Pointer to the radio packet to transmit. */
    packet_t * p_packet;
    /** The radio will be configured by the broadcast module before sending the packet. */
    radio_config_t radio_config;
    /** The access_address to send the packet on. */
    uint32_t access_address;
    /** The complete calback is called once the given packet is sent on all channels. */
    broadcast_complete_cb_t tx_complete_cb;
    /** @ref p_packet will be sent on all given channels, in order, starting from index 0 */
    const uint8_t * p_channels;
    /** The size of the @ref p_channels array. */
    uint8_t channel_count;
};

typedef struct
{
    ts_timestamp_t prev_tx_complete_app_time_us; /**< Time spent in the TX complete call on the previous run. */
} broadcast_debug_t;

/** A valid instance of the @c broadcast_t must be provided to the broadcast_send function. */
typedef struct
{
    bearer_action_t action; /**< For internal use only */
    broadcast_params_t params; /**< Broadcast params. Should only be modified when inactive. */
    bool active; /**< Flag indicating whether the broadcast module is active. For internal use only. */
#if BROADCAST_DEBUG
    broadcast_debug_t debug; /**< Debug structure, used to pull usage statistics out of the instance. */
#endif
} broadcast_t;


/**
 * Broadcasts the given packet with the given parameters as soon as possible.
 *
 * @warning    The @c p_broadcast instant must be valid and must not be already in a send state,
 *             each instance can send one packet at a time and a new packet can only be sent after
 *             the tx_complete_cb has returned.
 *
 * @param[in,out] p_broadcast Broadcast instance must to send with. This must be statically allocated.
 *
 * @retval NRF_SUCCESS The broadcast was successfully scheduled.
 * @retval NRF_ERROR_BUSY The broadcast instance is already in use.
 */
uint32_t broadcast_send(broadcast_t * p_broadcast);

/** @} */

#endif /* BROADCAST_H__ */
