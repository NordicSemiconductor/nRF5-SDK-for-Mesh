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

#ifndef NET_BEACON_H__
#define NET_BEACON_H__

#include <stdbool.h>
#include <stdint.h>
#include "nrf_mesh.h"
#include "packet.h"

/**
 * @defgroup NET_BEACON Network beacon
 * @ingroup MESH_CORE
 * Handles incoming and outgoing network beacons.
 * @{
 */

/** Size of the network beacon buffer. */
#define NET_BEACON_BUFFER_SIZE 22

/**
 * Size (in octets) of the CMAC field in a secure network broadcast beacon
 * packet.
 */
#define NET_BEACON_CMAC_SIZE 8

/**
 * Initializes the network beacon module.
 */
void net_beacon_init(void);


/**
 * Enables the net beacon module.
 */
void net_beacon_enable(void);

/**
 * Sets the state of the secure network beacon.
 * @param[in] enabled Set to @c true to enable the secure network beacon or @c false to disable the beacon.
 */
void net_beacon_state_set(bool enabled);

/**
 * Gets the state of the secure network beacon.
 * @return The current state of the secure network beacon, @c true if enabled, @c false otherwise.
 */
bool net_beacon_state_get(void);

/**
 * Build a network beacon packet in the given buffer.
 *
 * @param[in] p_beacon_secmat Security material to authenticate the beacon.
 * @param[in] iv_index IV index.
 * @param[in] iv_update IV update state.
 * @param[in] key_refresh Key refresh flag.
 * @param[in,out] p_buffer Buffer to build the packet in. Must be at least @ref
 * NET_BEACON_BUFFER_SIZE bytes long.
 *
 * @retval NRF_SUCCESS The beacon packet was built successfully with the given parameters.
 * @retval NRF_ERROR_NULL One or more of the parameters are NULL.
 */
uint32_t net_beacon_build(const nrf_mesh_beacon_secmat_t * p_beacon_secmat,
                          uint32_t iv_index,
                          net_state_iv_update_t iv_update,
                          bool key_refresh,
                          uint8_t * p_buffer);

/**
 * Processes incoming network beacon packets.
 *
 * Called for an incoming packet with the secure network broadcast beacon type.
 *
 * @param[in] p_beacon_data Pointer to beacon data to process.
 * @param[in] data_length   Length of the given beacon data.
 * @param[in] p_meta        The metadata tied to the incoming beacon packet.
 */
void net_beacon_packet_in(const uint8_t * p_beacon_data, uint8_t data_length, const nrf_mesh_rx_metadata_t * p_meta);

/**
 * Get the network beacon flag representation of the given key refresh phase.
 *
 * @param[in] phase Key refresh phase to get the key refresh flag for.
 *
 * @returns The key refresh flag corresponding to the given phase.
 */
static inline bool net_beacon_key_refresh_flag(nrf_mesh_key_refresh_phase_t phase)
{
    return (phase == NRF_MESH_KEY_REFRESH_PHASE_2);
}

/** @} */

#endif /* NET_BEACON_H__ */

