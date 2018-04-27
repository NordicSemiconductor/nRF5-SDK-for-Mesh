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

/**
 * Initializes the network beacon module.
 */
void net_beacon_init(void);

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
 * Processes incoming network beacon packets.
 *
 * Called for an incoming packet with the secure network broadcast beacon type.
 *
 * @param[in] p_beacon_data Pointer to beacon data to process.
 * @param[in] data_length   Length of the given beacon data.
 * @param[in] p_meta        The metadata tied to the incoming beacon packet.
 */
void net_beacon_packet_in(const uint8_t * p_beacon_data, uint8_t data_length, const nrf_mesh_rx_metadata_t * p_meta);




/** @} */

#endif /* NET_BEACON_H__ */

