/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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
#ifndef PROXY_H__
#define PROXY_H__

#include <stdint.h>
#include <stdbool.h>
#include "net_packet.h"

/**
 * @defgroup PROXY GATT Proxy server
 * Implements the GATT Proxy role.
 * @{
 */

/**
 * Initialize the proxy service, adding it to the GATT server.
 */
void proxy_init(void);

/**
 * Notify the proxy module that a subnetwork was added to the device.
 *
 * Triggers the proxy server to send the matching network beacon to all connected devices.
 *
 * @param[in] net_key_index Key index of the added subnetwork.
 * @param[in] p_network_id Network ID of the added subnetwork.
 */
void proxy_subnet_added(uint16_t net_key_index, const uint8_t * p_network_id);

/**
 * Notify the proxy server that a mesh packet was successfully processed in the network layer.
 *
 * @param[in] p_net_metadata Network metadata associated with the processed packet.
 * @param[in] p_rx_meta RX metadata tied to the packet.
 */
void proxy_net_packet_processed(const network_packet_metadata_t * p_net_metadata,
                                const nrf_mesh_rx_metadata_t * p_rx_meta);

/**
 * Start proxy advertisements.
 *
 * Advertises with network ID to allow proxy clients to establish a connection to a mesh device.
 *
 * @retval NRF_SUCCESS The advertisement was started successfully.
 * @retval NRF_ERROR_INVALID_STATE In an invalid state for starting advertisements.
 */
uint32_t proxy_enable(void);

/**
 * Start advertising with node ID.
 *
 * Advertises with node ID to allow proxy clients to establish a connection specifically to this
 * device. The advertisement runs for 60 seconds before ending. If the advertisement times out or
 * results in a connection, the proxy server will resume advertising with the network ID as soon as
 * possible.
 *
 * @note This function implicitly enables the proxy server, which must be disabled with @ref
 * proxy_disable after the node ID advertising is over to prevent it from advertising with the
 * network ID.
 *
 * @param[in] p_beacon_info Pointer to the beacon info to use for authenticating the advertisement,
 * or @p NULL to cycle through all subnetworks.
 * @param[in] kr_phase Current key refresh phase of the key tied to the given beacon info.
 *
 * @retval NRF_SUCCESS The advertisement was started successfully.
 * @retval NRF_ERROR_INVALID_STATE In an invalid state for starting advertisements.
 * @retval NRF_ERROR_BUSY There aren't any connection slots available to start advertising.
 */
uint32_t proxy_node_id_enable(const nrf_mesh_beacon_info_t * p_beacon_info,
                              nrf_mesh_key_refresh_phase_t kr_phase);

/**
 * Check whether the node ID state for the given beacon info is running.
 *
 * @param[in] p_beacon_info Beacon info to check for or @c NULL to get the general node ID state.
 *
 * @returns Whether or not the node ID advertisement is running for the given beacon info, or in
 * general if the @p p_beacon_info is NULL.
 */
bool proxy_node_id_is_enabled(const nrf_mesh_beacon_info_t * p_beacon_info);

/**
 * Disable the node ID advertisements, if they are enabled.
 *
 * The device will go back to advertising with the network ID upon success.
 *
 * @retval NRF_SUCCESS Successfully disabled the node ID state, and resumed advertising with network
 * ID.
 * @retval NRF_ERROR_INVALID_STATE Node ID advertisements are already disabled.
 */
uint32_t proxy_node_id_disable(void);

/**
 * Stop any ongoing advertisements or connections.
 */
void proxy_disable(void);

/**
 * Check whether the proxy server is enabled.
 *
 * @returns Whether the proxy server is enabled.
 */
bool proxy_is_enabled(void);

/**
 * Check whether the proxy server is connected to any clients.
 *
 * @returns Whether the proxy server is connected to any clients.
 */
bool proxy_is_connected(void);

/** @} */

#endif /* PROXY_H__ */
