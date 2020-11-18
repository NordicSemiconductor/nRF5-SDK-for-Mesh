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

#ifndef NRF_MESH_EXTERNS_H__
#define NRF_MESH_EXTERNS_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh.h"
#include "nrf_mesh_keygen.h"

/**
 * @internal
 * @defgroup NRF_MESH_EXTERNS Extern functions
 * @ingroup NRF_MESH
 * @brief Functions that are needed by the core stack but implemented by upper layers.
 * @{
 */

/**
 * Requests primary network security material for a specific device address.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] owner_addr Unicast address of the device that will get the primary network key.
 * @param[out] pp_secmat Double pointer that will contain the primary network key
 * for the given address, or NULL if not found.
 */
extern void nrf_mesh_primary_net_secmat_get(uint16_t owner_addr, const nrf_mesh_network_secmat_t ** pp_secmat);

/**
 * Requests network security material.
 * This function is expected to iterate, starting from the @c pp_secmat, if it points to a valid
 * security material.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] nid The network identifier used for lookup of network keys.
 * @param[in, out] pp_secmat The network security material, which is used as a starting point if
 * valid, the next match or NULL (if no new match is found) is returned via this parameter.
 * @param[out] pp_secmat_secondary Additional security material. This is used if a key refresh procedure
 * is in progress and a network has an additional network key with the same NID that can be used for
 * decrypting packets.
 */
extern void nrf_mesh_net_secmat_next_get(uint8_t nid, const nrf_mesh_network_secmat_t ** pp_secmat,
            const nrf_mesh_network_secmat_t ** pp_secmat_secondary);

/**
 * Requests application security material.
 * This function is expected to iterate, starting from the @c pp_app_secmat, if
 * it points to a valid security material.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] p_network_secmat The network security material.
 * @param[in] aid The application identifier used for lookup of application keys.
 * @param[in, out] pp_app_secmat The application security material, which is used as a starting
 * point if valid, the next match or NULL (if no new match is found) is returned via this parameter.
 * @param[in, out] pp_app_secmat_secondary Additional security material. This is used if a key refresh procedure
 * is in progress and a node has an additional application key with the same AID that can be used for
 * decrypting packets.
 */
extern void nrf_mesh_app_secmat_next_get(const nrf_mesh_network_secmat_t * p_network_secmat,
        uint8_t aid, const nrf_mesh_application_secmat_t ** pp_app_secmat,
        const nrf_mesh_application_secmat_t ** pp_app_secmat_secondary);

/**
 * Requests device key security material for a specific device address.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] owner_addr Unicast address of the device to get the device key for.
 * @param[out] pp_devkey_secmat Double pointer that will contain the device key
 * for the given address, or NULL if not found.
 */
extern void nrf_mesh_devkey_secmat_get(uint16_t owner_addr, const nrf_mesh_application_secmat_t ** pp_devkey_secmat);

/**
 * Requests beacon info structures.
 * This function is expected to iterate, starting from the @c
 * pp_beacon_info, if it points to a valid beacon info structure.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] p_network_id The network ID to lookup, or NULL if all beacons should be included.
 * @param[in, out] pp_beacon_info The beacon info struct, which is used as a
 * starting point if valid, or NULL if iteration should start from the beginning.
 * The next match or NULL (if no new match is found) is returned via this parameter.
 * @param[out] p_kr_phase Pointer to a variable where the current key refresh phase of the
 * subnet the beacon belongs to is stored.
 */
extern void nrf_mesh_beacon_info_next_get(const uint8_t * p_network_id, const nrf_mesh_beacon_info_t ** pp_beacon_info,
        nrf_mesh_key_refresh_phase_t * p_kr_phase);

/**
 * Checks if a given address is in the RX address list
 * This function is expected to iterate, starting from the @c p_address, if it points to a
 * valid virtual address since multiple virtual addresses may have the same raw_address.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] raw_address The 16-bit address value.
 * @param[in, out] p_address The address struct to be populated with the full address information,
 * if the raw_address is in the RX list.
 *
 * @returns True if the address is in the RX address list, otherwise false.
 */
extern bool nrf_mesh_rx_address_get(uint16_t raw_address, nrf_mesh_address_t * p_address);

/**
 * Get the device's unicast address.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[out] p_addr_start Outputs the device's start address.
 * @param[out] p_addr_count Outputs the number of addresses allocated.
 */
extern void nrf_mesh_unicast_address_get(uint16_t * p_addr_start, uint16_t * p_addr_count);

/**
 * Sets friendship security credentials.
 *
 * @note Only invoked if @ref MESH_FEATURE_LPN_ENABLED is set.
 *
 * @param[in] p_net The network security material.
 * @param[in] p_secmat_params The friendship security credentials
 *
 * @returns NRF_SUCCESS
 * @returns NRF_ERROR_NOT_FOUND if subnetwork for specified network secmat has not been found
 * @returns NRF_ERROR_NO_MEM No more friendship credentials can be added
 */
extern uint32_t nrf_mesh_friendship_secmat_params_set(const nrf_mesh_network_secmat_t * p_net, const nrf_mesh_keygen_friendship_secmat_params_t *p_secmat_params);

/**
 * Requests friendship network security material for a Low Power node address.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] lpn_addr   Unicast address of the Low Power node that will get the friendship network key.
 * @param[out] pp_secmat Double pointer that will contain the friendship network key
 * for the given address, or NULL if not found.
 */
extern void nrf_mesh_friendship_secmat_get(uint16_t lpn_addr, const nrf_mesh_network_secmat_t ** pp_secmat);

#if MESH_FEATURE_FRIEND_ENABLED
/**
 * Retrives the master TX network secmat from the given friendship secmat.
 *
 * @param[in] p_secmat  Pointer to the friendship secmat.
 *
 * @retval  <pointer>   Pointer to the master network secmat corresponding to the given friendship secmat.
 * @retval  NULL        No suitable secmat could be found.
 */
extern nrf_mesh_network_secmat_t * nrf_mesh_net_master_secmat_get(const nrf_mesh_network_secmat_t * p_secmat);
#endif

/**
 * Gets the key refresh phase of the given security material.
 *
 * @note This function is implemented by the Device State Manager module.
 *
 * @param[in] p_secmat Valid secmat pointer. The function will assert on invalid input.
 *
 * @retval NRF_MESH_KEY_REFRESH_PHASE_0  See @ref NRF_MESH_KEY_REFRESH_PHASE_0.
 * @retval NRF_MESH_KEY_REFRESH_PHASE_1  See @ref NRF_MESH_KEY_REFRESH_PHASE_1.
 * @retval NRF_MESH_KEY_REFRESH_PHASE_2  See @ref NRF_MESH_KEY_REFRESH_PHASE_2.
 * @retval NRF_MESH_KEY_REFRESH_PHASE_3  See @ref NRF_MESH_KEY_REFRESH_PHASE_3.
 */
nrf_mesh_key_refresh_phase_t nrf_mesh_key_refresh_phase_get(const nrf_mesh_network_secmat_t * p_secmat);

/**
 * Gets the network security material from the given subnetwork key index.
 *
 * @param[in] subnet_index Subnetwork key index.
 *
 * @retval <pointer> Pointer to network security material.
 * @retval NULL      No security material for the key index was found.
 */
const nrf_mesh_network_secmat_t * nrf_mesh_net_secmat_from_index_get(uint16_t subnet_index);

/**
 * Returns whether the device will process packets received on the given destination address.
 *
 * @param[in] p_addr The raw address to check for.
 *
 * @returns Whether the device will process packets received on the given destination address.
 */
extern bool nrf_mesh_is_address_rx(const nrf_mesh_address_t * p_addr);

/**
 * Checks if the device has been provisioned.
 *
 * @retval true   The device has been provisioned.
 * @retval false  The device has not been provisioned.
 */
extern bool nrf_mesh_is_device_provisioned(void);

/** @} end of NRF_MESH_EXTERNS */

#endif /* NRF_MESH_EXTERNS_H__ */
