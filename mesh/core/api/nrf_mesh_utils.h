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

#ifndef NRF_MESH_UTILS_H__
#define NRF_MESH_UTILS_H__

#include <stdint.h>
#include <stddef.h>
#include "nrf_mesh_defines.h"
#include "nrf_mesh.h"

/**
 * @defgroup NRF_MESH_UTILS Utility functions
 * @ingroup NRF_MESH
 * Utility functions for mesh users.
 * @{
 */

#define BLE_GAP_ADDR_TYPE_RANDOM_INVALID 0x04 /**< Invalid Random address type. */

/**
 * Obtains a random number from the random number generator.
 *
 * @param[out] p_output Pointer to a buffer where the output is written.
 * @param[in]  size     Size of the output buffer.
 *
 * @retval NRF_SUCCESS    The specified buffer was filled with the requested amount of random data.
 * @retval NRF_ERROR_NULL The parameter p_output is a NULL pointer.
 */
uint32_t nrf_mesh_rand_get(uint8_t * p_output, uint8_t size);

/**
 * Get the address type of little endian 16-bit mesh addresses.
 *
 * @param[in] address Little endian 16bit mesh address to get the type of.
 *
 * @return Address type of the given address.
 */
nrf_mesh_address_type_t nrf_mesh_address_type_get(uint16_t address);

/**
 * Get the GAP address type of a given device address
 *
 * @param[in] p_address Pointer to the 6 byte address array.
 * @param[in] txadd_bit The value of the TxAdd bit in the advertisement packet pdu header.
 *
 * @return One of see BLE_GAP_ADDR_TYPES in ble_gap.h, if the given address has a valid type,
 * otherwise @ref BLE_GAP_ADDR_TYPE_RANDOM_INVALID.
 */
uint8_t nrf_mesh_gap_address_type_get(const uint8_t * p_address, uint8_t txadd_bit);

/**
 * Get the beacon secmat representing the given key refresh phase from the beacon info.
 *
 * @param[in] p_beacon_info Beacon info the get secmat from.
 * @param[in] kr_phase Current key refresh phase.
 *
 * @returns A pointer to the secmat used during the given key refresh phase.
 */
static inline const nrf_mesh_beacon_secmat_t * nrf_mesh_beacon_secmat_from_info(const nrf_mesh_beacon_info_t * p_beacon_info, nrf_mesh_key_refresh_phase_t kr_phase)
{
    if (p_beacon_info)
    {
        return ((kr_phase == NRF_MESH_KEY_REFRESH_PHASE_0) ? &p_beacon_info->secmat
                                                        : &p_beacon_info->secmat_updated);
    }
    else
    {
        return NULL;
    }
}

/** @} */

#endif /* NRF_MESH_UTILS_H__*/
