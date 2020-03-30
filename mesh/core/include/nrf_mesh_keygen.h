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

#ifndef NRF_MESH_KEYGEN__
#define NRF_MESH_KEYGEN__

#include <stdint.h>
#include "nrf_mesh.h"

/**
 * @defgroup MESH_KEYGEN Key Generation
 * @ingroup MESH_CORE
 * @brief Generation of the keys and value needed for mesh operation as specified by
 * @tagMeshSp.
 * @{
 */

/** Parameters for friendship security material derivation*/
typedef struct
{
    /** LPN address */
    uint16_t lpn_address;
    /** Friend address */
    uint16_t friend_address;
    /** LPN counter value */
    uint16_t lpn_counter;
    /** Friend counter value */
    uint16_t friend_counter;
} nrf_mesh_keygen_friendship_secmat_params_t;

/**
 * AID generation from application key. As specified by @tagMeshSp section 3.8.6.2.
 *
 * @param[in] p_appkey Pointer to the application key of size @ref NRF_MESH_KEY_SIZE.
 * @param[out] p_aid Pointer to an @c uint8_t value to store the resulting AID.
 *
 * @retval NRF_SUCCESS The result of AID from the given key is successfuly stored.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 */
uint32_t nrf_mesh_keygen_aid(const uint8_t * p_appkey, uint8_t * p_aid);

/**
 * Generation of network security material:  NID, encryption key, and privacy key from the given
 * network key. As specified by @tagMeshSp section 3.8.6.3.
 *
 * @param[in] p_netkey Pointer to the network key of size @ref NRF_MESH_KEY_SIZE.
 * @param[out] p_secmat Pointer to a @c nrf_mesh_network_secmat_t struct, in order to store the
 * resulting NID and keys.
 *
 * @retval NRF_SUCCESS The resulting security material from the given key is successfuly stored.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 */
uint32_t nrf_mesh_keygen_network_secmat(const uint8_t * p_netkey, nrf_mesh_network_secmat_t * p_secmat);

/**
 * Generation of friendship network security material:  NID, encryption key, and privacy key from the given
 * network key and friendship parameters. As specified by @tagMeshSp section 3.8.6.3.
 *
 * @param[in] p_netkey Pointer to the network key of size @ref NRF_MESH_KEY_SIZE.
 * @param[in] p_params Pointer to friendship parameter structure
 * @param[out] p_secmat Pointer to a @c nrf_mesh_network_secmat_t struct, in order to store the
 * resulting NID and keys.
 *
 * @retval NRF_SUCCESS The resulting security material from the given key is successfuly stored.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 */
uint32_t nrf_mesh_keygen_friendship_secmat(const uint8_t * p_netkey, const nrf_mesh_keygen_friendship_secmat_params_t * p_params,
                                           nrf_mesh_network_secmat_t * p_secmat);

/**
 * Generation of network beacon security material:  beacon key and net_id from the given network key.
 * As specified by @tagMeshSp section 3.8.6.3.2 and 3.8.6.3.4.
 *
 * @param[in] p_netkey Pointer to the network key of size @ref NRF_MESH_KEY_SIZE.
 * @param[out] p_secmat Pointer to a @c nrf_mesh_beacon_secmat_t struct, in order to store the
 * resulting keys.
 *
 * @retval NRF_SUCCESS The resulting security material from the given key is successfuly stored.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 */
uint32_t nrf_mesh_keygen_beacon_secmat(const uint8_t * p_netkey, nrf_mesh_beacon_secmat_t * p_secmat);

/**
 * Generation of identity key.
 * As specified by @tagMeshSp section 3.8.6.3.3.
 *
 * @param[in] p_netkey Pointer to the network key of size @ref NRF_MESH_KEY_SIZE.
 * @param[out] p_key Pointer to a size @ref NRF_MESH_KEY_SIZE array for storing the resulting
 * identity key.
 *
 * @retval NRF_SUCCESS The resulting identity key is successfuly stored.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 */
uint32_t nrf_mesh_keygen_identitykey(const uint8_t * p_netkey, uint8_t * p_key);

/**
 * Generation of the 16-bit virtual address from the virtual address UUID.
 * As specified by @tagMeshSp section 3.4.2.3.
 *
 * @param[in] p_virtual_uuid Pointer to the virtual address UUID key of size @ref NRF_MESH_KEY_SIZE.
 * @param[out] p_address Pointer to an @c uint16_t value for storing the resulting 16-bit address.
 *
 * @retval NRF_SUCCESS The resulting address is successfuly stored.
 * @retval NRF_ERROR_NULL An unexpected NULL pointer is given.
 */
uint32_t nrf_mesh_keygen_virtual_address(const uint8_t * p_virtual_uuid, uint16_t * p_address);

/** @} end of MESH_KEYGEN */

#endif  /* NRF_MESH_KEYGEN__ */
