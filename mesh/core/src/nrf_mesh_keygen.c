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

#include "nrf_mesh_keygen.h"
#include "enc.h"
#include "nrf_error.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "utils.h"

/* See @tagMeshSp section 3.8.6.3.1 */
typedef struct __attribute((packed))
{
    uint8_t p0;
    uint16_t lpn_address;
    uint16_t friend_address;
    uint16_t lpn_counter;
    uint16_t friend_counter;
} friendship_p_t;

uint32_t nrf_mesh_keygen_aid(const uint8_t * p_appkey, uint8_t * p_aid)
{
    if (NULL == p_appkey || NULL == p_aid)
    {
        return NRF_ERROR_NULL;
    }
    /* See @tagMeshSp section 3.8.5.2 */
    enc_k4(p_appkey, p_aid);
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_keygen_network_secmat(const uint8_t * p_netkey, nrf_mesh_network_secmat_t * p_secmat)
{
    if (NULL == p_netkey || NULL == p_secmat)
    {
        return NRF_ERROR_NULL;
    }
    /* See @tagMeshSp section 3.8.5.3.1 */
    const uint8_t p = 0;
    enc_k2(p_netkey, &p, sizeof(p), p_secmat);
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_keygen_friendship_secmat(const uint8_t * p_netkey, const nrf_mesh_keygen_friendship_secmat_params_t * p_params, nrf_mesh_network_secmat_t * p_secmat)
{
    if (NULL == p_netkey || NULL == p_params  || NULL == p_secmat)
    {
        return NRF_ERROR_NULL;
    }

    friendship_p_t p;
    p.p0 = 0x01;
    p.lpn_address = LE2BE16(p_params->lpn_address);
    p.friend_address = LE2BE16(p_params->friend_address);
    p.lpn_counter = LE2BE16(p_params->lpn_counter);
    p.friend_counter = LE2BE16(p_params->friend_counter);
    enc_k2(p_netkey, (const uint8_t *) &p, sizeof(p), p_secmat);
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_keygen_beacon_secmat(const uint8_t * p_netkey, nrf_mesh_beacon_secmat_t * p_secmat)
{
    if (NULL == p_netkey || NULL == p_secmat)
    {
        return NRF_ERROR_NULL;
    }
    /* See @tagMeshSp section 3.8.5.3.2 */
    enc_k3(p_netkey, p_secmat->net_id);

    /* See @tagMeshSp section 3.8.5.3.4 */
    const uint8_t salt_input[4] =  "nkbk";
    const uint8_t key_info[6] = "id128\x01";
    uint8_t salt[NRF_MESH_KEY_SIZE]; /**< Beacon salt used for all beacon key derivations. */
    enc_s1(salt_input, sizeof(salt_input), salt);
    enc_k1(p_netkey, NRF_MESH_KEY_SIZE, salt, key_info,
           sizeof(key_info), p_secmat->key);
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_keygen_identitykey(const uint8_t * p_netkey, uint8_t * p_key)
{
    if (NULL == p_netkey || NULL == p_key)
    {
        return NRF_ERROR_NULL;
    }
    /* See @tagMeshSp section 3.8.5.3.3 */
    const uint8_t salt_input[4] =  "nkik";
    const uint8_t key_info[6] = "id128\x01";
    uint8_t salt[NRF_MESH_KEY_SIZE]; /**< Beacon salt used for all beacon key derivations. */
    enc_s1(salt_input, sizeof(salt_input), salt);
    enc_k1(p_netkey, NRF_MESH_KEY_SIZE, salt, key_info,
           sizeof(key_info), p_key);
    return NRF_SUCCESS;
}

uint32_t nrf_mesh_keygen_virtual_address(const uint8_t * p_virtual_uuid, uint16_t * p_address)
{
    if (NULL == p_virtual_uuid || NULL == p_address)
    {
        return NRF_ERROR_NULL;
    }
    uint8_t tmp[NRF_MESH_KEY_SIZE];
    const uint8_t salt_input[4] =  "vtad";
    enc_s1(salt_input, sizeof(salt_input), &tmp[0]);
    enc_aes_cmac(&tmp[0], p_virtual_uuid, NRF_MESH_KEY_SIZE, &tmp[0]);

    /* Concatenate the upper two bytes to get the 16 bit address and force the two upper bits
     * to '0b10XXXXXX'. See @tagMeshSp section 3.4.2.3 Virtual Address*/
    *p_address = ((((tmp[NRF_MESH_KEY_SIZE - 2] & 0x3F) | 0x80) << 8) | tmp[NRF_MESH_KEY_SIZE - 1]);
    return NRF_SUCCESS;
}
