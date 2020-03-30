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

#include "enc.h"

#include <stddef.h>
#include <string.h>
#include <nrf_error.h>

#include "rand.h"
#include "aes.h"
#include "aes_cmac.h"
#include "ccm_soft.h"
#include "utils.h"
#include "nrf_mesh_assert.h"

#define ENC_K2_SALT_INPUT { 's', 'm', 'k', '2' }
#define ENC_K2_NID_MASK   0x7F

#define ENC_K3_SALT_INPUT { 's', 'm', 'k', '3' }
#define ENC_K3_KEY_DATA   { 'i', 'd', '6', '4', 0x01 }

#define ENC_K4_SALT_INPUT  { 's', 'm', 'k', '4' }
#define ENC_K4_KEY_DATA    { 'i', 'd', '6', 0x01 }
#define ENC_K4_OUTPUT_MASK 0x3f

/********************/
/* Public functions */
/********************/

void enc_key_generate(uint8_t * p_key)
{
    rand_hw_rng_get(p_key, 16);
}

void enc_aes_encrypt(const uint8_t * p_key, const uint8_t * p_plaintext, uint8_t * p_result)
{
    aes_data_t aes_data;
    memcpy(aes_data.key, p_key, NRF_MESH_KEY_SIZE);
    memcpy(aes_data.cleartext, p_plaintext, NRF_MESH_KEY_SIZE);
    aes_encrypt(&aes_data);
    memcpy(p_result, aes_data.ciphertext, NRF_MESH_KEY_SIZE);
}

void enc_aes_cmac(const uint8_t * p_key, const uint8_t * p_data, uint16_t data_len, uint8_t * p_result)
{
    aes_cmac(p_key, p_data, data_len, p_result);
}

void enc_aes_ccm_encrypt(ccm_soft_data_t * const p_ccm_data)
{
    ccm_soft_encrypt(p_ccm_data);
}

void enc_aes_ccm_decrypt(ccm_soft_data_t * const p_ccm_data, bool * const p_mic_passed)
{
    ccm_soft_decrypt(p_ccm_data, p_mic_passed);
}


/*********************/
/* Utility functions */
/*********************/

void enc_nonce_generate(const network_packet_metadata_t * p_net_metadata,
                        enc_nonce_t type,
                        uint8_t aszmic,
                        uint8_t * p_nonce)
{
    switch (type)
    {
        case ENC_NONCE_NET:
        {
            enc_nonce_net_t * p_net_nonce = (enc_nonce_net_t *) p_nonce;
            p_net_nonce->type     = type;
            p_net_nonce->ttl      = p_net_metadata->ttl;
            p_net_nonce->ctl      = p_net_metadata->control_packet;
            p_net_nonce->seq      = LE2BE24(p_net_metadata->internal.sequence_number);
            p_net_nonce->src      = LE2BE16(p_net_metadata->src);
            p_net_nonce->padding  = 0;
            p_net_nonce->iv_index = LE2BE32(p_net_metadata->internal.iv_index);
            break;
        }
        case ENC_NONCE_APP:
        case ENC_NONCE_DEV:
        {
            enc_nonce_app_t * p_app_nonce = (enc_nonce_app_t *) p_nonce;
            p_app_nonce->type     = type;
            p_app_nonce->padding  = 0;
            p_app_nonce->aszmic   = aszmic;
            p_app_nonce->seq      = LE2BE24(p_net_metadata->internal.sequence_number);
            p_app_nonce->src      = LE2BE16(p_net_metadata->src);
            p_app_nonce->dst      = LE2BE16(p_net_metadata->dst.value);
            p_app_nonce->iv_index = LE2BE32(p_net_metadata->internal.iv_index);
            break;
        }
        case ENC_NONCE_PROXY:
        {
            enc_nonce_proxy_t * p_proxy_nonce = (enc_nonce_proxy_t *) p_nonce;
            p_proxy_nonce->type     = type;
            p_proxy_nonce->pad      = 0;
            p_proxy_nonce->seq      = LE2BE24(p_net_metadata->internal.sequence_number);
            p_proxy_nonce->src      = LE2BE16(p_net_metadata->src);
            p_proxy_nonce->pad2     = 0;
            p_proxy_nonce->iv_index = LE2BE32(p_net_metadata->internal.iv_index);
            break;
        }
        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

void enc_s1(const uint8_t * p_in, uint16_t in_length, uint8_t * p_out)
{
    NRF_MESH_ASSERT(p_in != NULL && p_out != NULL);

    uint8_t key[16] = {0};
    enc_aes_cmac(key, p_in, in_length, p_out);
}

void enc_k1(const uint8_t * p_ikm, const uint8_t ikm_length, const uint8_t * p_salt,
            const uint8_t * p_info, const uint8_t info_length, uint8_t * const p_out)
{
    uint8_t tmp[NRF_MESH_KEY_SIZE];

    NRF_MESH_ASSERT(p_ikm != NULL && p_salt != NULL && p_info != NULL && p_out != NULL);

    enc_aes_cmac(p_salt, p_ikm, ikm_length, tmp);
    enc_aes_cmac(tmp, p_info, info_length, p_out);
}

void enc_k2(const uint8_t * p_netkey, const uint8_t * p_p, uint16_t length_p,
            nrf_mesh_network_secmat_t * p_output)
{
    NRF_MESH_ASSERT(p_netkey != NULL && p_p != NULL && p_output != NULL);
    NRF_MESH_ASSERT(length_p >= ENC_K2_P_VALUE_MINLEN);
    NRF_MESH_ASSERT(length_p <= ENC_K2_P_VALUE_MAXLEN);

    uint8_t tmp[NRF_MESH_KEY_SIZE + ENC_K2_P_VALUE_MAXLEN + 1];
    const uint8_t salt_input[] = ENC_K2_SALT_INPUT;
    enc_s1(salt_input, sizeof(salt_input), tmp);

    uint8_t key[NRF_MESH_KEY_SIZE];
    enc_aes_cmac(tmp, p_netkey, NRF_MESH_KEY_SIZE, key);

    /* T0 = zero length input */
    /* T1 = AES-CMAC(key, T0 || P || 0x01) */
    memcpy(tmp, p_p, length_p);
    tmp[length_p] = 0x01;
    enc_aes_cmac(key, tmp, length_p + 1, tmp);
    p_output->nid = tmp[NRF_MESH_KEY_SIZE - 1] & ENC_K2_NID_MASK;

    /* T2 = AES-CMAC(key, T1 || P || 0x02) */
    memcpy(tmp + NRF_MESH_KEY_SIZE, p_p, length_p);
    tmp[NRF_MESH_KEY_SIZE + length_p] = 0x02;
    enc_aes_cmac(key, tmp, NRF_MESH_KEY_SIZE + length_p + 1, p_output->encryption_key);

    /* T3 = AES-CMAC(key, T2 || P || 0x03) */
    memcpy(tmp, p_output->encryption_key, NRF_MESH_KEY_SIZE);
    tmp[NRF_MESH_KEY_SIZE + length_p] = 0x03;
    enc_aes_cmac(key, tmp, NRF_MESH_KEY_SIZE + length_p + 1, p_output->privacy_key);
}

void enc_k3(const uint8_t * p_in, uint8_t * p_out)
{
    NRF_MESH_ASSERT(p_in != NULL && p_out != NULL);

    uint8_t tmp[NRF_MESH_KEY_SIZE];
    const uint8_t salt_input[] = ENC_K3_SALT_INPUT;
    enc_s1(salt_input, sizeof(salt_input), tmp);

    enc_aes_cmac(tmp, p_in, NRF_MESH_KEY_SIZE, tmp);

    const uint8_t data_array[] = ENC_K3_KEY_DATA;
    enc_aes_cmac(tmp, data_array, sizeof(data_array), tmp);

    /* Only the 8 least significant bytes are returned (mod 2^64): */
    memcpy(p_out, tmp + 8, 8);
}

void enc_k4(const uint8_t * p_in, uint8_t * p_out)
{
    NRF_MESH_ASSERT(p_in != NULL && p_out != NULL);

    uint8_t tmp[NRF_MESH_KEY_SIZE];

    const uint8_t salt_input[] = ENC_K4_SALT_INPUT;
    enc_s1(salt_input, sizeof(salt_input), tmp);

    enc_aes_cmac(tmp, p_in, NRF_MESH_KEY_SIZE, tmp);

    const uint8_t data_array[] = ENC_K4_KEY_DATA;
    enc_aes_cmac(tmp, data_array, sizeof(data_array), tmp);

    *p_out = tmp[NRF_MESH_KEY_SIZE - 1] & ENC_K4_OUTPUT_MASK;
}

