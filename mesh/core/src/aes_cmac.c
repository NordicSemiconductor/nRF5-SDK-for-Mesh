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
#include <stddef.h>
#include <string.h>
#include "aes_cmac.h"

#include "aes.h"
#include "utils.h"
#include "nrf_mesh_assert.h"

typedef enum
{
    CMAC_SUBKEY_INDEX_K1 = 1,
    CMAC_SUBKEY_INDEX_K2 = 2,
} cmac_subkey_index_t;

static inline void xor_Rb(uint8_t * p_key)
{
    /* Rb is all zeros except the last byte, which is 0x87. */
    p_key[NRF_MESH_KEY_SIZE - 1] ^= 0x87;
}


/**
 * Generates AES-CMAC Subkey K1 or K2.
 *
 * @param[in,out] p_aes_data An AES data structure with the correct key set.
 * Returns the correct subkey in its ciphertext.
 * @param[in] subkey_index Index of the subkey to generate.
 */
static void aes_cmac_subkey_generate(aes_data_t * p_aes_data, cmac_subkey_index_t subkey_index)
{
    NRF_MESH_ASSERT(subkey_index == CMAC_SUBKEY_INDEX_K1 ||
                    subkey_index == CMAC_SUBKEY_INDEX_K2);
    memset(p_aes_data->cleartext, 0x00, sizeof(p_aes_data->cleartext));

    /* Step 1: L = AES(K, zero) */
    aes_encrypt(p_aes_data);

    /* Calculate K1 or K2 */
    for (cmac_subkey_index_t i = CMAC_SUBKEY_INDEX_K1; i <= subkey_index; ++i)
    {
        /* K_i+1 = (K_i << 1) xor (Rb && msb); */
        uint8_t msb = !!(p_aes_data->ciphertext[0] & 0x80);
        utils_lshift(p_aes_data->ciphertext, p_aes_data->ciphertext, NRF_MESH_KEY_SIZE);
        if (msb)
        {
            xor_Rb(p_aes_data->ciphertext);
        }
    }
}


void aes_cmac(const uint8_t * const p_key, const uint8_t * const p_msg, uint16_t msg_len, uint8_t * const p_out)
{
    uint16_t num_blocks = (msg_len + 15)/16;

    aes_data_t aes_data;
    memcpy(aes_data.key, p_key, NRF_MESH_KEY_SIZE);

    /* Last block */
    uint8_t last[NRF_MESH_KEY_SIZE];
    uint8_t remainder = (msg_len % 16);
    bool flag = (num_blocks > 0 && remainder == 0);
    cmac_subkey_index_t subkey_index = (flag ?
                                        CMAC_SUBKEY_INDEX_K1 :
                                        CMAC_SUBKEY_INDEX_K2);

    /* Generate the subkey we need */
    aes_cmac_subkey_generate(&aes_data, subkey_index);

    if (flag)
    {
        utils_xor(last, &p_msg[(num_blocks-1)*NRF_MESH_KEY_SIZE], aes_data.ciphertext, NRF_MESH_KEY_SIZE);
    }
    else
    {
        utils_pad(last, &p_msg[(num_blocks-1)*NRF_MESH_KEY_SIZE], remainder);
        utils_xor(last, last, aes_data.ciphertext, NRF_MESH_KEY_SIZE);
    }

    /* First X is zero */
    memset(aes_data.ciphertext, 0x00, sizeof(aes_data.ciphertext));

    /* num_blocks may be zero! */
    for (int i = 0; i < num_blocks - 1; ++i)
    {
        /* Y := X XOR M_i     */
        /* X := AES-128(K, Y) */
        utils_xor(aes_data.cleartext, aes_data.ciphertext, &p_msg[i*NRF_MESH_KEY_SIZE], NRF_MESH_KEY_SIZE);
        aes_encrypt(&aes_data);
    }

    utils_xor(aes_data.cleartext, last, aes_data.ciphertext, NRF_MESH_KEY_SIZE);
    aes_encrypt(&aes_data);
    memcpy(p_out, aes_data.ciphertext, NRF_MESH_KEY_SIZE);
}
