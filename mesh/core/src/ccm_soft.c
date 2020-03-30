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
#include <string.h>
#include <nrf_error.h>

#include "utils.h"
#include "log.h"
#include "aes.h"
#include "toolchain.h"

#include "ccm_soft.h"
#include "nrf_mesh_assert.h"

/* How it all works:
 * Generation of the MIC and encryption are two separate procedures.
 *
 * Everything in AES-CCM happens in 16 byte blocks.
 *
 * Encryption of the payload is done by making a cipher stream S and xoring this with the payload:
 * enc_data = (S[1..N] xor data), where N = (data_len + 15) / 16 + 1. Note that we're skipping S[0].
 * S is made up of 16 byte blocks, where S[i] = AES(A[i]) and
 *
 *        | 1 byte    | 16 - L_LEN bytes | L_LEN bytes |
 * A[i] = | L_LEN - 1 | Nonce            | i           |
 *
 * To generate the MIC, we make a chain of 16 byte blocks X[i], i=0..M, where
 *
 * M = (a_len + 15) / 16 + (data_len + 15) / 16 + 1
 *
 * X[0] = AES(B[0])
 * X[i] = enc(X[i-1] xor B[i]).
 *
 * B is made up of some metadata, the additional data and the data:
 *
 *           | 16 bytes | 2 bytes | a_len bytes | 16-(a_len % 16) | data_len | 16-(data_len % 16) |
 * B[0..M] = | Metadata | a_len   | a_data ...  | 000000000000000 | data     | 000000000000000000 |
 *
 * The metadata (B[0]) is made up of 1 byte of flags, the (16 - L_LEN - 1) byte nonce and mic_len.
 *
 * Finally, the MIC = (X[M] xor S[0]), truncated to fit the desired MIC-length. NOTE: X[M] is named
 * T in the spec and code.
 *
 * To decrypt, we first calculate data = (S[1..N] xor enc_data), then insert this clear text data
 * into B, calculate the MIC, and compare it.
 */

/* All multibyte numbers are in big endian. Nonces, keys and data are represented as byte streams,
 * and are not encoded. */
#define L_LEN CCM_LENGTH_FIELD_LENGTH
#define CCM_BLOCK_SIZE CCM_KEY_LENGTH

typedef struct
{
    uint8_t len_field_len;
    uint8_t nonce[CCM_NONCE_LENGTH];
    uint16_t counter;
} a_block_t;

typedef struct
{
    uint8_t flags;
    uint8_t nonce[CCM_NONCE_LENGTH];
    uint16_t length_field;
} b0_t;

NRF_MESH_STATIC_ASSERT(sizeof(a_block_t) == CCM_BLOCK_SIZE);
NRF_MESH_STATIC_ASSERT(sizeof(b0_t) == CCM_BLOCK_SIZE);

static void ccm_soft_authenticate_blocks(aes_data_t * p_aes_data,
                                         const uint8_t * p_data,
                                         uint16_t data_size,
                                         uint8_t offset_B)
{
    uint8_t * p_clear = p_aes_data->cleartext;
    uint8_t * p_cipher = p_aes_data->ciphertext;

    while (data_size != 0)
    {
        if (data_size < (CCM_BLOCK_SIZE - offset_B))
        {
            memcpy(&p_clear[offset_B], p_data, data_size);
            memset(&p_clear[offset_B + data_size], 0x00, CCM_BLOCK_SIZE - (offset_B + data_size));
            data_size = 0;
        }
        else
        {
            memcpy(&p_clear[offset_B], p_data, (CCM_BLOCK_SIZE - offset_B));
            data_size -= (CCM_BLOCK_SIZE - offset_B);
            p_data += (CCM_BLOCK_SIZE - offset_B);
        }

        offset_B = 0;

        utils_xor(p_clear, p_cipher, p_clear, CCM_BLOCK_SIZE);

        aes_encrypt((nrf_ecb_hal_data_t *) p_aes_data);
    }
}

static void ccm_soft_authenticate(ccm_soft_data_t * p_data, aes_data_t * p_aes_data, uint8_t * T)
{
    b0_t * p_b0 = (b0_t *) &p_aes_data->cleartext[0];

    /* construct B0 */
    p_b0->flags = (
            ((p_data->a_len > 0 ? 1 : 0) << 6)        |
            ((((p_data->mic_len - 2)/2) & 0x07) << 3) |
            ((L_LEN - 1) & 0x07));

    memcpy(p_b0->nonce, p_data->p_nonce, CCM_NONCE_LENGTH);
    p_b0->length_field = LE2BE16(p_data->m_len);

    aes_encrypt((nrf_ecb_hal_data_t *) p_aes_data);

    if (p_data->a_len > 0)
    {
        NRF_MESH_ASSERT(p_data->a_len < 0xFF00); /* Longer a-data requires different (unsupported) encoding */
        *((uint16_t *) &p_aes_data->cleartext[0]) = LE2BE16(p_data->a_len);

        ccm_soft_authenticate_blocks(p_aes_data, p_data->p_a, p_data->a_len, 2);
    }

    if (p_data->m_len > 0)
    {
        ccm_soft_authenticate_blocks(p_aes_data, p_data->p_m, p_data->m_len, 0);
    }

    memcpy(T, p_aes_data->ciphertext, p_data->mic_len);
}

/**
 * Encrypt all data. Assumes p_aes_data already has key set and cleartext=A[0]
 */
static void ccm_soft_crypt(ccm_soft_data_t * p_data, aes_data_t * p_aes_data)
{
    uint16_t i = 1;
    uint16_t octets_m = p_data->m_len;

    a_block_t * p_a = (a_block_t *) p_aes_data->cleartext;

    while (octets_m)
    {
        /* Just alter the already created A-block */
        p_a->counter = LE2BE16(i);
        /* S[i] = AES(A[i]) */
        aes_encrypt((nrf_ecb_hal_data_t *) p_aes_data);

        uint8_t block_size = (octets_m > CCM_BLOCK_SIZE ? CCM_BLOCK_SIZE : octets_m);
        /* enc_data = (S xor data) */
        utils_xor(&p_data->p_out[CCM_BLOCK_SIZE * (i - 1)],
                  &p_data->p_m[CCM_BLOCK_SIZE * (i - 1)],
                  p_aes_data->ciphertext,
                  block_size);
        octets_m -= block_size;
        i++;
    }
}

static inline void build_a_block(const uint8_t * p_nonce, void * A0, uint16_t i)
{
    a_block_t * p_a_block = (a_block_t *) A0;
    p_a_block->len_field_len = (L_LEN - 1); /* encoded */
    memcpy(p_a_block->nonce, p_nonce, CCM_NONCE_LENGTH);
    p_a_block->counter = LE2BE16(i);
}

static inline void build_mic(ccm_soft_data_t * p_ccm_data, aes_data_t * p_aes_data, uint8_t * T, uint8_t * p_mic_out)
{
    build_a_block(p_ccm_data->p_nonce, p_aes_data->cleartext, 0);

    /* S0 = AES(A0) */
    aes_encrypt((nrf_ecb_hal_data_t *) p_aes_data);

    /* MIC = T ^ S0 */
    utils_xor(p_mic_out, T, p_aes_data->ciphertext, p_ccm_data->mic_len);
}

void ccm_soft_encrypt(ccm_soft_data_t * p_data)
{
#if CCM_DEBUG_MODE_ENABLED
    __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_encrypt: IN ",  p_data->p_m, p_data->m_len);
#endif

    aes_data_t aes_data;

    memcpy(aes_data.key, p_data->p_key, CCM_BLOCK_SIZE);

    ccm_soft_authenticate(p_data, &aes_data, p_data->p_mic);

    build_mic(p_data, &aes_data, p_data->p_mic, p_data->p_mic);

    /* aes_data.cleartext now contains A0, no need to regenerate it. */
    ccm_soft_crypt(p_data, &aes_data);

#if CCM_DEBUG_MODE_ENABLED
    __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_encrypt: OUT", p_data->p_out, p_data->m_len);
    __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_encrypt: MIC", p_data->p_mic, p_data->mic_len);
#endif
}

void ccm_soft_decrypt(ccm_soft_data_t * p_data, bool * p_mic_passed)
{
#if CCM_DEBUG_MODE_ENABLED
    __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_decrypt: IN",  p_data->p_m, p_data->m_len);
#endif
    NRF_MESH_ASSERT_DEBUG(p_data->mic_len <= CCM_MIC_LENGTH_MAX);

    aes_data_t aes_data;

    memcpy(aes_data.key, p_data->p_key, CCM_BLOCK_SIZE);

    if (p_data->m_len > 0)
    {
        /* Try to decrypt data with ciphers. */
        build_a_block(p_data->p_nonce, aes_data.cleartext, 0);
        ccm_soft_crypt(p_data, &aes_data);
    }

    const uint8_t * p_m = p_data->p_m;
    p_data->p_m = p_data->p_out;

    /* Authenticate data */
    uint8_t mic_out[CCM_MIC_LENGTH_MAX];

    ccm_soft_authenticate(p_data, &aes_data, mic_out);
    build_mic(p_data, &aes_data, mic_out, mic_out);

    p_data->p_m = p_m;
#if CCM_DEBUG_MODE_ENABLED
    __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_decrypt: OUT", p_data->p_out, p_data->m_len);
    __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_decrypt: MIC", mic_out, p_data->mic_len);
#endif

    *p_mic_passed = memcmp(mic_out, p_data->p_mic, p_data->mic_len) == 0;
#if CCM_DEBUG_MODE_ENABLED
    if (!*p_mic_passed)
    {
        /* No MIC match. */
        __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_decrypt: mic_in", p_data->p_mic, p_data->mic_len);
        __LOG_XB(LOG_SRC_CCM, LOG_LEVEL_INFO, "ccm_soft_decrypt: mic_out", mic_out, p_data->mic_len);
    }
#endif
}
