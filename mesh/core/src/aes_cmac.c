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
#include <stddef.h>
#include <string.h>
#include <nrf_error.h>

#include "utils.h"
#include "log.h"
#include "aes.h"

#include "aes_cmac.h"

static uint8_t m_subkey1[16];
static uint8_t m_subkey2[16];

/* 128-bit scratch buffer */
static uint8_t m_128buf[16];

static void my_xor(uint8_t * p_dst, const uint8_t * p_src1, const uint8_t * p_src2, uint16_t size)
{
    while (0 != size)
    {
        size--;
        p_dst[size] = p_src1[size] ^ p_src2[size];
    }
}


void aes_cmac_subkey_generate(const uint8_t * const p_key, uint8_t * p_subkey1_out, uint8_t * p_subkey2_out)
{
    static const uint8_t Rb[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x87};

    memset(m_128buf, 0x00, sizeof(m_128buf));

    aes_encrypt(p_key, m_128buf, m_128buf);

    /* Calculate K1 */
    if ((m_128buf[0] & 0x80) == 0)
    {
        utils_lshift(p_subkey1_out, m_128buf, sizeof(m_128buf));
    }
    else
    {
        utils_lshift(p_subkey1_out, m_128buf, sizeof(m_128buf));
        utils_xor(p_subkey1_out, p_subkey1_out, Rb, sizeof(m_128buf));
    }

    /* Calculate K2 */
    if ((p_subkey1_out[0] & 0x80) == 0)
    {
        utils_lshift(p_subkey2_out, p_subkey1_out, sizeof(m_128buf));
    }
    else
    {
        utils_lshift(p_subkey2_out, p_subkey1_out, sizeof(m_128buf));
        utils_xor(p_subkey2_out, p_subkey2_out, Rb, sizeof(m_128buf));
    }
}


void aes_cmac(const uint8_t * const p_key, const uint8_t * const p_msg, uint16_t msg_len, uint8_t * const p_out)
{
    uint16_t num_blocks = (msg_len + 15)/16;

    //__LOG_XB(LOG_SRC_ENC, LOG_LEVEL_INFO, "CMAC Key", p_key, 16);

    aes_cmac_subkey_generate(p_key, m_subkey1, m_subkey2);

    /* First X is zero */
    memset(m_128buf, 0x00, sizeof(m_128buf));

    /* Last block */
    uint8_t last[16];
    uint8_t remainder = (msg_len % 16);

    if ( num_blocks > 0 && remainder == 0)
    {
        utils_xor(last, &p_msg[(num_blocks-1)*16], m_subkey1, sizeof(m_128buf));
    }
    else
    {
        /* Note: this covers the case num_blocks == 0. */
        /**
         * @todo cleanup!
         */

        utils_pad(last, &p_msg[16*(num_blocks - 1)], remainder);
        utils_xor(last, last, m_subkey2, sizeof(m_128buf));
    }

    /* num_blocks may be zero! */
    for (int i = 0; i < num_blocks - 1; ++i)
    {
        /* Y := X XOR M_i     */
        /* X := AES-128(K, Y) */
        utils_xor(m_128buf, m_128buf, &p_msg[i*16], sizeof(m_128buf));
        aes_encrypt(p_key, m_128buf, m_128buf);
    }


    my_xor(m_128buf, last, m_128buf, 16);
    aes_encrypt(p_key, m_128buf, p_out);
}
