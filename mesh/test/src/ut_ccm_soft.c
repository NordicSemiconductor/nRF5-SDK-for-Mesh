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

#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "ccm_soft.h"
#include "nrf_mesh_assert.h"

#define MIC_LEN 4

/* Test vectors: */
static const uint8_t m_unencrypted[] = { 0x06, 0x07, 0xa1, 0x01, 0x3f, 0x5e, 0x05, 0xd2, 0xbc, 0x8c, 0x26, 0x03 };
static const uint8_t m_encrypted[]   = { 0x21, 0x31, 0x93, 0xf0, 0x4f, 0xba, 0x43, 0xee, 0xbc, 0x8c, 0x26, 0x03 };
static const uint8_t m_key[]         = { 0xac, 0x16, 0x1f, 0x58, 0x9e, 0x5d, 0xe7, 0x45, 0x6d, 0x2c, 0x1a, 0x5f, 0x49, 0x72, 0x12, 0x6b };
static const uint8_t m_nonce[]       = { 0x07, 0x01, 0x02, 0x03, 0x04, 0x05, 0x26, 0xd2, 0xa6, 0x9d, 0xa0, 0x82, 0xe0 };

NRF_MESH_STATIC_ASSERT(sizeof(m_unencrypted) == sizeof(m_encrypted));
NRF_MESH_STATIC_ASSERT(sizeof(m_key) == NRF_MESH_KEY_SIZE);

void setUp(void)
{
}

void tearDown(void)
{
}


void test_ccm_soft_encrypt(void)
{
    uint8_t output[sizeof(m_unencrypted)] = {0};
    uint8_t mic[MIC_LEN] = {0};

    ccm_soft_data_t enc_data =
    {
        .p_key   = m_key,
        .p_nonce = m_nonce,
        .p_m     = m_unencrypted,
        .m_len   = sizeof(m_unencrypted) - MIC_LEN,
        .mic_len = MIC_LEN,
        .p_mic   = mic,
        .p_out   = output
    };

    ccm_soft_encrypt(&enc_data);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&m_encrypted[sizeof(m_encrypted) - MIC_LEN], mic, MIC_LEN);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_encrypted, output, sizeof(m_encrypted) - MIC_LEN);
}

void test_ccm_soft_decrypt(void)
{
    uint8_t output[sizeof(m_encrypted) - MIC_LEN] = {0};

    ccm_soft_data_t dec_data =
    {
        .p_key   = m_key,
        .p_nonce = m_nonce,
        .p_m     = m_encrypted,
        .m_len   = sizeof(m_encrypted) - MIC_LEN,
        .mic_len = MIC_LEN,
        .p_mic   = (uint8_t *) &m_encrypted[sizeof(m_encrypted) - MIC_LEN],
        .p_out   = output
    };

    bool mic_passed = false;
    ccm_soft_decrypt(&dec_data, &mic_passed);

    TEST_ASSERT_EQUAL(true, mic_passed);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_unencrypted, output, sizeof(m_encrypted) - MIC_LEN);
}
