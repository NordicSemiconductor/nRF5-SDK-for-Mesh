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

#include <stdbool.h>
#include <string.h>
#include <unity.h>

#include "ccm_soft.h"

/* Scratch buffer */
static uint8_t m_out_buffer[64];

static ccm_soft_data_t ccm_data;

/* Sample data from rfc3610.txt */
static uint8_t tv1_key[]   = {0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
                              0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf};
static uint8_t tv1_nonce[] = {0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xa0,
                              0xa1, 0xa2, 0xa3, 0xa4, 0xa5};
static uint8_t tv1_in[]    = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                              0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                              0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                              0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e};
static uint8_t tv1_out[]   = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                              0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2,
                              0xf0, 0x66, 0xd0, 0xc2, 0xc0, 0xf9, 0x89, 0x80,
                              0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84, 0x17,
                              0xe8, 0xd1, 0x2c, 0xfd, 0xf9, 0x26, 0xe0};
static uint16_t tv1_adlen  = 8;

static uint8_t tv2_key[] = {0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
                            0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf};
static uint8_t tv2_nonce[] = {0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01, 0xa0,
                              0xa1, 0xa2, 0xa3, 0xa4, 0xa5};
static uint8_t tv2_in[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
                           0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
                           0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f};
static uint8_t tv2_out[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                            0x72, 0xc9, 0x1a, 0x36, 0xe1, 0x35, 0xf8, 0xcf,
                            0x29, 0x1c, 0xa8, 0x94, 0x08, 0x5c, 0x87, 0xe3,
                            0xcc, 0x15, 0xc4, 0x39, 0xc9, 0xe4, 0x3a, 0x3b,
                            0xa0, 0x91, 0xd5, 0x6e, 0x10, 0x40, 0x09, 0x16};
static uint16_t tv2_adlen  = 8;

static uint8_t tv18_key[] = {0xd7, 0x82, 0x8d, 0x13, 0xb2, 0xb0, 0xbd, 0xc3,
                             0x25, 0xa7, 0x62, 0x36, 0xdf, 0x93, 0xcc, 0x6b};
static uint8_t tv18_nonce[] = {0x00, 0xd5, 0x60, 0x91, 0x2d, 0x3f, 0x70, 0x3c,
                               0x96, 0x96, 0x76, 0x6c, 0xfa};
static uint8_t tv18_in[] = {0xcd, 0x90, 0x44, 0xd2, 0xb7, 0x1f, 0xdb, 0x81,
                            0x20, 0xea, 0x60, 0xc0, 0x64, 0x35, 0xac, 0xba,
                            0xfb, 0x11, 0xa8, 0x2e, 0x2f, 0x07, 0x1d, 0x7c,
                            0xa4, 0xa5, 0xeb, 0xd9, 0x3a, 0x80, 0x3b, 0xa8,
                            0x7f};
static uint8_t tv18_out[] = {0xcd, 0x90, 0x44, 0xd2, 0xb7, 0x1f, 0xdb, 0x81,
                             0x20, 0xea, 0x60, 0xc0, 0x00, 0x97, 0x69, 0xec,
                             0xab, 0xdf, 0x48, 0x62, 0x55, 0x94, 0xc5, 0x92,
                             0x51, 0xe6, 0x03, 0x57, 0x22, 0x67, 0x5e, 0x04,
                             0xc8, 0x47, 0x09, 0x9e, 0x5a, 0xe0, 0x70, 0x45,
                             0x51};
static uint16_t tv18_adlen = 12;

void setUp(void)
{
}

void tearDown(void)
{
}

void test_tv1(void)
{
    ccm_data.p_key   = tv1_key;
    ccm_data.p_nonce = tv1_nonce;
    ccm_data.p_m     = &tv1_in[tv1_adlen];
    ccm_data.m_len   = sizeof(tv1_in) - tv1_adlen;
    ccm_data.p_a     = tv1_in;
    ccm_data.a_len   = tv1_adlen;
    ccm_data.p_mic   = &m_out_buffer[sizeof(tv1_in)];
    ccm_data.p_out   = &m_out_buffer[tv1_adlen];
    ccm_data.mic_len = sizeof(tv1_out) - sizeof(tv1_in);

    memcpy(m_out_buffer, tv1_in, tv1_adlen);
    ccm_soft_encrypt(&ccm_data);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tv1_out, m_out_buffer, sizeof(tv1_out));


    /* Test decrypt. */
    memset(m_out_buffer, 0x00, sizeof(m_out_buffer));
    memcpy(m_out_buffer, tv1_out, sizeof(tv1_out));
    ccm_data.p_key   = tv1_key;
    ccm_data.p_nonce = tv1_nonce;
    ccm_data.p_m     = &m_out_buffer[tv1_adlen];
    ccm_data.m_len   = sizeof(tv1_out) - tv1_adlen - 8; /* - additional data - mic */
    ccm_data.p_a     = m_out_buffer;
    ccm_data.a_len   = tv1_adlen;
    ccm_data.p_mic   = &m_out_buffer[sizeof(tv1_in)];
    ccm_data.p_out   = &m_out_buffer[tv1_adlen];
    ccm_data.mic_len = sizeof(tv1_out) - sizeof(tv1_in);

    bool mic_passed;
    ccm_soft_decrypt(&ccm_data, &mic_passed);

    TEST_ASSERT_TRUE(mic_passed);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tv1_in, m_out_buffer, sizeof(tv1_in));
}

void test_tv2(void)
{
    ccm_data.p_key   = tv2_key;
    ccm_data.p_nonce = tv2_nonce;
    ccm_data.p_m     = &tv2_in[tv2_adlen];
    ccm_data.m_len   = sizeof(tv2_in) - tv2_adlen;
    ccm_data.p_a     = tv2_in;
    ccm_data.a_len   = tv2_adlen;
    ccm_data.p_mic   = &m_out_buffer[sizeof(tv2_in)];
    ccm_data.p_out   = &m_out_buffer[tv2_adlen];
    ccm_data.mic_len = sizeof(tv2_out) - sizeof(tv2_in);

    memcpy(m_out_buffer, tv2_in, tv2_adlen);
    ccm_soft_encrypt(&ccm_data);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tv2_out, m_out_buffer, sizeof(tv2_out));


    /* Test decrypt. */
    memset(m_out_buffer, 0x00, sizeof(m_out_buffer));
    memcpy(m_out_buffer, tv2_out, sizeof(tv2_out));
    ccm_data.p_key   = tv2_key;
    ccm_data.p_nonce = tv2_nonce;
    ccm_data.p_m     = &m_out_buffer[tv2_adlen];
    ccm_data.m_len   = sizeof(tv2_out) - tv2_adlen - 8; /* - additional data - mic */
    ccm_data.p_a     = m_out_buffer;
    ccm_data.a_len   = tv2_adlen;
    ccm_data.p_mic   = &m_out_buffer[sizeof(tv2_in)];
    ccm_data.p_out   = &m_out_buffer[tv2_adlen];
    ccm_data.mic_len = sizeof(tv2_out) - sizeof(tv2_in);

    bool mic_passed;
    ccm_soft_decrypt(&ccm_data, &mic_passed);

    TEST_ASSERT_TRUE(mic_passed);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tv2_in, m_out_buffer, sizeof(tv2_in));
}

void test_tv18(void)
{
    ccm_data.p_key   = tv18_key;
    ccm_data.p_nonce = tv18_nonce;
    ccm_data.p_m     = &tv18_in[tv18_adlen];
    ccm_data.m_len   = sizeof(tv18_in) - tv18_adlen;
    ccm_data.p_a     = tv18_in;
    ccm_data.a_len   = tv18_adlen;
    ccm_data.p_mic   = &m_out_buffer[sizeof(tv18_in)];
    ccm_data.p_out   = &m_out_buffer[tv18_adlen];
    ccm_data.mic_len = sizeof(tv18_out) - sizeof(tv18_in);

    memcpy(m_out_buffer, tv18_in, tv18_adlen);
    ccm_soft_encrypt(&ccm_data);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tv18_out, m_out_buffer, sizeof(tv18_out));


    /* Test decrypt. */
    memset(m_out_buffer, 0x00, sizeof(m_out_buffer));
    memcpy(m_out_buffer, tv18_out, sizeof(tv18_out));
    ccm_data.p_key   = tv18_key;
    ccm_data.p_nonce = tv18_nonce;
    ccm_data.p_m     = &m_out_buffer[tv18_adlen];
    ccm_data.m_len   = sizeof(tv18_out) - tv18_adlen - 8; /* - additional data - mic */
    ccm_data.p_a     = m_out_buffer;
    ccm_data.a_len   = tv18_adlen;
    ccm_data.p_mic   = &m_out_buffer[sizeof(tv18_in)];
    ccm_data.p_out   = &m_out_buffer[tv18_adlen];
    ccm_data.mic_len = sizeof(tv18_out) - sizeof(tv18_in);

    bool mic_passed;
    ccm_soft_decrypt(&ccm_data, &mic_passed);

    TEST_ASSERT_TRUE(mic_passed);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(tv18_in, m_out_buffer, sizeof(tv18_in));
}
