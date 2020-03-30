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

#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "enc.h"
#include "packet.h"
#include "utils.h"

#define ENC_TEST_S1_INPUT_DATA  { 't', 'e', 's', 't' }
#define ENC_TEST_S1_RESULT_DATA { 0xb7, 0x3c, 0xef, 0xbd, 0x64, 0x1e, 0xf2, 0xea, 0x59, 0x8c, 0x2b, 0x6e, 0xfb, 0x62, 0xf7, 0x9c }

#define ENC_TEST_K1_INPUT_IKM   { 0x32, 0x16, 0xd1, 0x50, 0x98, 0x84, 0xb5, 0x33, 0x24, 0x85, 0x41, 0x79, 0x2b, 0x87, 0x7f, 0x98 }
#define ENC_TEST_K1_INPUT_SALT  { 0x2b, 0xa1, 0x4f, 0xfa, 0x0d, 0xf8, 0x4a, 0x28, 0x31, 0x93, 0x8d, 0x57, 0xd2, 0x76, 0xca, 0xb4 }
#define ENC_TEST_K1_INPUT_P     { 0x5a, 0x09, 0xd6, 0x07, 0x97, 0xee, 0xb4, 0x47, 0x8a, 0xad, 0xa5, 0x9d, 0xb3, 0x35, 0x2a, 0x0d }
#define ENC_TEST_K1_RESULT_DATA { 0xf6, 0xed, 0x15, 0xa8, 0x93, 0x4a, 0xfb, 0xe7, 0xd8, 0x3e, 0x8d, 0xcb, 0x57, 0xfc, 0xf5, 0xd7 }

#define ENC_TEST_K2_INPUT_NETKEY { 0xf7, 0xa2, 0xa4, 0x4f, 0x8e, 0x8a, 0x80, 0x29, 0x06, 0x4f, 0x17, 0x3d, 0xdc, 0x1e, 0x2b, 0x00 }
#define ENC_TEST_K2_INPUT_P      { 0x00 }
#define ENC_TEST_K2_RESULT_DATA  { 0x7f, \
    0x9f, 0x58, 0x91, 0x81, 0xa0, 0xf5, 0x0d, 0xe7, 0x3c, 0x80, 0x70, 0xc7, 0xa6, 0xd2, 0x7f, 0x46, \
    0x4c, 0x71, 0x5b, 0xd4, 0xa6, 0x4b, 0x93, 0x8f, 0x99, 0xb4, 0x53, 0x35, 0x16, 0x53, 0x12, 0x4f }

#define ENC_TEST_K3_INPUT_DATA  { 0xf7, 0xa2, 0xa4, 0x4f, 0x8e, 0x8a, 0x80, 0x29, 0x06, 0x4f, 0x17, 0x3d, 0xdc, 0x1e, 0x2b, 0x00 }
#define ENC_TEST_K3_RESULT_DATA { 0xff, 0x04, 0x69, 0x58, 0x23, 0x3d, 0xb0, 0x14 }

#define ENC_TEST_K4_INPUT_DATA_1  { 0x32, 0x16, 0xd1, 0x50, 0x98, 0x84, 0xb5, 0x33, 0x24, 0x85, 0x41, 0x79, 0x2b, 0x87, 0x7f, 0x98 }
#define ENC_TEST_K4_RESULT_DATA_1 0x38
#define ENC_TEST_K4_INPUT_DATA_2  { 0x63, 0x96, 0x47, 0x71, 0x73, 0x4f, 0xbd, 0x76, 0xe3, 0xb4, 0x05, 0x19, 0xd1, 0xd9, 0x4a, 0x48 }
#define ENC_TEST_K4_RESULT_DATA_2 0x26

/* Nonce test vectors constructed from the sample messages in @tagMeshSp: */
#define EXPECTED_ARRAY(...) { __VA_ARGS__ }
#define EXPECTED_NET(...) EXPECTED_ARRAY(__VA_ARGS__)
#define EXPECTED_APP(...) EXPECTED_ARRAY(__VA_ARGS__)
#define EXPECTED_PROXY(...) EXPECTED_ARRAY(__VA_ARGS__)
#define NONCE_TV(test_ctl, test_ttl, test_seq, test_src, test_dst_type, test_dst, test_ivi, aszmic, expected_net, expected_app, expected_proxy) \
    {.metadata = {.control_packet = test_ctl, .ttl = test_ttl, .src = test_src, .dst = {.type = test_dst_type, .value = test_dst}, .internal = {.sequence_number = test_seq, .iv_index = test_ivi}}, \
        .net_nonce = expected_net, .app_nonce = expected_app, .proxy_nonce = expected_proxy }
/*lint -save -e572 -e648 Ignore overflow and excessive shifts in the initialization below */
static const struct
{
    network_packet_metadata_t metadata;
    uint8_t  aszmic : 1;
    uint8_t  net_nonce[CCM_NONCE_LENGTH];
    uint8_t  app_nonce[CCM_NONCE_LENGTH];
    uint8_t  proxy_nonce[CCM_NONCE_LENGTH];
} nonce_test_vectors[] = {
    /* Test message #6: */
    NONCE_TV(0, 4, 0x3129ab, 0x0003, NRF_MESH_ADDRESS_TYPE_UNICAST, 0x1201, 0x12345678, 0,
            EXPECTED_NET(0x00, 0x04, 0x31, 0x29, 0xab, 0x00, 0x03, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_APP(0x02, 0x00, 0x31, 0x29, 0xab, 0x00, 0x03, 0x12, 0x01, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_PROXY(0x03, 0x00, 0x31, 0x29, 0xab, 0x00, 0x03, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78)),
    /* Test message #16: */
    NONCE_TV(0, 11, 0x000006, 0x1201, NRF_MESH_ADDRESS_TYPE_UNICAST, 0x0003, 0x12345678, 0,
            EXPECTED_NET(0x00, 0x0b, 0x00, 0x00, 0x06, 0x12, 0x01, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_APP(0x02, 0x00, 0x00, 0x00, 0x06, 0x12, 0x01, 0x00, 0x03, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_PROXY(0x03, 0x00, 0x00, 0x00, 0x06, 0x12, 0x01, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78)),
    /* Test message #18: */
    NONCE_TV(0, 3, 0x000007, 0x1201, NRF_MESH_ADDRESS_TYPE_GROUP, 0xffff, 0x12345678, 0,
            EXPECTED_NET(0x00, 0x03, 0x00, 0x00, 0x07, 0x12, 0x01, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_APP(0x01, 0x00, 0x00, 0x00, 0x07, 0x12, 0x01, 0xff, 0xff, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_PROXY(0x03, 0x00, 0x00, 0x00, 0x07, 0x12, 0x01, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78)),
    /* Test message #19: */
    NONCE_TV(0, 3, 0x000009, 0x1201, NRF_MESH_ADDRESS_TYPE_GROUP, 0xffff, 0x12345678, 0,
            EXPECTED_NET(0x00, 0x03, 0x00, 0x00, 0x09, 0x12, 0x01, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_APP(0x01, 0x00, 0x00, 0x00, 0x09, 0x12, 0x01, 0xff, 0xff, 0x12, 0x34, 0x56, 0x78),
            EXPECTED_PROXY(0x03, 0x00, 0x00, 0x00, 0x09, 0x12, 0x01, 0x00, 0x00, 0x12, 0x34, 0x56, 0x78)),
    /* Test message #20: */
    NONCE_TV(0, 3, 0x070809, 0x1234, NRF_MESH_ADDRESS_TYPE_GROUP, 0xffff, 0x12345677, 0,
            EXPECTED_NET(0x00, 0x03, 0x07, 0x08, 0x09, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_APP(0x01, 0x00, 0x07, 0x08, 0x09, 0x12, 0x34, 0xff, 0xff, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_PROXY(0x03, 0x00, 0x07, 0x08, 0x09, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77)),
    /* Test message #21: */
    NONCE_TV(0, 3, 0x07080a, 0x1234, NRF_MESH_ADDRESS_TYPE_VIRTUAL, 0x8105, 0x12345677, 0,
            EXPECTED_NET(0x00, 0x03, 0x07, 0x08, 0x0a, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_APP(0x01, 0x00, 0x07, 0x08, 0x0a, 0x12, 0x34, 0x81, 0x05, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_PROXY(0x03, 0x00, 0x07, 0x08, 0x0a, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77)),
    /* Test message #22: */
    NONCE_TV(0, 3, 0x07080b, 0x1234, NRF_MESH_ADDRESS_TYPE_VIRTUAL, 0xb529, 0x12345677, 0,
            EXPECTED_NET(0x00, 0x03, 0x07, 0x08, 0x0b, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_APP(0x01, 0x00, 0x07, 0x08, 0x0b, 0x12, 0x34, 0xb5, 0x29, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_PROXY(0x03, 0x00, 0x07, 0x08, 0x0b, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77)),
    /* Test message #23: */
    NONCE_TV(0, 3, 0x07080c, 0x1234, NRF_MESH_ADDRESS_TYPE_VIRTUAL, 0x9736, 0x12345677, 0,
            EXPECTED_NET(0x00, 0x03, 0x07, 0x08, 0x0c, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_APP(0x01, 0x00, 0x07, 0x08, 0x0c, 0x12, 0x34, 0x97, 0x36, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_PROXY(0x03, 0x00, 0x07, 0x08, 0x0c, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77)),
    /* Test message #24: */
    NONCE_TV(0, 3, 0x07080d, 0x1234, NRF_MESH_ADDRESS_TYPE_VIRTUAL, 0x9736, 0x12345677, 1,
            EXPECTED_NET(0x00, 0x03, 0x07, 0x08, 0x0d, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_APP(0x01, 0x00, 0x07, 0x08, 0x0d, 0x12, 0x34, 0x97, 0x36, 0x12, 0x34, 0x56, 0x77),
            EXPECTED_PROXY(0x03, 0x00, 0x07, 0x08, 0x0d, 0x12, 0x34, 0x00, 0x00, 0x12, 0x34, 0x56, 0x77))
};
/*lint -restore */

void setUp()
{
}

void tearDown()
{
}

void test_enc_nonce_generate(void)
{
    const unsigned int num_vectors = ARRAY_SIZE(nonce_test_vectors);
    for (unsigned int i = 0; i < num_vectors; ++i)
    {
        /* Net nonce */
        uint8_t output_buf[CCM_NONCE_LENGTH];
        enc_nonce_generate(&nonce_test_vectors[i].metadata,
                ENC_NONCE_NET, nonce_test_vectors[i].aszmic /* this bit is ignored for network nonces */, output_buf);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(nonce_test_vectors[i].net_nonce, output_buf, CCM_NONCE_LENGTH);

        /* App nonce */
        if (nonce_test_vectors[i].app_nonce[0] == ENC_NONCE_APP) /* Use the expected result to choose application/device nonce */
        {
            enc_nonce_generate(&nonce_test_vectors[i].metadata,
                    ENC_NONCE_APP, nonce_test_vectors[i].aszmic, output_buf);
        }
        else
        {
            enc_nonce_generate(&nonce_test_vectors[i].metadata,
                    ENC_NONCE_DEV, nonce_test_vectors[i].aszmic, output_buf);
        }
        TEST_ASSERT_EQUAL_HEX8_ARRAY(nonce_test_vectors[i].app_nonce, output_buf, CCM_NONCE_LENGTH);

        /* Proxy nonce */
        enc_nonce_generate(&nonce_test_vectors[i].metadata,
                ENC_NONCE_PROXY, nonce_test_vectors[i].aszmic /* this bit is ignored for proxy nonces */, output_buf);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(nonce_test_vectors[i].proxy_nonce, output_buf, CCM_NONCE_LENGTH);
    }
}

void test_s1(void)
{
    const uint8_t input[] = ENC_TEST_S1_INPUT_DATA;
    const uint8_t expected[] = ENC_TEST_S1_RESULT_DATA;
    uint8_t result[16];

    enc_s1(input, sizeof(input), result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, result, sizeof(expected));
}

void test_k1(void)
{
    const uint8_t input[] = ENC_TEST_K1_INPUT_IKM;
    const uint8_t input_salt[] = ENC_TEST_K1_INPUT_SALT;
    const uint8_t input_p[] = ENC_TEST_K1_INPUT_P;
    const uint8_t expected[] = ENC_TEST_K1_RESULT_DATA;
    uint8_t result[NRF_MESH_KEY_SIZE] = {};

    enc_k1(input, sizeof(input), input_salt, input_p, sizeof(input_p), result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, result, sizeof(expected));
}

void test_k2_master(void)
{
    const uint8_t input_netkey[] = ENC_TEST_K2_INPUT_NETKEY;
    const uint8_t input_p[] = ENC_TEST_K2_INPUT_P;
    uint8_t expected[] = ENC_TEST_K2_RESULT_DATA;
    nrf_mesh_network_secmat_t result;

    enc_k2(input_netkey, input_p, sizeof(input_p), &result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, (uint8_t *) &result, sizeof(expected));
}

void test_k3(void)
{
    const uint8_t input[] = ENC_TEST_K3_INPUT_DATA;
    const uint8_t expected[] = ENC_TEST_K3_RESULT_DATA;
    uint8_t result[8];

    enc_k3(input, result);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, result, sizeof(expected));
}

void test_k4_1(void)
{
    const uint8_t input[] = ENC_TEST_K4_INPUT_DATA_1;
    const uint8_t expected = ENC_TEST_K4_RESULT_DATA_1;
    uint8_t result;

    enc_k4(input, &result);
    TEST_ASSERT_EQUAL_UINT8(expected, result);
}

void test_k4_2(void)
{
    const uint8_t input[] = ENC_TEST_K4_INPUT_DATA_2;
    const uint8_t expected = ENC_TEST_K4_RESULT_DATA_2;
    uint8_t result;

    enc_k4(input, &result);
    TEST_ASSERT_EQUAL_UINT8(expected, result);
}

