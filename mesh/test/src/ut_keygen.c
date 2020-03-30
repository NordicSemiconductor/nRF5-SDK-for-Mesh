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

#include <unity.h>
#include <cmock.h>

#include "nrf_mesh_keygen.h"

/*****************************************************************************
* Test vectors (from the Sample data section in @tagMeshSp)
*****************************************************************************/

#define APPLICATION_KEY    { 0x63, 0x96, 0x47, 0x71, 0x73, 0x4f, 0xbd, 0x76,\
                             0xe3, 0xb4, 0x05, 0x19, 0xd1, 0xd9, 0x4a, 0x48 }
#define NETWORK_KEY        { 0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18,\
                             0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6 }
#define VIRTUAL_LABEL_UUID { 0x00, 0x73, 0xe7, 0xe4, 0xd8, 0xb9, 0x44, 0x0f,\
                             0xaf, 0x84, 0x15, 0xdf, 0x4c, 0x56, 0xc0, 0xe1 }

#define EXPECTED_AID             0x26
#define EXPECTED_NID             0x68
#define EXPECTED_VIRTUAL_ADDRESS 0xb529

#define EXPECTED_ENCRYPTION_KEY { 0x09, 0x53, 0xfa, 0x93, 0xe7, 0xca, 0xac, 0x96,\
                                  0x38, 0xf5, 0x88, 0x20, 0x22, 0x0a, 0x39, 0x8e }
#define EXPECTED_PRIVACY_KEY    { 0x8b, 0x84, 0xee, 0xde, 0xc1, 0x00, 0x06, 0x7d,\
                                  0x67, 0x09, 0x71, 0xdd, 0x2a, 0xa7, 0x00, 0xcf }
#define EXPECTED_BEACON_KEY     { 0x54, 0x23, 0xd9, 0x67, 0xda, 0x63, 0x9a, 0x99,\
                                  0xcb, 0x02, 0x23, 0x1a, 0x83, 0xf7, 0xd2, 0x54 }
#define EXPECTED_NETWORK_ID     { 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70 }
#define EXPECTED_IDENTITY_KEY   { 0x84, 0x39, 0x6c, 0x43, 0x5a, 0xc4, 0x85, 0x60,\
                                  0xb5, 0x96, 0x53, 0x85, 0x25, 0x3e, 0x21, 0x0c }

/* Sample data from section 8.1.3 and 8.1.4 */
#define FND_NETWORK_KEY                 { 0xf7, 0xa2, 0xa4, 0x4f, 0x8e, 0x8a, 0x80, 0x29, \
                                          0x06, 0x4f, 0x17, 0x3d, 0xdc, 0x1e, 0x2b, 0x00 }

#define FND_EXPECTED_NID                (0x73)
#define FND_EXPECTED_ENCRYPTION_KEY     { 0x11, 0xef, 0xec, 0x06, 0x42, 0x77, 0x49, 0x92, \
                                         0x51, 0x0f, 0xb5, 0x92, 0x96, 0x46, 0xdf, 0x49 }
#define FND_EXPECTED_PRIVACY_KEY        { 0xd4, 0xd7, 0xcc, 0x0d, 0xfa, 0x77, 0x2d, 0x83, \
                                         0x6a, 0x8d, 0xf9, 0xdf, 0x55, 0x10, 0xd7, 0xa7 }

#define LPN_ADDRESS                     (0x0203)
#define FND_ADDRESS                     (0x0405)
#define LPN_COUNTER                     (0x0607)
#define FND_COUNTER                     (0x0809)

/*****************************************************************************
* Setup functions
*****************************************************************************/

void setUp(void)
{
}

void tearDown(void)
{
}

/*****************************************************************************
* Tests
*****************************************************************************/

void test_aid(void)
{
    const uint8_t key[NRF_MESH_KEY_SIZE] = APPLICATION_KEY;
    uint8_t aid;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_aid(NULL, &aid));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_aid(key, NULL));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_aid(key, &aid));
    TEST_ASSERT_EQUAL_HEX8(EXPECTED_AID, aid);
}

void test_network_secmat(void)
{
    const uint8_t key[NRF_MESH_KEY_SIZE] = NETWORK_KEY;
    nrf_mesh_network_secmat_t secmat;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_network_secmat(NULL, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_network_secmat(key, NULL));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_network_secmat(key, &secmat));

    const uint8_t expected_encryption_key[NRF_MESH_KEY_SIZE] = EXPECTED_ENCRYPTION_KEY;
    const uint8_t expected_privacy_key[NRF_MESH_KEY_SIZE] = EXPECTED_PRIVACY_KEY;
    TEST_ASSERT_EQUAL_HEX8(EXPECTED_NID, secmat.nid);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_encryption_key, secmat.encryption_key, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_privacy_key, secmat.privacy_key, NRF_MESH_KEY_SIZE);
}

void test_network_friendship_secmat(void)
{
    const uint8_t key[NRF_MESH_KEY_SIZE] = FND_NETWORK_KEY;
    nrf_mesh_network_secmat_t secmat;
    nrf_mesh_keygen_friendship_secmat_params_t secmat_params;

    secmat_params.lpn_address = LPN_ADDRESS;
    secmat_params.friend_address = FND_ADDRESS;
    secmat_params.lpn_counter = LPN_COUNTER;
    secmat_params.friend_counter = FND_COUNTER;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_friendship_secmat(key, &secmat_params, &secmat));

    const uint8_t expected_encryption_key[NRF_MESH_KEY_SIZE] = FND_EXPECTED_ENCRYPTION_KEY;
    const uint8_t expected_privacy_key[NRF_MESH_KEY_SIZE] = FND_EXPECTED_PRIVACY_KEY;
    TEST_ASSERT_EQUAL_HEX8(FND_EXPECTED_NID, secmat.nid);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_encryption_key, secmat.encryption_key, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_privacy_key, secmat.privacy_key, NRF_MESH_KEY_SIZE);
}

void test_beacon_secmat(void)
{
    const uint8_t key[NRF_MESH_KEY_SIZE] = NETWORK_KEY;
    nrf_mesh_beacon_secmat_t secmat;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_beacon_secmat(NULL, &secmat));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_beacon_secmat(key, NULL));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_beacon_secmat(key, &secmat));

    const uint8_t expected_beacon_key[NRF_MESH_KEY_SIZE] = EXPECTED_BEACON_KEY;
    const uint8_t expected_network_id[NRF_MESH_NETID_SIZE] = EXPECTED_NETWORK_ID;
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_beacon_key, secmat.key, NRF_MESH_KEY_SIZE);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_network_id, secmat.net_id, NRF_MESH_NETID_SIZE);
}

void test_identity_key(void)
{
    const uint8_t key[NRF_MESH_KEY_SIZE] = NETWORK_KEY;
    uint8_t identity_key[NRF_MESH_KEY_SIZE];
    const uint8_t expected_identity_key[NRF_MESH_KEY_SIZE] = EXPECTED_IDENTITY_KEY;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_identitykey(NULL, identity_key));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_identitykey(key, NULL));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_identitykey(key, identity_key));
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_identity_key, identity_key, NRF_MESH_KEY_SIZE);
}

void test_virtual_address(void)
{
    uint8_t uuid[NRF_MESH_KEY_SIZE] = VIRTUAL_LABEL_UUID;
    uint16_t address;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_virtual_address(NULL, &address));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_keygen_virtual_address(uuid, NULL));

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_keygen_virtual_address(uuid, &address));
    TEST_ASSERT_EQUAL_HEX16(EXPECTED_VIRTUAL_ADDRESS, address);
}

