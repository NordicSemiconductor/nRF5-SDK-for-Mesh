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

#include "utils.h"
#include "prov_utils.h"
#include "test_assert.h"

#include "enc_mock.h"
#include "uECC_mock.h"
#include "rand_mock.h"
#include "mesh_config_entry.h"
#include "mesh_opt_prov.h"

typedef struct
{
    uint8_t method;
    uint8_t action;
} oob_method_and_action_pair_t;

static nrf_mesh_prov_ctx_t m_ctx;

extern mesh_config_entry_params_t m_ecdh_offloading_params;

uint32_t mesh_config_entry_set(mesh_config_entry_id_t id, const void * p_entry)
{
    return m_ecdh_offloading_params.callbacks.setter(id, p_entry);
}

uint32_t mesh_config_entry_get(mesh_config_entry_id_t id, void * p_entry)
{
    m_ecdh_offloading_params.callbacks.getter(id, p_entry);
    return NRF_SUCCESS;
}

/* Simple integer power function. Converting between integers and floats can cause rounding errors. */
static uint32_t pow__(uint32_t num, uint8_t p)
{
    uint32_t ret = 1;
    for (uint8_t i = 0; i < p; ++i)
    {
        ret *= num;
    }
    return ret;
}

void setUp(void)
{
    enc_mock_Init();
    uECC_mock_Init();
    rand_mock_Init();
}

void tearDown(void)
{
    enc_mock_Verify();
    enc_mock_Destroy();
    uECC_mock_Verify();
    uECC_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
}

/*****************************************************************************
* Tests
*****************************************************************************/

void test_authentication_values_derive(void)
{
    uint8_t confirmation_value[16];
    memset(confirmation_value, 0xCB, 16);
    uint8_t local_pubkey[NRF_MESH_ECDH_PUBLIC_KEY_SIZE] = {};
    m_ctx.p_public_key = local_pubkey;
    uint8_t conf_input[145];
    memset(m_ctx.confirmation_inputs, 0xAB, 17);
    memset(m_ctx.auth_value, 0xAD, 16);
    memset(m_ctx.node_random, 0xDB, 16);
    uint8_t random_and_auth[PROV_RANDOM_LEN + PROV_AUTH_LEN];
    memcpy(random_and_auth, m_ctx.node_random, PROV_RANDOM_LEN);
    memcpy(&random_and_auth[PROV_RANDOM_LEN], m_ctx.auth_value, PROV_AUTH_LEN);
    uint8_t confirmation_key[NRF_MESH_KEY_SIZE];
    memset(confirmation_key, 0xCF, NRF_MESH_KEY_SIZE);

    /***** AS PROVISIONER *****/
    m_ctx.role = NRF_MESH_PROV_ROLE_PROVISIONER;

    /* Build confirmation input */
    memset(conf_input, 0xAB, 17);
    memcpy(&conf_input[17], local_pubkey, NRF_MESH_ECDH_PUBLIC_KEY_SIZE);
    memcpy(&conf_input[17 + NRF_MESH_ECDH_PUBLIC_KEY_SIZE], m_ctx.peer_public_key, NRF_MESH_ECDH_PUBLIC_KEY_SIZE);

    /* conf salt generation */
    enc_s1_ExpectWithArray(
            conf_input,
            145,
            145,
            m_ctx.confirmation_salt,
            PROV_SALT_LEN);
    /* conf key generation */
    enc_k1_Expect(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            m_ctx.confirmation_salt,
            (const uint8_t *) "prck", /* Since the compiler stores identical strings together, this pointer will be the same */
            4,
            NULL);
    enc_k1_IgnoreArg_p_out();
    enc_k1_ReturnMemThruPtr_p_out(confirmation_key, 16);

    rand_hw_rng_get_Expect(m_ctx.node_random, 16);
    enc_aes_cmac_ExpectWithArray(confirmation_key, 16,
            random_and_auth,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            confirmation_value,
            16);
    enc_aes_cmac_ReturnMemThruPtr_p_result(confirmation_value, 16);

    prov_utils_authentication_values_derive(&m_ctx, m_ctx.confirmation_salt, confirmation_value, m_ctx.node_random);

    /***** AS PROVISIONEE *****/
    m_ctx.role = NRF_MESH_PROV_ROLE_PROVISIONEE;

    /* Build confirmation input */
    memset(conf_input, 0xAB, 17);
    memcpy(&conf_input[17], m_ctx.peer_public_key, NRF_MESH_ECDH_PUBLIC_KEY_SIZE);
    memcpy(&conf_input[17 + NRF_MESH_ECDH_PUBLIC_KEY_SIZE], local_pubkey, NRF_MESH_ECDH_PUBLIC_KEY_SIZE);

    /* conf salt generation */
    enc_s1_ExpectWithArray(
            conf_input,
            145,
            145,
            m_ctx.confirmation_salt,
            PROV_SALT_LEN);
    /* conf key generation */
    enc_k1_Expect(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            m_ctx.confirmation_salt,
            (const uint8_t *) "prck", /* Since the compiler stores identical strings together, this pointer will be the same */
            4,
            NULL);
    enc_k1_IgnoreArg_p_out();
    enc_k1_ReturnMemThruPtr_p_out(confirmation_key, 16);

    rand_hw_rng_get_Expect(m_ctx.node_random, 16);
    enc_aes_cmac_ExpectWithArray(confirmation_key, 16,
            random_and_auth,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            confirmation_value,
            16);
    enc_aes_cmac_ReturnMemThruPtr_p_result(confirmation_value, 16);

    prov_utils_authentication_values_derive(&m_ctx, m_ctx.confirmation_salt, confirmation_value, m_ctx.node_random);

}

void test_keys_generate(void)
{
    uint8_t pubkey[NRF_MESH_ECDH_PUBLIC_KEY_SIZE];
    uint8_t privkey[NRF_MESH_ECDH_PRIVATE_KEY_SIZE];
    uECC_secp256r1_ExpectAndReturn(NULL);
    uECC_make_key_ExpectAndReturn(pubkey, privkey, NULL, 1);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_utils_keys_generate(pubkey, privkey));
    uECC_secp256r1_ExpectAndReturn(NULL);
    uECC_make_key_ExpectAndReturn(pubkey, privkey, NULL, 0);
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INTERNAL, prov_utils_keys_generate(pubkey, privkey));
}

void test_derive_keys(void)
{
    uint8_t session_key[NRF_MESH_KEY_SIZE];
    uint8_t data_nonce[PROV_NONCE_LEN];
    uint8_t device_key[NRF_MESH_KEY_SIZE];
    uint8_t prov_salt[PROV_SALT_LEN];
    uint8_t temp_nonce[NRF_MESH_KEY_SIZE];
    for (uint32_t i = 0; i < NRF_MESH_KEY_SIZE; i++)
    {
        temp_nonce[i] = i;
    }

    memset(m_ctx.confirmation_salt, 0xCC, PROV_SALT_LEN);
    memset(m_ctx.node_random, 0x55, PROV_RANDOM_LEN);
    memset(m_ctx.peer_random, 0x66, PROV_RANDOM_LEN);

    uint8_t prov_salt_data[PROV_SALT_LEN + PROV_RANDOM_LEN + PROV_RANDOM_LEN];
    memcpy(prov_salt_data, m_ctx.confirmation_salt, PROV_SALT_LEN);
    memcpy(&prov_salt_data[PROV_SALT_LEN], m_ctx.node_random, PROV_SALT_LEN);
    memcpy(&prov_salt_data[PROV_SALT_LEN + PROV_RANDOM_LEN], m_ctx.peer_random, PROV_SALT_LEN);

    /***** AS PROVISIONER *****/
    m_ctx.role = NRF_MESH_PROV_ROLE_PROVISIONER;

    memset(prov_salt, 0x22, PROV_SALT_LEN);
    enc_s1_ExpectWithArray(prov_salt_data,
        PROV_SALT_LEN + PROV_RANDOM_LEN + PROV_RANDOM_LEN,
        PROV_SALT_LEN + PROV_RANDOM_LEN + PROV_RANDOM_LEN,
        NULL,
        0);
    enc_s1_IgnoreArg_p_out();
    enc_s1_ReturnMemThruPtr_p_out(prov_salt, PROV_SALT_LEN);

    /* Session key */
    enc_k1_ExpectWithArray(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            prov_salt,
            PROV_SALT_LEN,
            (const uint8_t *) "prsk",
            4,
            4,
            session_key,
            NRF_MESH_KEY_SIZE);

    /* Session nonce */
    enc_k1_ExpectWithArray(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            prov_salt,
            PROV_SALT_LEN,
            (const uint8_t *) "prsn",
            4,
            4,
            NULL,
            0);
    enc_k1_IgnoreArg_p_out();
    enc_k1_ReturnMemThruPtr_p_out(temp_nonce, NRF_MESH_KEY_SIZE);

    /* Device key */
    enc_k1_ExpectWithArray(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            prov_salt,
            PROV_SALT_LEN,
            (const uint8_t *) "prdk",
            4,
            4,
            device_key,
            NRF_MESH_KEY_SIZE);

    prov_utils_derive_keys(&m_ctx,
            session_key,
            data_nonce,
            device_key);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data_nonce, &temp_nonce[3], PROV_NONCE_LEN);

    /***** AS PROVISIONEE *****/
    m_ctx.role = NRF_MESH_PROV_ROLE_PROVISIONEE;
    memcpy(&prov_salt_data[PROV_SALT_LEN], m_ctx.peer_random, PROV_SALT_LEN);
    memcpy(&prov_salt_data[PROV_SALT_LEN + PROV_RANDOM_LEN], m_ctx.node_random, PROV_SALT_LEN);

    memset(prov_salt, 0x22, PROV_SALT_LEN);
    enc_s1_ExpectWithArray(prov_salt_data,
        PROV_SALT_LEN + PROV_RANDOM_LEN + PROV_RANDOM_LEN,
        PROV_SALT_LEN + PROV_RANDOM_LEN + PROV_RANDOM_LEN,
        NULL,
        0);
    enc_s1_IgnoreArg_p_out();
    enc_s1_ReturnMemThruPtr_p_out(prov_salt, PROV_SALT_LEN);

    /* Session key */
    enc_k1_ExpectWithArray(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            prov_salt,
            PROV_SALT_LEN,
            (const uint8_t *) "prsk",
            4,
            4,
            session_key,
            NRF_MESH_KEY_SIZE);

    /* Session nonce */
    enc_k1_ExpectWithArray(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            prov_salt,
            PROV_SALT_LEN,
            (const uint8_t *) "prsn",
            4,
            4,
            NULL,
            0);
    enc_k1_IgnoreArg_p_out();
    enc_k1_ReturnMemThruPtr_p_out(temp_nonce, NRF_MESH_KEY_SIZE);

    /* Device key */
    enc_k1_ExpectWithArray(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            prov_salt,
            PROV_SALT_LEN,
            (const uint8_t *) "prdk",
            4,
            4,
            device_key,
            NRF_MESH_KEY_SIZE);

    prov_utils_derive_keys(&m_ctx,
            session_key,
            data_nonce,
            device_key);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(data_nonce, &temp_nonce[3], PROV_NONCE_LEN);
}

void test_calculate_shared_secret(void)
{
    uint8_t privkey[NRF_MESH_ECDH_PRIVATE_KEY_SIZE] = {};
    uint8_t shared_secret[NRF_MESH_KEY_SIZE] = {};
    m_ctx.p_private_key  = privkey;
    uECC_secp256r1_ExpectAndReturn(NULL);
    uECC_valid_public_key_ExpectAndReturn(m_ctx.peer_public_key, NULL, 0);
    TEST_NRF_MESH_ASSERT_EXPECT(prov_utils_calculate_shared_secret(&m_ctx, shared_secret));

    uECC_secp256r1_ExpectAndReturn(NULL);
    uECC_valid_public_key_ExpectAndReturn(m_ctx.peer_public_key, NULL, 1);
    uECC_secp256r1_ExpectAndReturn(NULL);
    uECC_shared_secret_ExpectAndReturn(m_ctx.peer_public_key, privkey, shared_secret, NULL, 0);
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INTERNAL, prov_utils_calculate_shared_secret(&m_ctx, shared_secret));

    uECC_secp256r1_ExpectAndReturn(NULL);
    uECC_valid_public_key_ExpectAndReturn(m_ctx.peer_public_key, NULL, 1);
    uECC_secp256r1_ExpectAndReturn(NULL);
    uECC_shared_secret_ExpectAndReturn(m_ctx.peer_public_key, privkey, shared_secret, NULL, 1);
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_utils_calculate_shared_secret(&m_ctx, shared_secret));

}

void test_generate_oob_data(void)
{
    uint8_t auth_value[PROV_AUTH_LEN];
    uint8_t expected_auth_value[PROV_AUTH_LEN];
    memset(expected_auth_value, 0, PROV_AUTH_LEN);

    m_ctx.oob_size = 0; //invalid
    TEST_NRF_MESH_ASSERT_EXPECT(prov_utils_generate_oob_data(&m_ctx, auth_value));
    m_ctx.oob_size = 9; //invalid
    TEST_NRF_MESH_ASSERT_EXPECT(prov_utils_generate_oob_data(&m_ctx, auth_value));

    /* Run all valid oob sizes */
    for (m_ctx.oob_size = 1; m_ctx.oob_size <= 8; m_ctx.oob_size++)
    {
        memset(expected_auth_value, 0, PROV_AUTH_LEN);
        /* Test count type oob data */
        oob_method_and_action_pair_t count_pairs[] =
        {
            {NRF_MESH_PROV_OOB_METHOD_OUTPUT, NRF_MESH_PROV_OUTPUT_ACTION_BLINK},
            {NRF_MESH_PROV_OOB_METHOD_OUTPUT, NRF_MESH_PROV_OUTPUT_ACTION_BEEP},
            {NRF_MESH_PROV_OOB_METHOD_OUTPUT, NRF_MESH_PROV_OUTPUT_ACTION_VIBRATE},
            {NRF_MESH_PROV_OOB_METHOD_INPUT,  NRF_MESH_PROV_INPUT_ACTION_PUSH},
            {NRF_MESH_PROV_OOB_METHOD_INPUT,  NRF_MESH_PROV_INPUT_ACTION_TWIST}
        };
        for (uint32_t i = 0; i < ARRAY_SIZE(count_pairs); i++)
        {
            m_ctx.oob_method = (nrf_mesh_prov_oob_method_t) count_pairs[i].method;
            m_ctx.oob_action = count_pairs[i].action;

            uint8_t rand_output[] = {92, 11, 1, 0, 255, 128};
            for (uint32_t j = 0; j < sizeof(rand_output); j++)
            {
                rand_hw_rng_get_Expect(NULL, 1);
                rand_hw_rng_get_IgnoreArg_p_result();
                rand_hw_rng_get_ReturnMemThruPtr_p_result(&rand_output[j], 1);
                prov_utils_generate_oob_data(&m_ctx, auth_value);
                expected_auth_value[15] = (rand_output[j] % m_ctx.oob_size) + 1;
                TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_auth_value, auth_value, PROV_AUTH_LEN);
            }
        }

        /* Test numeric type oob data */
        oob_method_and_action_pair_t numeric_pairs[] =
        {
            {NRF_MESH_PROV_OOB_METHOD_OUTPUT, NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC},
            {NRF_MESH_PROV_OOB_METHOD_INPUT,  NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER},
        };
        for (uint32_t i = 0; i < ARRAY_SIZE(numeric_pairs); i++)
        {
            m_ctx.oob_method = (nrf_mesh_prov_oob_method_t) numeric_pairs[i].method;
            m_ctx.oob_action = numeric_pairs[i].action;

            uint32_t rand_output[] = {1000, 100, 99, 999, 10000000, 3, 7, 432483922, 0xFFFFFFFF, 11, 1, 0, 255, 128};
            for (uint32_t j = 0; j < ARRAY_SIZE(rand_output); j++)
            {
                rand_hw_rng_get_Expect(NULL, 4);
                rand_hw_rng_get_IgnoreArg_p_result();
                rand_hw_rng_get_ReturnMemThruPtr_p_result((uint8_t *) &rand_output[j], 4);
                prov_utils_generate_oob_data(&m_ctx, auth_value);
                uint32_t pow = pow__(10, m_ctx.oob_size);
                uint32_t value = LE2BE32(rand_output[j] % pow);
                memcpy(&expected_auth_value[PROV_AUTH_LEN - 4], &value, 4);
                char outputstring[16];
                sprintf(outputstring, "%d", LE2BE32(value));
                TEST_ASSERT_TRUE(strlen(outputstring) <= m_ctx.oob_size);
                TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_auth_value, auth_value, PROV_AUTH_LEN);
            }

        }
        /* Test alphanumeric type oob data */
        oob_method_and_action_pair_t alphanumeric_pairs[] =
        {
            {NRF_MESH_PROV_OOB_METHOD_OUTPUT, NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC},
            {NRF_MESH_PROV_OOB_METHOD_INPUT,  NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING},
        };
        for (uint32_t i = 0; i < ARRAY_SIZE(alphanumeric_pairs); i++)
        {
            m_ctx.oob_method = (nrf_mesh_prov_oob_method_t) alphanumeric_pairs[i].method;
            m_ctx.oob_action = alphanumeric_pairs[i].action;

            uint8_t rand_output[][8] = {
                {1, 2, 3, 4, 5, 6, 7, 8},
                {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88},
                {92, 11, 1, 0, 255, 128, 68, 23},
                {17, 24, 24, 27, 10, 34, 0, 1},
                {27, 10, 18, 23, 11, 24, 32, 28}
            };
            for (uint32_t j = 0; j < ARRAY_SIZE(rand_output); j++)
            {
                memset(expected_auth_value, 0, PROV_AUTH_LEN);
                rand_hw_rng_get_Expect(NULL, m_ctx.oob_size);
                rand_hw_rng_get_IgnoreArg_p_result();
                rand_hw_rng_get_ReturnMemThruPtr_p_result(rand_output[j], m_ctx.oob_size);
                prov_utils_generate_oob_data(&m_ctx, auth_value);
                static const char alphanumeric[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";
                for (uint32_t k = 0; k < m_ctx.oob_size; k++)
                {
                    expected_auth_value[k] = alphanumeric[rand_output[j][k] % 36];
                }
                TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_auth_value, auth_value, PROV_AUTH_LEN);
            }
        }

        /* Test invalid oob data */
        oob_method_and_action_pair_t invalid_pairs[] =
        {
            {NRF_MESH_PROV_OOB_METHOD_OUTPUT, 0xFF},
            {NRF_MESH_PROV_OOB_METHOD_INPUT,  0xFF},
            {NRF_MESH_PROV_OOB_METHOD_NONE,   0xFF},
            {NRF_MESH_PROV_OOB_METHOD_STATIC, 0xFF},
        };
        memset(expected_auth_value, 0, PROV_AUTH_LEN);
        for (uint32_t i = 0; i < ARRAY_SIZE(invalid_pairs); i++)
        {
            m_ctx.oob_method = (nrf_mesh_prov_oob_method_t) invalid_pairs[i].method;
            m_ctx.oob_action = invalid_pairs[i].action;
            prov_utils_generate_oob_data(&m_ctx, auth_value);
            TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_auth_value, auth_value, PROV_AUTH_LEN);
        }
    }
}

void test_confirmation_check(void)
{
    uint8_t confirmation_key[NRF_MESH_KEY_SIZE];
    memset(confirmation_key, 0xCF, NRF_MESH_KEY_SIZE);
    memset(m_ctx.confirmation_inputs, 0xAB, 17);
    memset(m_ctx.auth_value, 0xAD, 16);
    memset(m_ctx.peer_random, 0x59, 16);
    memset(m_ctx.peer_confirmation, 0x59, 16);
    uint8_t confirmation[PROV_CONFIRMATION_LEN];
    memset(confirmation, 0x58, 16); //WRONG VALUE, HAS TO MATCH m_ctx.peer_confirmation

    uint8_t random_and_auth[PROV_RANDOM_LEN + PROV_AUTH_LEN];
    memcpy(&random_and_auth[0], m_ctx.peer_random, PROV_RANDOM_LEN);
    memcpy(&random_and_auth[PROV_RANDOM_LEN], m_ctx.auth_value, PROV_AUTH_LEN);

    /* conf key generation */
    enc_k1_Expect(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            m_ctx.confirmation_salt,
            (const uint8_t *) "prck", /* Since the compiler stores identical strings together, this pointer will be the same */
            4,
            NULL);
    enc_k1_IgnoreArg_p_out();
    enc_k1_ReturnMemThruPtr_p_out(confirmation_key, 16);

    enc_aes_cmac_ExpectWithArray(confirmation_key,
            NRF_MESH_KEY_SIZE,
            random_and_auth,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            NULL,
            0);
    enc_aes_cmac_IgnoreArg_p_result();
    enc_aes_cmac_ReturnMemThruPtr_p_result(confirmation, PROV_CONFIRMATION_LEN);

    TEST_ASSERT_EQUAL(false, prov_utils_confirmation_check(&m_ctx));

    /************* Set to correct value *************/
    memset(confirmation, 0x59, 16);

    /* conf key generation */
    enc_k1_Expect(m_ctx.shared_secret,
            NRF_MESH_ECDH_SHARED_SECRET_SIZE,
            m_ctx.confirmation_salt,
            (const uint8_t *) "prck", /* Since the compiler stores identical strings together, this pointer will be the same */
            4,
            NULL);
    enc_k1_IgnoreArg_p_out();
    enc_k1_ReturnMemThruPtr_p_out(confirmation_key, 16);

    memcpy(&random_and_auth[0], m_ctx.peer_random, PROV_RANDOM_LEN);
    memcpy(&random_and_auth[PROV_RANDOM_LEN], m_ctx.auth_value, PROV_AUTH_LEN);

    enc_aes_cmac_ExpectWithArray(confirmation_key,
            NRF_MESH_KEY_SIZE,
            random_and_auth,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            PROV_RANDOM_LEN + PROV_AUTH_LEN,
            NULL,
            0);
    enc_aes_cmac_IgnoreArg_p_result();
    enc_aes_cmac_ReturnMemThruPtr_p_result(confirmation, PROV_CONFIRMATION_LEN);

    TEST_ASSERT_EQUAL(true, prov_utils_confirmation_check(&m_ctx));

    /************* Check rejection if there are identical random numbers *************/

    m_ctx.role = NRF_MESH_PROV_ROLE_PROVISIONER;
    memcpy(m_ctx.node_random, m_ctx.peer_random, 16);
    TEST_ASSERT_EQUAL(false, prov_utils_confirmation_check(&m_ctx));
}

void test_options(void)
{
    nrf_mesh_opt_t opt_out;
    memset(&opt_out, 0, sizeof(nrf_mesh_opt_t));
    nrf_mesh_opt_t opt_in;
    memset(&opt_in, 0, sizeof(nrf_mesh_opt_t));
    opt_in.len = 1;
    opt_in.opt.val = 1;
    /* invalid options for prov: */
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_PARAM, prov_utils_opt_set(NRF_MESH_OPT_NET_RELAY_ENABLE, &opt_in));
    TEST_ASSERT_EQUAL_HEX32(NRF_ERROR_INVALID_PARAM, prov_utils_opt_get(NRF_MESH_OPT_NET_RELAY_ENABLE, &opt_out));
    /* valid options: */
    TEST_ASSERT_EQUAL(false, prov_utils_use_ecdh_offloading());
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_utils_opt_set(NRF_MESH_OPT_PROV_ECDH_OFFLOADING, &opt_in));
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_utils_opt_get(NRF_MESH_OPT_PROV_ECDH_OFFLOADING, &opt_out));
    TEST_ASSERT_EQUAL(true, prov_utils_use_ecdh_offloading());
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&opt_in, &opt_out, sizeof(opt_in));
    opt_in.opt.val = 0;
    TEST_ASSERT_EQUAL_HEX32(NRF_SUCCESS, prov_utils_opt_set(NRF_MESH_OPT_PROV_ECDH_OFFLOADING, &opt_in));
    TEST_ASSERT_EQUAL(false, prov_utils_use_ecdh_offloading());
}

void test_is_alphanumeric(void)
{
    static const unsigned char alphanumeric[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    TEST_ASSERT_EQUAL(true,
                      prov_utils_auth_data_is_alphanumeric(
                          alphanumeric, sizeof(alphanumeric) - 1));


    bool valid_input_values[UINT8_MAX] = {0}; /* False */
    for (uint32_t i = 0; i < sizeof(alphanumeric) - 1; ++i)
    {
        valid_input_values[alphanumeric[i]] = true;
    }

    for (uint8_t i = 0; i < UINT8_MAX; ++i)
    {
        TEST_ASSERT_EQUAL(valid_input_values[i],
                          prov_utils_auth_data_is_alphanumeric(&i, 1));
    }
}

void test_is_number(void)
{
    uint32_t three_digits = 123;
    uint32_t eight_digits = 12345678;
    TEST_ASSERT_EQUAL(false, prov_utils_auth_data_is_valid_number((const uint8_t *) &three_digits, 2));
    TEST_ASSERT_EQUAL(false, prov_utils_auth_data_is_valid_number((const uint8_t *) &three_digits, 1));
    TEST_ASSERT_EQUAL(true, prov_utils_auth_data_is_valid_number((const uint8_t *) &three_digits, 3));
    TEST_ASSERT_EQUAL(true, prov_utils_auth_data_is_valid_number((const uint8_t *) &three_digits, 4));

    TEST_ASSERT_EQUAL(false, prov_utils_auth_data_is_valid_number((const uint8_t *) &eight_digits, 4));
    TEST_ASSERT_EQUAL(true, prov_utils_auth_data_is_valid_number((const uint8_t *) &eight_digits, 8));

}

void test_ecdh_deleter(void)
{
    mesh_config_entry_id_t entry_id = MESH_OPT_PROV_ECDH_OFFLOADING_EID;
    bool result = true;
    m_ecdh_offloading_params.callbacks.getter(entry_id, &result);
    TEST_ASSERT_FALSE(result);
    result = true;
    m_ecdh_offloading_params.callbacks.setter(entry_id, &result);
    m_ecdh_offloading_params.callbacks.getter(entry_id, &result);
    TEST_ASSERT_TRUE(result);
    m_ecdh_offloading_params.callbacks.deleter(entry_id);
    m_ecdh_offloading_params.callbacks.getter(entry_id, &result);
    TEST_ASSERT_FALSE(result);
}

