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
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "prov_utils.h"

#include "utils.h"
#include "enc.h"
#include "rand.h"
#include "uECC.h"
#include "mesh_config.h"
#include "mesh_opt_prov.h"

#define CONFIRMATION_KEY_INFO        (const uint8_t *) "prck"
#define CONFIRMATION_KEY_INFO_LENGTH 4
#define SESSION_KEY_INFO             (const uint8_t *) "prsk"
#define SESSION_KEY_INFO_SIZE        4
#define DATA_NONCE_INFO              (const uint8_t *) "prsn"
#define DATA_NONCE_INFO_SIZE         4
#define DEVICE_KEY_INFO              (const uint8_t *) "prdk"
#define DEVICE_KEY_INFO_SIZE         4

#define CONFIRMATION_INPUTS_SIZE     (PROV_CONFIRMATION_INPUT_LEN + 2 * NRF_MESH_PROV_PUBKEY_SIZE)

/* Used to optimize lookup of valid PDUs. */
#define PDU_VALID_PROVISIONER 0x40
#define PDU_VALID_PROVISIONEE 0x80

/* We reserve the two upper bits for PDU lookup. */
NRF_MESH_STATIC_ASSERT(PROV_PDU_TYPE_COUNT <= 0x3f);
NRF_MESH_STATIC_ASSERT(NRF_MESH_PROV_OOB_SIZE_MAX == 8);

/* Parameter digits is somewhere from 1 to @ref NRF_MESH_PROV_OOB_SIZE_MAX.
 * We're storing the largest number permitted for each value in a lookup
 * table:
 */
static const uint32_t m_numeric_max[] =
{
    0, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000
};

static bool m_enabled;

/*****************************************************************************
 * Mesh Config wrapper functions
 *****************************************************************************/
static uint32_t ecdh_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    m_enabled = *(bool *)p_entry;

    return NRF_SUCCESS;
}

static void ecdh_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    *(bool *)p_entry = m_enabled;
}

static void ecdh_deleter(mesh_config_entry_id_t entry_id)
{
    m_enabled = false;
}

MESH_CONFIG_ENTRY(ecdh_offloading,
                  MESH_OPT_PROV_ECDH_OFFLOADING_EID,
                  1,
                  sizeof(m_enabled),
                  ecdh_setter,
                  ecdh_getter,
                  ecdh_deleter,
                  true);

static void create_confirmation_salt(const nrf_mesh_prov_ctx_t * p_ctx, uint8_t * p_confirmation_salt)
{
    /* ConfirmationInputs = AES-CMAC(AES-CMAC(
     *      ProvisioningInvitePDUValue || ProvisioningCapabilitiesPacketValue || ProvisioningStartPDUValue // Already stored in p_ctx->confirmation_inputs
     *      || PublicKeyProvisioner || PublicKeyDevice)
     */
    uint8_t confirmation_inputs[CONFIRMATION_INPUTS_SIZE];

    memcpy(confirmation_inputs, p_ctx->confirmation_inputs, PROV_CONFIRMATION_INPUT_LEN);
    uint8_t target_index = PROV_CONFIRMATION_INPUT_LEN;

    if (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        memcpy(&confirmation_inputs[target_index], p_ctx->p_public_key, NRF_MESH_PROV_PUBKEY_SIZE);
        target_index += NRF_MESH_PROV_PUBKEY_SIZE;
        memcpy(&confirmation_inputs[target_index], p_ctx->peer_public_key, NRF_MESH_PROV_PUBKEY_SIZE);
    }
    else
    {
        memcpy(&confirmation_inputs[target_index], p_ctx->peer_public_key, NRF_MESH_PROV_PUBKEY_SIZE);
        target_index += NRF_MESH_PROV_PUBKEY_SIZE;
        memcpy(&confirmation_inputs[target_index], p_ctx->p_public_key, NRF_MESH_PROV_PUBKEY_SIZE);
    }
    enc_s1(confirmation_inputs, CONFIRMATION_INPUTS_SIZE, p_confirmation_salt);
}

static void oob_gen_count(uint8_t * p_auth_value, uint8_t oob_size)
{
    uint8_t count;
    rand_hw_rng_get(&count, 1);
    /* @tagMeshSp section 5.4.2.2: random integer number between 1
     * and the [oob size] inclusive */
    count = (count % oob_size) + 1;
    p_auth_value[PROV_AUTH_LEN - 1] = count;
}

static void oob_gen_numeric(uint8_t * p_auth_value, uint8_t digits)
{
    uint32_t number;
    rand_hw_rng_get((uint8_t *) &number, sizeof(number));
    /* Big endian at end of auth value: */
    number = LE2BE32((number % m_numeric_max[digits]));
    memcpy(&p_auth_value[PROV_AUTH_LEN - sizeof(number)], &number, sizeof(number));
}

static void oob_gen_alphanumeric(uint8_t * p_auth_value, uint8_t characters)
{
    /* @tagMeshSp section 5.4.2.2: valid ASCII character codes are 0x30-0x39 and 0x41-0x5A */
    static const char alphanumeric[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";

    uint8_t indexes[NRF_MESH_PROV_OOB_SIZE_MAX];
    rand_hw_rng_get(indexes, characters);
    for (uint32_t i = 0; i < characters; i++)
    {
        p_auth_value[i] = alphanumeric[indexes[i] % (sizeof(alphanumeric) - 1)];
    }
}


uint32_t prov_utils_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * p_opt)
{
    if (id == NRF_MESH_OPT_PROV_ECDH_OFFLOADING)
    {
        return mesh_opt_prov_ecdh_offloading_set((bool) p_opt->opt.val);
    }

    return NRF_ERROR_INVALID_PARAM;
}

uint32_t prov_utils_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * p_opt)
{
    if (id == NRF_MESH_OPT_PROV_ECDH_OFFLOADING)
    {
        p_opt->len = 1;
        return mesh_opt_prov_ecdh_offloading_get((bool *) &p_opt->opt.val);
    }

    return NRF_ERROR_INVALID_PARAM;
}

bool prov_utils_use_ecdh_offloading(void)
{
    bool retval;
    NRF_MESH_ERROR_CHECK(mesh_opt_prov_ecdh_offloading_get(&retval));
    return retval;
}

void prov_utils_authentication_values_derive(const nrf_mesh_prov_ctx_t * p_ctx,
        uint8_t * p_confirmation_salt,
        uint8_t * p_confirmation,
        uint8_t * p_random)
{
    create_confirmation_salt(p_ctx, p_confirmation_salt);

    uint8_t confirmation_key[NRF_MESH_KEY_SIZE];

    /* ConfirmationKey = k1(ECDHSecret, ConfirmationSalt, "prck") */
    enc_k1(p_ctx->shared_secret,
           NRF_MESH_ECDH_SHARED_SECRET_SIZE,
           p_confirmation_salt,
           CONFIRMATION_KEY_INFO, CONFIRMATION_KEY_INFO_LENGTH,
           confirmation_key);

    rand_hw_rng_get(p_random, PROV_RANDOM_LEN);

    uint8_t random_and_auth[PROV_RANDOM_LEN + PROV_AUTH_LEN];
    memcpy(&random_and_auth[0], p_random, PROV_RANDOM_LEN);
    memcpy(&random_and_auth[PROV_RANDOM_LEN], p_ctx->auth_value, PROV_AUTH_LEN);

    /* Confirmation value = AES-CMAC(ConfirmationKey, LocalRandom || AuthValue) */
    enc_aes_cmac(confirmation_key,
                 random_and_auth,
                 PROV_RANDOM_LEN + PROV_AUTH_LEN,
                 p_confirmation);
}

uint32_t prov_utils_keys_generate(uint8_t * p_public, uint8_t * p_private)
{
#if NRF_MESH_UECC_ENABLE
    return uECC_make_key(p_public, p_private, uECC_secp256r1()) == 1 ? NRF_SUCCESS : NRF_ERROR_INTERNAL;
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}

void prov_utils_derive_keys(const nrf_mesh_prov_ctx_t * p_ctx,
        uint8_t * p_session_key,
        uint8_t * p_session_nonce,
        uint8_t * p_device_key)
{

    /* ProvisioningSalt = s1(ECDHSecret, ConfirmationSalt || RandomProvisioner || RandomDevice) */
    uint8_t provisioning_salt_data[PROV_SALT_LEN + PROV_RANDOM_LEN + PROV_RANDOM_LEN];
    memcpy(&provisioning_salt_data[0], p_ctx->confirmation_salt, PROV_SALT_LEN);
    if (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        memcpy(&provisioning_salt_data[PROV_SALT_LEN], p_ctx->node_random, PROV_RANDOM_LEN);
        memcpy(&provisioning_salt_data[PROV_SALT_LEN + PROV_RANDOM_LEN], p_ctx->peer_random, PROV_RANDOM_LEN);
    }
    else
    {
        memcpy(&provisioning_salt_data[PROV_SALT_LEN], p_ctx->peer_random, PROV_RANDOM_LEN);
        memcpy(&provisioning_salt_data[PROV_SALT_LEN + PROV_RANDOM_LEN], p_ctx->node_random, PROV_RANDOM_LEN);
    }

    uint8_t provisioning_salt[PROV_SALT_LEN];
    enc_s1(provisioning_salt_data,
           PROV_SALT_LEN + PROV_RANDOM_LEN + PROV_RANDOM_LEN,
           provisioning_salt);

    /* SessionKey = k1(ECDHSecret, ProvisioningSalt, "prsk") */
    enc_k1(p_ctx->shared_secret, NRF_MESH_ECDH_SHARED_SECRET_SIZE,
           provisioning_salt,
           SESSION_KEY_INFO, SESSION_KEY_INFO_SIZE,
           p_session_key);

    uint8_t session_nonce_temp[NRF_MESH_KEY_SIZE];
    /* Nonce (for enrypting the data) = k1(ECDHSecret, ProvisioningSalt, "prsn") */
    enc_k1(p_ctx->shared_secret, NRF_MESH_ECDH_SHARED_SECRET_SIZE,
           provisioning_salt,
           DATA_NONCE_INFO, DATA_NONCE_INFO_SIZE,
           session_nonce_temp);
    /* The nonce is the 13 LSB of the generated key */
    memcpy(p_session_nonce, &session_nonce_temp[NRF_MESH_KEY_SIZE - PROV_NONCE_LEN], PROV_NONCE_LEN);

    /* DeviceKey = k1(ECDHSecret, ProvisioningSalt, "prdk") */
    enc_k1(p_ctx->shared_secret, NRF_MESH_ECDH_SHARED_SECRET_SIZE,
           provisioning_salt,
           DEVICE_KEY_INFO, DEVICE_KEY_INFO_SIZE,
           p_device_key);
}

bool prov_utils_is_valid_public_key(const uint8_t * p_public_key)
{
    return uECC_valid_public_key(p_public_key, uECC_secp256r1());
}

uint32_t prov_utils_calculate_shared_secret(const nrf_mesh_prov_ctx_t * p_ctx, uint8_t * p_shared_secret)
{
#if NRF_MESH_UECC_ENABLE
    /* We should have validated the public key before this point. */
    NRF_MESH_ASSERT_DEBUG(prov_utils_is_valid_public_key(p_ctx->peer_public_key));

    if (!uECC_shared_secret(p_ctx->peer_public_key, p_ctx->p_private_key, p_shared_secret, uECC_secp256r1()))
    {
        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}

void prov_utils_generate_oob_data(const nrf_mesh_prov_ctx_t * p_ctx, uint8_t * p_auth_value)
{

    NRF_MESH_ASSERT(p_ctx->oob_size > 0 && p_ctx->oob_size <= NRF_MESH_PROV_OOB_SIZE_MAX);
    memset(p_auth_value, 0, PROV_AUTH_LEN);
    switch (p_ctx->oob_method)
    {
        case NRF_MESH_PROV_OOB_METHOD_OUTPUT:
            switch ((nrf_mesh_prov_output_action_t) p_ctx->oob_action)
            {
                case NRF_MESH_PROV_OUTPUT_ACTION_BLINK:
                case NRF_MESH_PROV_OUTPUT_ACTION_BEEP:
                case NRF_MESH_PROV_OUTPUT_ACTION_VIBRATE:
                    oob_gen_count(p_auth_value, p_ctx->oob_size);
                    break;
                case NRF_MESH_PROV_OUTPUT_ACTION_DISPLAY_NUMERIC:
                    oob_gen_numeric(p_auth_value, p_ctx->oob_size);
                    break;
                case NRF_MESH_PROV_OUTPUT_ACTION_ALPHANUMERIC:
                    oob_gen_alphanumeric(p_auth_value, p_ctx->oob_size);
                    break;
                default:
                    break;
            }
            break;
        case NRF_MESH_PROV_OOB_METHOD_INPUT:
            switch ((nrf_mesh_prov_input_action_t) p_ctx->oob_action)
            {
                case NRF_MESH_PROV_INPUT_ACTION_PUSH:
                case NRF_MESH_PROV_INPUT_ACTION_TWIST:
                    oob_gen_count(p_auth_value, p_ctx->oob_size);
                    break;
                case NRF_MESH_PROV_INPUT_ACTION_ENTER_NUMBER:
                    oob_gen_numeric(p_auth_value, p_ctx->oob_size);
                    break;
                case NRF_MESH_PROV_INPUT_ACTION_ENTER_STRING:
                    oob_gen_alphanumeric(p_auth_value, p_ctx->oob_size);
                    break;
                default:
                    break;
            }
            break;
        default:
            /* Other OOB methods aren't generated runtime. */
            break;
    }
}

bool prov_utils_confirmation_check(const nrf_mesh_prov_ctx_t * p_ctx)
{
    uint8_t confirmation_key[NRF_MESH_KEY_SIZE];

    if (p_ctx->role == NRF_MESH_PROV_ROLE_PROVISIONER &&
        memcmp(p_ctx->node_random, p_ctx->peer_random, PROV_RANDOM_LEN) == 0)
    {
        return false;
    }

    /* ConfirmationKey = k1(ECDHSecret, ConfirmationSalt, "prck") */
    enc_k1(p_ctx->shared_secret,
           NRF_MESH_ECDH_SHARED_SECRET_SIZE,
           p_ctx->confirmation_salt,
           CONFIRMATION_KEY_INFO, CONFIRMATION_KEY_INFO_LENGTH,
           confirmation_key);

    uint8_t random_and_auth[PROV_RANDOM_LEN + PROV_AUTH_LEN];
    memcpy(&random_and_auth[0], p_ctx->peer_random, PROV_RANDOM_LEN);
    memcpy(&random_and_auth[PROV_RANDOM_LEN], p_ctx->auth_value, PROV_AUTH_LEN);

    /* Confirmation value = AES-CMAC(ConfirmationKey, LocalRandom || AuthValue) */
    uint8_t confirmation[PROV_CONFIRMATION_LEN];
    enc_aes_cmac(confirmation_key,
                 random_and_auth,
                 PROV_RANDOM_LEN + PROV_AUTH_LEN,
                 confirmation);

    return memcmp(confirmation, p_ctx->peer_confirmation, sizeof(confirmation)) == 0;
}

bool prov_utils_auth_data_is_alphanumeric(const uint8_t * p_data, uint8_t size)
{
    for (uint8_t i = 0; i < size; ++i)
    {
        if (!(IS_IN_RANGE(p_data[i], (uint8_t) '0', (uint8_t) '9') ||
              IS_IN_RANGE(p_data[i], (uint8_t) 'A', (uint8_t) 'Z')))
        {
            return false;
        }
    }
    return true;
}

bool prov_utils_auth_data_is_valid_number(const uint8_t * p_data, uint8_t size)
{
    NRF_MESH_ASSERT_DEBUG(size <= NRF_MESH_PROV_OOB_SIZE_MAX);
    uint32_t number;
    memcpy(&number, p_data, sizeof(uint32_t));
    return (number < m_numeric_max[size]);
}

uint32_t mesh_opt_prov_ecdh_offloading_set(bool enabled)
{
    return mesh_config_entry_set(MESH_OPT_PROV_ECDH_OFFLOADING_EID, &enabled);
}

uint32_t mesh_opt_prov_ecdh_offloading_get(bool * p_enabled)
{
    return mesh_config_entry_get(MESH_OPT_PROV_ECDH_OFFLOADING_EID, p_enabled);
}

bool prov_utils_is_valid_pdu(nrf_mesh_prov_role_t role, nrf_mesh_prov_state_t state, prov_pdu_type_t pdu_type)
{
    static const uint8_t pdu_lookup[] = {
        /* NRF_MESH_PROV_STATE_IDLE                   */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_LINK              */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_INVITE                 */ PROV_PDU_TYPE_INVITE         | PDU_VALID_PROVISIONEE,
        /* NRF_MESH_PROV_STATE_WAIT_CAPS              */ PROV_PDU_TYPE_CAPABILITIES   | PDU_VALID_PROVISIONER,
        /* NRF_MESH_PROV_STATE_WAIT_CAPS_CONFIRM      */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_START             */ PROV_PDU_TYPE_START          | PDU_VALID_PROVISIONEE,
        /* NRF_MESH_PROV_STATE_WAIT_START_ACK         */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_PUB_KEY_ACK       */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_PUB_KEY           */ PROV_PDU_TYPE_PUBLIC_KEY     | PDU_VALID_PROVISIONEE | PDU_VALID_PROVISIONER,
        /* NRF_MESH_PROV_STATE_WAIT_OOB_PUB_KEY       */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_EXTERNAL_ECDH     */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_OOB_INPUT         */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_OOB_STATIC        */ PROV_PDU_TYPE_CONFIRMATION   | PDU_VALID_PROVISIONEE,
        /* NRF_MESH_PROV_STATE_WAIT_OOB_STATIC_C_RCVD */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_CONFIRMATION_ACK  */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_WAIT_CONFIRMATION      */ PROV_PDU_TYPE_CONFIRMATION   | PDU_VALID_PROVISIONEE | PDU_VALID_PROVISIONER,
        /* NRF_MESH_PROV_STATE_WAIT_INPUT_COMPLETE    */ PROV_PDU_TYPE_INPUT_COMPLETE | PDU_VALID_PROVISIONER,
        /* NRF_MESH_PROV_STATE_WAIT_RANDOM            */ PROV_PDU_TYPE_RANDOM         | PDU_VALID_PROVISIONEE | PDU_VALID_PROVISIONER,
        /* NRF_MESH_PROV_STATE_WAIT_DATA              */ PROV_PDU_TYPE_DATA           | PDU_VALID_PROVISIONEE,
        /* NRF_MESH_PROV_STATE_WAIT_COMPLETE          */ PROV_PDU_TYPE_COMPLETE       | PDU_VALID_PROVISIONER,
        /* NRF_MESH_PROV_STATE_COMPLETE               */ PROV_PDU_TYPE_INVALID,
        /* NRF_MESH_PROV_STATE_FAILED                 */ PROV_PDU_TYPE_INVALID,
    };

    if (role == NRF_MESH_PROV_ROLE_PROVISIONER)
    {
        return ((pdu_lookup[state] & ~PDU_VALID_PROVISIONEE) == (pdu_type | PDU_VALID_PROVISIONER) ||
                pdu_type == PROV_PDU_TYPE_FAILED);
    }
    else
    {
        return ((pdu_lookup[state] & ~PDU_VALID_PROVISIONER) == (pdu_type | PDU_VALID_PROVISIONEE));
    }
}
