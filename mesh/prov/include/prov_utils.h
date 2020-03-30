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

#ifndef PROV_UTILS_H__
#define PROV_UTILS_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_opt.h"
#include "nrf_mesh_config_core.h"
#include "provisioning.h"

/**
 * @defgroup PROV_UTILS Common Provisioning Functions
 * @ingroup MESH_PROV
 * This module provides functions common to both the provisioner and the provisionee.
 * @{
 */

/** Offset into the provisioning confirmation input array where the contents of the invite PDU is copied. */
#define PROV_CONFIRM_INPUTS_INVITE_OFFSET   (0)
/** Offset into the provisioning confirmation input array where the contents of the capabilities PDU is copied. */
#define PROV_CONFIRM_INPUTS_CAPS_OFFSET     (PROV_CONFIRM_INPUTS_INVITE_OFFSET + sizeof(prov_pdu_invite_t) - 1)
/** Offset into the provisioning confirmation input array where the contents of the start PDU is copied. */
#define PROV_CONFIRM_INPUTS_START_OFFSET    (PROV_CONFIRM_INPUTS_CAPS_OFFSET + sizeof(prov_pdu_caps_t) - 1)

/**
 * Sets provisioning options.
 *
 * @param[in] id    Option ID.
 * @param[in] p_opt Pointer to an option structure containing the new value of the option.
 *
 * @retval NRF_SUCCESS             The option was successfully set.
 * @retval NRF_ERROR_INVALID_PARAM The option ID was not recognzied.
 */
uint32_t prov_utils_opt_set(nrf_mesh_opt_id_t id, const nrf_mesh_opt_t * p_opt);

/**
 * Gets the value of provisioning options.
 *
 * @param[in]  id Option ID.
 * @param[out] p_opt Pointer to an option structure where the value of the option will be returned.
 *
 * @retval NRF_SUCCESS             The value of the option was successfully retrieved.
 * @retval NRF_ERROR_INVALID_PARAM The option ID was not recognized.
 */
uint32_t prov_utils_opt_get(nrf_mesh_opt_id_t id, nrf_mesh_opt_t * p_opt);

/**
 * Checks if ECC should be offloaded to the application.
 *
 * @retval true The ECC is offloaded to the application.
 * @retval false the ECC is done internally in the stack.
 */
bool prov_utils_use_ecdh_offloading(void);

/**
 * Generate confirmation salt, confirmation value and random value.
 *
 * @param[in]  p_ctx               Pointer to the context structure.
 * @param[out] p_confirmation_salt Pointer to a location in which to store the confirmation salt.
 * @param[out] p_confirmation      Pointer to a location in which to store the confirmation.
 * @param[out] p_random            Pointer to a location in which to store the random.
 */
void prov_utils_authentication_values_derive(const nrf_mesh_prov_ctx_t * p_ctx,
        uint8_t * p_confirmation_salt,
        uint8_t * p_confirmation,
        uint8_t * p_random);

/**
 * Generates a private/public keypair for the device.
 *
 * @param[out] p_public       Pointer to where the public key should be stored.
 * @param[out] p_private      Pointer to where the private key should be stored.
 *
 * @retval NRF_SUCCESS             The keys were successfully generated.
 * @retval NRF_ERROR_INTERNAL      An error occured while generating the keys.
 * @retval NRF_ERROR_NOT_SUPPORTED The mesh stack was compiled without uECC support,
 *                                 making the required functionality unavailable.
 */
uint32_t prov_utils_keys_generate(uint8_t * p_public, uint8_t * p_private);

/**
 * Derives the encryption keys used in the provisioning session.
 *
 * @param[in] p_ctx            Pointer to the context structure.
 * @param[out] p_session_key   Pointer to a location in which to store the session key.
 * @param[out] p_session_nonce Pointer to a location in which to store the session nonce.
 * @param[out] p_device_key    Pointer to a location in which to store the device key.
 */
void prov_utils_derive_keys(const nrf_mesh_prov_ctx_t * p_ctx,
        uint8_t * p_session_key,
        uint8_t * p_session_nonce,
        uint8_t * p_device_key);

/**
 * Calculates the shared secret for the two nodes, using ECDH.
 *
 * @param[in] p_ctx                Pointer to the context structure.
 * @param[out] p_shared_secret     Pointer to a location in which to store the shared secret.
 *
 * @retval NRF_SUCCESS             The shared secret was successfully derived.
 * @retval NRF_ERROR_INTERNAL      The shared secret could not be calculated; this is likely to happen if the public
 *                                 key received from the peer node is not valid.
 * @retval NRF_ERROR_NOT_SUPPORTED The mesh stack was compiled without uECC support,
 *                                 making the required functionality unavailable.
 */
uint32_t prov_utils_calculate_shared_secret(const nrf_mesh_prov_ctx_t * p_ctx, uint8_t * p_shared_secret);

/**
 * Generates data for OOB authentication.
 *
 * @param[in]  p_ctx        Pointer to the context structure.
 * @param[out] p_auth_value Pointer to a location in which to store the auth value.
 */
void prov_utils_generate_oob_data(const nrf_mesh_prov_ctx_t * p_ctx, uint8_t * p_auth_value);

/**
 * Checks the confirmation values of the provisionee and the provisioner.
 *
 * @param[in] p_ctx Pointer to the context structure.
 *
 * @retval true  The confirmation values are matching.
 * @retval false The confirmation values are not matching.
 */
bool prov_utils_confirmation_check(const nrf_mesh_prov_ctx_t * p_ctx);

/**
 * Checks whether the given data is alphanumeric.
 *
 * @note Valid ASCII characters are '0'-'9' (codes 0x30-0x39) and 'A'-'Z' (codes 0x41-0x5A). See @tagMeshSp section 5.4.2.2.
 *
 * @retval true     The input data is alphanumeric.
 * @retval false    The input is not alphanumeric.
 */
bool prov_utils_auth_data_is_alphanumeric(const uint8_t * p_data, uint8_t size);

/**
 * Check whether a number has less than @p size number of digits.
 *
 * @param[in] p_data Pointer to a 4-byte number.
 * @param[in] size   Number of digits in the number.
 *
 * @retval true      The input data has less than @p size digits.
 * @retval false     The input data does not have less than @p size digits.
 */
bool prov_utils_auth_data_is_valid_number(const uint8_t * p_data, uint8_t size);

/**
 * Checks whether a received PDU is valid in the given state.
 *
 * @param[in] role     Provisioner or provisionee role.
 * @param[in] state    Current state of the provisioning context.
 * @param[in] pdu_type PDU type.
 *
 * @retval true        The PDU is expected in the given state.
 * @retval false       The PDU is unexpected in the given state.
 */
bool prov_utils_is_valid_pdu(nrf_mesh_prov_role_t role, nrf_mesh_prov_state_t state, prov_pdu_type_t pdu_type);

/**
 * Checks whether a public key is valid.
 *
 * @param[in] p_public_key Pointer to a public key array. Assumed to be @ref
 *                         NRF_MESH_PROV_PUBKEY_SIZE.
 *
 * @retval true            The public key is valid.
 * @retval false           The public key is invalid.
 */
bool prov_utils_is_valid_public_key(const uint8_t * p_public_key);

/** @} */

#endif
