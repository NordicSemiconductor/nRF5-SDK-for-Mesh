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

#ifndef PROV_PROVISIONER_H__
#define PROV_PROVISIONER_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf_mesh_prov.h"
#include "provisioning.h"
#include "nrf_mesh_prov_bearer.h"

/**
 * @defgroup PROV_PROVISIONER Provisioner Support
 * @ingroup MESH_PROV
 * Module providing provisioner support.
 *
 * @dotfile provisioner_state_diagram.dot "Provisioner state machine diagram"
 * @{
 */

/**
 * Starts provisioning of a device.
 *
 * @param[in,out] p_ctx                 Pointer to the provisioning context structure.
 * @param[in]     p_uuid                Pointer to the UUID for the device that is to be provisioned.
 * @param[in]     attention_duration_s  Time in seconds during which the device will identify itself using any means it can.
 * @param[in]     p_data                Pointer to the provisioning data structure for the device
 *
 * @retval NRF_SUCCESS   The provisioning process was successfully started for the specified device.
 */
uint32_t prov_provisioner_provision(nrf_mesh_prov_ctx_t * p_ctx,
                                    const uint8_t * p_uuid,
                                    uint8_t attention_duration_s,
                                    const nrf_mesh_prov_provisioning_data_t * p_data);

/**
 * Specifies which out-of-band mechanism to use for authentication.
 *
 * @param[in,out] p_ctx  Pointer to the provisioning context structure.
 * @param[in]     method Which OOB authentication method to use.
 * @param[in]     action Which action of the chosen OOB method to use.
 *                       The appropriate action shall be chosen from @ref nrf_mesh_prov_input_action_t
 *                       or @ref nrf_mesh_prov_output_action_t subsets.
 * @param[in]     size   Size of the OOB authentication data. This parameter will be ignored when the
 *                       method is NRF_MESH_PROV_OOB_METHOD_STATIC.
 *
 * @retval NRF_SUCCESS The OOB authentication method was successfully chosen.
 * @retval NRF_ERROR_INVALID_STATE The provisioning context was not in the correct state for this function to be used.
 */
uint32_t prov_provisioner_oob_use(nrf_mesh_prov_ctx_t * p_ctx,
                                  nrf_mesh_prov_oob_method_t method,
                                  uint8_t action,
                                  uint8_t size);

/**
 * Provides authentication data to the provisioner module.
 *
 * @param[in,out] p_ctx  Pointer to the provisioning context structure.
 * @param[in]     p_data Pointer to the authentication data.
 * @param[in]     size   Size of the authentication data.
 *
 * @retval NRF_SUCCESS              The authentication data was accepted.
 * @retval NRF_ERROR_INVALID_STATE  Authentication data was not expected at this point.
 * @retval NRF_ERROR_INVALID_LENGTH The size of the authentication data array was incorrect.
 */
uint32_t prov_provisioner_auth_data(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size);

/**
 * Provides the ECDH shared secret to the provisioner module.
 *
 * @param[in,out] p_ctx    Pointer to the provisioning context structure.
 * @param[in]     p_shared Pointer to the shared key from the ECDH calculation.
 *
 * @retval NRF_SUCCESS             The ECDH shared secret from the application was accepted.
 * @retval NRF_ERROR_INVALID_STATE A shared secret was not expected at this point.
 * @retval NRF_ERROR_NULL          The @c p_shared parameter was @c NULL.
 */
uint32_t prov_provisioner_shared_secret(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared);

/**
 * Provides an out-of-band public key to the provisioner module.
 *
 * @param[in,out] p_ctx Pointer to the provisioning context structure.
 * @param[in] p_key Pointer to the start of the array containing the public key of the peer node.
 *
 * @retval NRF_SUCCESS             The public key was valid and accepted.
 * @retval NRF_ERROR_INVALID_PARAM The public key was not a valid key.
 * @retval NRF_ERROR_INVALID_STATE A public key was not expected at this point.
 */
uint32_t prov_provisioner_oob_pubkey(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_key);

/** @} */

#endif

