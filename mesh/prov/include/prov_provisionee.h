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

#ifndef PROV_PROVISIONEE_H__
#define PROV_PROVISIONEE_H__

#include <stdbool.h>
#include <stdint.h>

#include "nrf_mesh_prov.h"
#include "nrf_mesh.h"
#include "provisioning.h"
#include "nrf_mesh_prov_bearer.h"

/**
 * @defgroup PROV_PROVISIONEE Provisionee Support
 * @ingroup MESH_PROV
 * Module providing provisionee support.
 *
 * @dotfile provisionee_state_diagram.dot "Provisionee state machine diagram"
 * @{
 */

/**
 * Starts listening for provisioning links on the given bearer.
 * @param[in,out] p_ctx            Pointer to the provisioning context structure.
 * @param[in,out] p_bearer         Generic bearer interface to start listening for provisioning
 *                                 links on.
 * @param[in]     URI              Optional device URI string used as identifier in some other context. May be NULL.
 * @param[in]     oob_info_sources Known OOB information sources, see @ref NRF_MESH_PROV_OOB_INFO_SOURCES.
 *
 * @retval NRF_SUCCESS             The provisionee module was successfully initialized.
 * @retval NRF_ERROR_NOT_SUPPORTED The provisionee role is not supported in the application.
 * @retval NRF_ERROR_INVALID_STATE The given bearer was not in idle state.
 * @retval NRF_ERROR_INTERNAL      Something went wrong when initializing the substructures.
 */
uint32_t prov_provisionee_listen(nrf_mesh_prov_ctx_t * p_ctx, prov_bearer_t * p_bearer, const char * URI, uint16_t oob_info_sources);

/**
 * Provides authentication data to the provisioner module.
 *
 * @param[in,out] p_ctx  Pointer to the provisioning context structure.
 * @param[in]     p_data Pointer to the authentication data.
 * @param[in]     size   Size of the authentication data.
 *
 * @retval NRF_SUCCESS   The authentication data was accepted.
 */
uint32_t prov_provisionee_auth_data(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_data, uint8_t size);

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
uint32_t prov_provisionee_shared_secret(nrf_mesh_prov_ctx_t * p_ctx, const uint8_t * p_shared);

/** @} */

#endif

