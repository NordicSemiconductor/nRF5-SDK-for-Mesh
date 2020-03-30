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
#ifndef MESH_OPT_PROV_H__
#define MESH_OPT_PROV_H__

#include "mesh_opt.h"

/**
 * @defgroup MESH_OPT_PROV Provisioning runtime configuration options
 * @ingroup MESH_OPT
 * Runtime configuration for Mesh provisioning functionality
 * @{
 */

/** ECDH offloading entry ID */
#define MESH_OPT_PROV_ECDH_OFFLOADING_EID   MESH_OPT_CORE_ID(MESH_OPT_PROV_ID_START + 0)

/**
 * Enables/disables ECDH offloading for the provisioning procedure
 *
 * @param[in] enabled Whether the ECDH offloading is enabled.
 *
 * @retval NRF_SUCCESS Successfully set.
 * @retval NRF_ERROR_NOT_FOUND ECDH offloading not available.
 */
uint32_t mesh_opt_prov_ecdh_offloading_set(bool enabled);

/**
 * Checks whether ECDH offloading for the provisioning procedure is enabled.
 *
 * @param[in,out] p_enabled Returns whether the ECDH offloading is enabled.
 *
 * @retval NRF_SUCCESS Successfully populated return parameter.
 * @retval NRF_ERROR_NULL Got NULL parameter.
 * @retval NRF_ERROR_NOT_FOUND ECDH offloading status not available.
 */
uint32_t mesh_opt_prov_ecdh_offloading_get(bool * p_enabled);

/** @} */

#endif /* MESH_OPT_PROV_H__ */
