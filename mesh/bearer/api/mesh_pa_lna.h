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
#ifndef MESH_PA_LNA_H__
#define MESH_PA_LNA_H__

#include "nrf_mesh_config_bearer.h"
#include "ble.h"
/**
 * @defgroup MESH_PA_LNA Mesh PA/LNA interface
 * @ingroup MESH_API_GROUP_BEARER
 * Controls external PA/LNA support for the mesh radio operation.
 *
 * The mesh PA/LNA module enables external power amplifiers and low noise amplifiers by setting
 * user configured GPIO-pins during radio operation.
 *
 * The PA/LNA is triggered some time before the radio operation starts, to allow the amplifiers
 * to stabilize. The timing can be configured at compile time by changing
 * @ref MESH_PA_SETUP_TIME_US and @ref MESH_LNA_SETUP_TIME_US. Note that the setup time cannot be
 * longer than the radio rampup time (by default 40us on nRF52 and 140us on nRF51).
 * @{
 */


/**
 * Configuration parameters for gpiote-based setup of PA/LNA. Identical to the Softdevice
 * parameters.
 */
typedef ble_common_opt_pa_lna_t mesh_pa_lna_gpiote_params_t;


/**
 * Enables GPIOTE controlled PA/LNA triggering.
 *
 * Parameter requirements:
 * - The PPI channels must not collide with the mesh timer channel range (8-11).
 * - The PPI channels cannot be the same for set and clear.
 * - The PPI channels must be valid for the given chip configuration.
 * - The GPIOTE channel ID must be valid for the given chip configuration.
 * - The GPIO pins must be valid for the given chip configuration.
 *
 * @param[in] p_params Configuration parameters.
 *
 * @retval NRF_SUCCESS The GPIOTE controlled PA/LNA triggering was successfully enabled.
 * @retval NRF_ERROR_NULL The parameter pointer was NULL.
 * @retval NRF_ERROR_INVALID_PARAMS One or more of the parameters broke the parameter requirements.
 */
uint32_t mesh_pa_lna_gpiote_enable(const mesh_pa_lna_gpiote_params_t * p_params);

/**
 * Disables PA/LNA triggering.
 *
 * Immediately stops any running GPIO signal.
 */
void mesh_pa_lna_gpiote_disable(void);

/**
 * Retrieves the current PA/LNA parameters.
 *
 * @returns The current PA/LNA parameters or NULL if disabled.
 */
const mesh_pa_lna_gpiote_params_t * mesh_pa_lna_gpiote_params_get(void);


/** @} */

#endif /* MESH_PA_LNA_H__ */
