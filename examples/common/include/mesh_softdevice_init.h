/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#ifndef MESH_SOFTDEVICE_INIT_H__
#define MESH_SOFTDEVICE_INIT_H__

#include <stdint.h>
#include "nrf_sdm.h"

/**
 * @defgroup MESH_SOFTDEVICE_INIT SoftDevice initialization helper module
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Application support module for performing a typical SoftDevice initialization.
 * @{
 */

/* This allows us to use the same typenames for all versions of the SoftDevice: */
#if defined(S110)
#define nrf_fault_handler_t softdevice_assertion_handler_t
#define nrf_clock_lf_cfg_t nrf_clock_lfclksrc_t
#endif

/**
 * Initialize the SoftDevice.
 *
 * @param[in]  lfc_cfg Low frequency clock configuration.
 *
 * @retval NRF_ERROR_INVALID_ADDR  Invalid or NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_STATE SoftDevice is already enabled, and the clock source and fault handler cannot be updated.
 * @retval NRF_ERROR_SDM_INCORRECT_INTERRUPT_CONFIGURATION SoftDevice interrupt is already enabled, or an enabled interrupt has an illegal priority level.
 * @retval NRF_ERROR_SDM_LFCLK_SOURCE_UNKNOWN Unknown low frequency clock source selected.
 * @retval NRF_SUCCESS SoftDevice was successfully initialized.
 */
uint32_t mesh_softdevice_init(nrf_clock_lf_cfg_t lfc_cfg);

/**
 * @}
 */

#endif
