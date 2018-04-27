/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#ifndef NRF_MESH_SDK_H
#define NRF_MESH_SDK_H

/**
 * @internal
 * @defgroup NRF_MESH_SDK SoftDevice and setup abstraction module for example applications
 *
 * Application support module abstracting the interaction with various SoftDevice versions
 * targetted in the example applications.
 *
 * The Mesh SDK-module aims to reduce the amount of boiler plate code required by the examples, to
 * avoid taking focus away from the mesh-related code.
 * @{
 */

#include "nrf_soc.h"
#if defined(S130) || defined(S132) || defined(S140)
#include "nrf_nvic.h"
#endif
#include "nrf_sdm.h"
#include "ble.h"
#include "boards.h"

#include "nrf_mesh.h"
#include "log.h"

#define ERROR_CHECK(__error_code)                                       \
    do                                                                  \
    {                                                                   \
        const uint32_t __local_code = (__error_code);                  \
        if (__local_code != NRF_SUCCESS)                                \
        {                                                               \
            app_error_handler(__local_code, __LINE__, (const uint8_t *) __FILE__); \
        }                                                               \
    } while (0)

void app_error_handler(uint32_t error_code, uint32_t line_number, const uint8_t * filename);

void mesh_assert_handler(uint32_t pc);

#if defined(S130) || defined(S132) || defined(S140)
void softdevice_assert_handler(uint32_t id, uint32_t pc, uint32_t info);
uint32_t mesh_softdevice_setup(nrf_clock_lf_cfg_t lfc_cfg);
#elif defined(S110)
void softdevice_assert_handler(uint32_t pc, uint16_t line_number, const uint8_t * p_filename);
uint32_t mesh_softdevice_setup(nrf_clock_lfclksrc_t lfclksrc);
#endif
void mesh_core_setup(void);

/** @} */

#endif
