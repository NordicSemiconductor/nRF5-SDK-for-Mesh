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

#ifndef MESH_APP_UTILS_H__
#define MESH_APP_UTILS_H__

#include <stdint.h>
#include "toolchain.h"
#if defined(S130) || defined(S132) || defined(S140)
#include "nrf_nvic.h"
#elif defined(S110)
#include "nrf_soc.h"
#endif

/**
 * @defgroup MESH_APP_UTILS Mesh app utility functions
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 * Utilities for use in Mesh applications and examples.
 * @{
 */

extern void app_error_handler(uint32_t error_code, uint32_t line_number, const uint8_t * filename);

#define ERROR_CHECK(__error_code)                                                  \
    do                                                                             \
    {                                                                              \
        const uint32_t __local_code = (__error_code);                              \
        if (__local_code != NRF_SUCCESS)                                           \
        {                                                                          \
            app_error_handler(__local_code, __LINE__, (const uint8_t *) __FILE__); \
        }                                                                          \
    } while (0)

#define RETURN_ON_ERROR(status)    \
    do {                           \
        uint32_t result = status;  \
        if (result != NRF_SUCCESS) \
        {                          \
            return result;         \
        }                          \
    } while (0);

#if defined(S130) || defined(S132) || defined(S140) || defined(S112) || defined(S113)
#include "nrf_nvic.h"
extern nrf_nvic_state_t nrf_nvic_state;
#endif



/**
 * Prints the given UUID in a readable format.
 *
 * This function will not print anything if NULL pointer is provided as an input parameter.
 *
 * @param[in]  p_uuid           Pointer to 16-byte array that represents a UUID.
 */
void mesh_app_uuid_print(const uint8_t * p_uuid);


/**
 * @}
 */

#endif
