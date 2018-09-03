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

#if defined(S130) || defined(S132) || defined(S140) || defined(S112)
#include "nrf_nvic.h"
extern nrf_nvic_state_t nrf_nvic_state;
#endif

typedef void (*execution_start_cb_t)(void);

#if defined S140
#include "nrf_soc.h"
/**
 * Checks the current interrupt priority of the softdevice event interrupt.
 * In case the priority is not the lowest one the assertion is generated.
 *
 * @note The function is required only for S140 v6.0.0
 *       since it has the priority issue at the moment
 */
static inline void softdevice_irq_priority_checker(void)
{
    uint32_t priority = 0ul;

    ERROR_CHECK(sd_nvic_GetPriority(SD_EVT_IRQn, &priority));
    if (priority != NRF_MESH_IRQ_PRIORITY_LOWEST)
    {
        ERROR_CHECK(NRF_ERROR_INTERNAL);
    }
}
#endif

/**
 * Run function for starting dynamic behavior.
 *
 * Unless the application event handling is running in thread mode, there is a risk of race
 * conditions if events are processed while running the start function in thread mode. This
 * function will prevent this by disabling all application level interrupts while running the start
 * function.
 *
 * @param[in]  execution_start_callback  Function for starting dynamic bahavior.
 */
static inline void execution_start(execution_start_cb_t execution_start_callback)
{
    uint8_t is_nested;
    ERROR_CHECK(sd_nvic_critical_region_enter(&is_nested));
    execution_start_callback();
    ERROR_CHECK(sd_nvic_critical_region_exit(is_nested));
}


/**
 * Generates a UUID using DEVICE ID and given prefix bytes
 *
 * @note: This function merely generates a 128 bit unique number for a purpose of demonstration.
 * The UUID for the end product should be generated as per the format specified in ITU-T Rec. X.667.
 *
 * @param[out] p_uuid_dest      Pointer to buffer storing the UUID. This buffer must be @ref
 *                              NRF_MESH_UUID_SIZE bytes long.
 *
 * @param[in]  p_uuid_prefix    Array of bytes that will be used as first of the UUID.
 * @param[in]  uuid_prefix_len  Length of the array pointed by `p_uuid_prefix`. Maximum value of
 *                              this parameter can be (@ref NRF_MESH_UUID_SIZE/2). If value of this
 *                              parameter is less than (@ref NRF_MESH_UUID_SIZE/2), remaining bytes
 *                              will be filled with zeros.
 *
 * @retval NRF_SUCCESS               UUID is generated successfully.
 * @retval NRF_ERROR_INVALID_LENGTH  Supplied uuid_prefix_len is greater than @ref NRF_MESH_UUID_SIZE/2.
 *
 */
uint32_t mesh_app_uuid_gen(uint8_t * p_uuid_dest, const uint8_t * p_uuid_prefix, uint8_t uuid_prefix_len);


/**
 * @}
 */

#endif
