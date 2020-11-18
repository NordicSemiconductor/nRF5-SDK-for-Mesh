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

#ifndef PWM_UTILS_H__
#define PWM_UTILS_H__

#include <stdint.h>

#include "nrf_mesh_config_core.h"
#include "app_pwm.h"

/**
 * @defgroup PWM_UTILS PWM utility functions
 * @ingroup MESH_API_GROUP_APP_SUPPORT
 *
 * This module implements some utility functions for using PWM functionality in the application.
 *
 * @{
 */

/**
 * Context structure for defining application specific instance of PWM peripheral to be used with
 * this module.
 * */
typedef struct
{
    /** Pointer to a PWM instance created using @link_APP_PWM_INSTANCE macro. */
    const app_pwm_t * p_pwm;
    /** Pointer to a @link_app_pwm_config_t PWM configuration structure. */
    app_pwm_config_t * p_pwm_config;
    /** Channel number to use */
    uint8_t channel;

    /** Internal */
    uint16_t pwm_ticks_max;
} pwm_utils_contex_t;


/**
 * Enables the PWM channel.
 *
 * @param[in] p_ctx   Pointer to a PWM utility context structure @ref pwm_utils_contex_t.
 */
void pwm_utils_enable(pwm_utils_contex_t * p_ctx);

/**
 * Set the PWM duty cycle corresponding to given int16 value.
 *
 * This function maps the range [INT16_MIN, INT16_MAX] to [1, PWM max], where PWM max is
 * the tick value for the 100% PWM duty cycle.
 *
 * @param[in] p_ctx   Pointer to a PWM utils context structure @ref pwm_utils_contex_t.
 * @param[in] level   Desired PWM duty cycle represented in the form of int16 value.
 */
void pwm_utils_level_set(pwm_utils_contex_t * p_ctx, int16_t level);

/**
 * Get the int16 value corresponding to the PWM duty cycle.
 *
 * This function maps the range [1, PWM max] to [INT16_MIN, INT16_MAX], where PWM max is
 * the tick value for the 100% PWM duty cycle.
 *
 * @param[in] p_ctx   Pointer to a PWM utils context structure @ref pwm_utils_contex_t.
 * @returns int16 representation of the current PWM duty cycle.
 */
int16_t pwm_utils_level_get(pwm_utils_contex_t * p_ctx);

/**
 * Set the PWM duty cycle corresponding to given uint16 value.
 *
 * This function maps the range [0, UINT16_MAX] to [1, PWM max], where PWM max is
 * the tick value for the 100% PWM duty cycle.
 *
 * @param[in] p_ctx   Pointer to a PWM utils context structure @ref pwm_utils_contex_t.
 * @param[in] level   Desired PWM duty cycle represented in the form of uint16 value.
 *
 * @retval NRF_SUCCESS              Successfully set PWM duty cycle.
 * @retval NRF_ERROR_INVALID_STATE  Invalid state to perform operation.
 * @retval NRF_ERROR_BUSY           PPI channels for synchronization are still in use.
 */
uint32_t pwm_utils_level_set_unsigned(pwm_utils_contex_t * p_ctx, uint16_t level);

/**
 * Get the uint16 value corresponding to the PWM duty cycle.
 *
 * This function maps the range [1, PWM max] to [0, UINT16_MAX], where PWM max is
 * the tick value for the 100% PWM duty cycle.
 *
 * @param[in] p_ctx   Pointer to a PWM utils context structure @ref pwm_utils_contex_t.
 * @returns uint16 representation of the current PWM duty cycle.
 */
uint16_t pwm_utils_level_get_unsigned(pwm_utils_contex_t * p_ctx);

/** @} end of PWM_UTILS */

#endif
