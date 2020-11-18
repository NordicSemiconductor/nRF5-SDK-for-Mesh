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

#include "pwm_utils.h"
#include "log.h"
#include "nrf_mesh_assert.h"

/* Input is a signed 16-bit integer. The following scaling maps the range [INT16_MIN, INT16_MAX] to
 * [1, pwm_tick_max], where pwm_tick_max is the tick value for the 100% PWM duty cycle.
 */
static inline uint16_t pwm_signed_level_to_ticks(pwm_utils_contex_t * p_ctx, int16_t signed_value)
{
    return (uint16_t)((((int32_t)(signed_value - INT16_MIN) * (p_ctx->pwm_ticks_max - 1))/UINT16_MAX) + 1);
}

/* Input is a unsigned 16-bit integer. The following scaling maps the range [0, UINT16_MAX] to [1,
 * pwm_tick_max], where pwm_tick_max is the tick value for the 100% PWM duty cycle.
 */
static inline uint16_t pwm_unsigned_level_to_ticks(pwm_utils_contex_t * p_ctx, uint16_t unsigned_value)
{
    return (uint16_t)(((uint32_t)(unsigned_value * (p_ctx->pwm_ticks_max - 1))/UINT16_MAX) + 1);
}

/* Input is a unsigned 16-bit integer. The following scaling maps the range [1, pwm_tick_max] to
 * [0, UINT16_MAX], where pwm_tick_max is the tick value for the 100% PWM duty cycle.
 */
static inline uint16_t pwm_ticks_to_unsigned_level(pwm_utils_contex_t * p_ctx, uint16_t ticks)
{
    if (ticks == 0)
    {
        ticks = 1;
    }

    return (uint16_t)(((uint32_t)(ticks - 1) * UINT16_MAX) / (uint32_t)(p_ctx->pwm_ticks_max - 1));
}

/* Input is a unsigned 16-bit integer. The following scaling maps the range [1, pwm_tick_max] to
 * [INT16_MIN, INT16_MAX], where pwm_tick_max is the tick value for the 100% PWM duty cycle.
 */
static inline int16_t pwm_ticks_to_signed_level(pwm_utils_contex_t * p_ctx, uint16_t ticks)
{
    return (int16_t)((int32_t)pwm_ticks_to_unsigned_level(p_ctx, ticks) + INT16_MIN);
}

/***** Interface functions *****/
void pwm_utils_enable(pwm_utils_contex_t * p_ctx)
{
    /* Prior to instantiation the value of pwm_tick_max is zero. */
    NRF_MESH_ASSERT(!p_ctx->pwm_ticks_max);
   /* The pwm peripheral is not enabled. */

    NRF_MESH_ERROR_CHECK(app_pwm_init(p_ctx->p_pwm, p_ctx->p_pwm_config, NULL));
    p_ctx->pwm_ticks_max = app_pwm_cycle_ticks_get(p_ctx->p_pwm);
    __LOG(LOG_SRC_APP, LOG_LEVEL_INFO, "PWM max ticks: %d\n", p_ctx->pwm_ticks_max);

    /* The pwm peripheral uses pwm_tick_max as a divisor. */
    /* Thus the value of pwm_tick_max must be non-zero here.*/
    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);

    app_pwm_enable(p_ctx->p_pwm);
}

void pwm_utils_level_set(pwm_utils_contex_t * p_ctx, int16_t level)
{
    /* The pwm peripheral is enabled. */
    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);
    (void) app_pwm_channel_duty_ticks_set(p_ctx->p_pwm, p_ctx->channel, pwm_signed_level_to_ticks(p_ctx, level));
}

int16_t pwm_utils_level_get(pwm_utils_contex_t * p_ctx)
{
    /* The pwm peripheral is enabled. */
    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);
    return pwm_ticks_to_signed_level(p_ctx, app_pwm_channel_duty_ticks_get(p_ctx->p_pwm, p_ctx->channel));
}

uint32_t pwm_utils_level_set_unsigned(pwm_utils_contex_t * p_ctx, uint16_t level)
{
    /* The pwm peripheral is enabled. */
    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);
    return (app_pwm_channel_duty_ticks_set(p_ctx->p_pwm, p_ctx->channel, pwm_unsigned_level_to_ticks(p_ctx, level)));
}

uint16_t pwm_utils_level_get_unsigned(pwm_utils_contex_t * p_ctx)
{
    /* The pwm peripheral is enabled. */
    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);
    return pwm_ticks_to_unsigned_level(p_ctx, app_pwm_channel_duty_ticks_get(p_ctx->p_pwm, p_ctx->channel));
}
