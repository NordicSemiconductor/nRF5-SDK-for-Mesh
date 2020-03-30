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

/* Input is a signed 16-bit integer. The following scaling maps the range
 * [INT16_MIN, INT16_MAX] to [0, pwm_tick_max], where pwm_tick_max is the tick value for
 * the 100% PWM duty cycle.
 */
static inline uint16_t pwm_level_to_ticks(pwm_utils_contex_t * p_ctx, int16_t signed_value)
{
    uint16_t retval;

    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);
    /* The pwm peripheral is enabled. */

    retval = (uint16_t)(((int32_t)(signed_value - INT16_MIN) * p_ctx->pwm_ticks_max)/UINT16_MAX);

    if (retval == 0)
    {
        retval = 1;
    }

    return retval;
}

/* Input is a unsigned 16-bit integer. The following scaling maps the range
 * [0, pwm_tick_max] to [INT16_MIN, INT16_MAX], where pwm_tick_max is the tick value for
 * the 100% PWM duty cycle.
 */
static inline int16_t pwm_ticks_to_level(pwm_utils_contex_t * p_ctx, uint16_t ticks)
{
    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);
    /* The pwm peripheral is enabled. */

    /* pwm_level_to_ticks(INT16_MIN) = 1. */
    /* So 1 is a special case. */
    ticks = (1 == ticks) ? 0 : ticks;

    return ((int32_t)(UINT16_MAX * ticks)
            - (int32_t)(INT16_MIN * p_ctx->pwm_ticks_max)) / p_ctx->pwm_ticks_max;
}

/***** Interface functions *****/
void pwm_utils_level_set(pwm_utils_contex_t * p_ctx, int16_t level)
{
    (void) app_pwm_channel_duty_ticks_set(p_ctx->p_pwm, p_ctx->channel, pwm_level_to_ticks(p_ctx, level));
}

int16_t pwm_utils_level_get(pwm_utils_contex_t * p_ctx)
{
    NRF_MESH_ASSERT(p_ctx->pwm_ticks_max);
    /* The pwm peripheral is enabled. */

    return pwm_ticks_to_level(p_ctx, app_pwm_channel_duty_ticks_get(p_ctx->p_pwm, p_ctx->channel));
}

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
