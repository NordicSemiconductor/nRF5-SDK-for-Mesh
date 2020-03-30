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

#ifndef LIGHT_LIGHTNESS_UTILS_H__
#define LIGHT_LIGHTNESS_UTILS_H__

#include <stdint.h>
#include "utils.h"
#include "generic_level_common.h"

/**
 * @defgroup LIGHT_LIGHTNESS_UTILS Light Lightness model utility functions
 * @ingroup LIGHT_LIGHTNESS_MODELS
 * @{
 */

/**
 * Converts the Light Lightness Actual state to the Light Lightness Linear state.
 *
 * See @tagMeshMdlSp section 6.1.2.2.1 for more details.
 *
 * @param[in]   ll_actual  The Light Lightness Actual state.
 *
 * @returns  The Light Lightness Linear state.
 */
static inline uint16_t light_lightness_utils_actual_to_linear(uint16_t ll_actual)
{
    return CEIL_DIV((uint32_t)ll_actual * ll_actual, UINT16_MAX);
}

/**
 * Convert the Light Lightness Linear state to the Light Lightness Actual state.
 *
 * See @tagMeshMdlSp section 6.1.2.2.1 for more details.
 *
 * @param[in]   ll_linear  The Light Lightness Linear state.
 *
 * @returns  The Light Lightness Actual state.
 */
static inline uint16_t light_lightness_utils_linear_to_actual(uint16_t ll_linear)
{
    return uint_sqrt((uint32_t)ll_linear * UINT16_MAX);
}

/**
 * Convert the Light Lightness Actual state to the Generic Level state.
 *
 * See @tagMeshMdlSp section 6.1.2.2.2 for more details.
 *
 * @param[in]   ll_actual  The Light Lightness Actual state.
 *
 * @returns  The Generic Level state.
 */
static inline int16_t light_lightness_utils_actual_to_generic_level(uint16_t ll_actual)
{
    return ll_actual - GENERIC_LEVEL_MIN;
}

/**
 * Convert the Generic Level state to the Light Lightness Actual state.
 *
 * See @tagMeshMdlSp section 6.1.2.2.2 for more details.
 *
 * @param[in]   level  The Generic Level state.
 *
 * @returns  The Light Lightness Actual state.
 */
static inline uint16_t light_lightness_utils_generic_level_to_actual(int16_t level)
{
    return level + GENERIC_LEVEL_MIN;
}

/**
 * Convert the Light Lightness Actual state to the Generic OnOff state.
 *
 * See @tagMeshMdlSp section 6.1.2.2.3 for more details.
 *
 * @param[in]   ll_actual  Light Lightness Actual state.
 *
 * @returns  Generic OnOff state.
 */
static inline uint8_t light_lightness_utils_actual_to_generic_onoff(uint16_t ll_actual)
{
    return (ll_actual == 0 ? 0 : 1);
}

/**
 * Convert the Light Lightness Actual state to the Generic OnOff state.
 *
 * See @tagMeshMdlSp section 6.1.2.2.5 for more details.
 *
 * @param[in]   ll_actual   The Light Lightness Actual state.
 * @param[in]   range_min   The Light Lightness Range Min state.
 * @param[in]   range_max   The Light Lightness Range Max state.
 *
 * @returns  The Light Lightness Range state.
 */
static inline uint16_t light_lightness_utils_actual_to_range_restrict(uint16_t ll_actual,
                                                                      uint16_t range_min,
                                                                      uint16_t range_max)
{
    return ll_actual != 0 ? MIN(MAX(ll_actual, range_min), range_max) : 0;
}

/**@} end of LIGHT_LIGHTNESS_UTILS */
#endif /* LIGHT_LIGHTNESS_UTILS_H__ */
