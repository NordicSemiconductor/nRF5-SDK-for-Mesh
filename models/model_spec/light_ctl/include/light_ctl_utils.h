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

#ifndef LIGHT_CTL_UTILS_H__
#define LIGHT_CTL_UTILS_H__
#include <stdint.h>

#include "nordic_common.h"

/**
 * @defgroup LIGHT_CTL_UTILS Utility functions for the Light CTL model
 * @ingroup LIGHT_CTL_MODELS
 *
 * These utility functions are used to handle internal 32 bit representation (temperature32) of
 * Light CTL Temperature state values.
 * Temperature32 is a temperature value that is multiplied by 65536 (T32_SCALE_FACTOR). This
 * allows us to keep around a more precise values that will convert back to level without integer
 * division round off errors.
 *
 * @tagMeshMdlSp section 6.1.3.1.1 Binding with the Generic Level state: the formulae are:
 *    temperature = t_min + (generic_level + 32768) * (t_max - t_min) / 65535
 *    generic_level = (temperature - t_min) * 65535 / (t_max - t_min) - 32768
 * So to avoid dividing by 65535 to compute temperature, we multiply the 1st equation by 65535 and
 * get a formula of:
 *    temperature32 = 65535 * t_min + (generic_level + 32768) * (t_max - t_min)
 * and reverse that to get a generic_level of:
 *    generic_level = (temperature32 - t_min * 65535) / (t_max - t_min) - 32768
 * But, we also are storing t_max and t_min as temperature32 values as well (abbreviated as t32_max
 * and t32_min), so the final form of the formulae is:
 *    temperature32 = t32_min + (generic_level + 32768) * (t32_max - t32_min)/65535
 *    generic_level = (((temperature32 - t32_min) * 65535)/(t32_max - t32_min))  - 32768
 *
 * For a delta, we don't add in t_min (that's an offset to get an absolute temperature value
 * into the non-prohibited range), and we don't add in the 32768, since that is designed to
 * convert a signed int into unsigned space - but we need to keep any negative delta value.
 * So the formula is then:
 *    temperature_delta = generic_level_delta * (t_max - t_min) / 65535
 *
 * @note These utility functions assume that the input parameters are sanitized and within the
 * valid limits.
 * @{
 */

/** Scaling factor used by this module to scale given temperature value so as not to loose precision.
 *
 * This factor must be 65535, for proper simplification of level<->temperature conversion
 * equations that ensure min/max values convert correctly back and forth after scaling.
 *
 * @warning Do not change this value.
 */
#define T32_SCALE_FACTOR (65535LL)

/** Macro for performing scaling down */
#define SCALE_DOWN(_input, _scale_factor)   ((_input)/(_scale_factor))

/** Macro for performing scaling up */
#define SCALE_UP(_input, _scale_factor)     ((_input)*(_scale_factor))

/** Converts the scaled up temperature32 value back to neutral value.
 *
 * @param[in] temperature32     Input temperature32 value.
 *
 * @returns The temperature value.
 */
static inline uint16_t light_ctl_utils_temperature32_to_temperature(uint32_t temperature32)
{
    return ((uint32_t)SCALE_DOWN(temperature32, T32_SCALE_FACTOR));
}

/** Converts netural temperature value to internal, scaled up, temperature32 value.
 *
 * @param[in] temperature     Input temperature value.
 *
 * @returns The temperature32 value.
 */
static inline uint32_t light_ctl_utils_temperature_to_temperature32(uint16_t temperature)
{
    return SCALE_UP((uint32_t)temperature, T32_SCALE_FACTOR);
}

/** Converts Generic Level state to internal, scaled up, temperature32 value.
 *
 * See @tagMeshMdlSp section 6.1.3.1.1.
 *
 * @param[in] level           Input Generic Level state value.
 * @param[in] t32_min         Scaled up value of minimum CTL Temperature Range.
 * @param[in] t32_max         Scaled up value of maximum CTL Temperature Range.
 *
 * @returns The temperature32 value.
 */
static inline uint32_t light_ctl_utils_level_to_temperature32(int16_t level, uint32_t t32_min,
                                                              uint32_t t32_max)
{
    return (t32_min + SCALE_DOWN((uint64_t) (level + 32768) * (uint64_t)(t32_max - t32_min), T32_SCALE_FACTOR));
}

/** Converts internal, scaled up, temperature32 value to Generic Level state.
 *
 * See @tagMeshMdlSp section 6.1.3.1.1.
 *
 * @param[in] temperature32   Input temperature32 value.
 * @param[in] t32_min         Scaled up value of minimum CTL Temperature Range.
 * @param[in] t32_max         Scaled up value of maximum CTL Temperature Range.
 *
 * @returns The Generic Level state value.
 */
static inline int16_t light_ctl_utils_temperature32_to_level(uint32_t temperature32, uint32_t t32_min,
                                                             uint32_t t32_max)
{
    if (t32_max == t32_min)
    {
        return 0;
    }

    return ((SCALE_UP((uint64_t)(temperature32 - t32_min), T32_SCALE_FACTOR)/(t32_max - t32_min)) - 32768);
}

/** Converts Generic Level Delta value to internal temperature value.
 * @internal
 *
 * This is used to handle Delta Level values received by the Level Server. It is required because
 * these values will be used to start CTL Temperature transition.
 *
 * @param[in] temperature     Input Generic Level state value.
 * @param[in] t_min           Minimum CTL Temperature Range.
 * @param[in] t_max           Maximum CTL Temperature Range.
 *
 * @returns The temperature32 delta value.
 */
static inline int32_t light_ctl_utils_level_delta_to_temperature_delta(int32_t generic_level_delta,
                                                                       uint16_t t_min, uint16_t t_max)
{
    return (generic_level_delta * ((int32_t)t_max - (int32_t)t_min))/65535L;
}

/** Restricts the given temperature32 value to given minimum and maximum (scaled up) range values.
 *
 * @param[in] temperature32   Input temperature32 value.
 * @param[in] t32_min         Scaled up value of minimum CTL Temperature Range.
 * @param[in] t32_max         Scaled up value of maximum CTL Temperature Range.
 *
 * @returns The temperature32 value restricted to given range
 */
static inline uint32_t light_ctl_utils_temperature32_range_restrict(uint32_t temperature32,
                                                                    uint32_t t32_min, uint32_t t32_max)
{
    /* 6.1.3.1.3 binding with CTL temperature range state */
    return MAX(t32_min, MIN(t32_max, temperature32));
}
/**@} end of LIGHT_CTL_UTILS */
#endif /* LIGHT_CTL_UTILS_H__ */
