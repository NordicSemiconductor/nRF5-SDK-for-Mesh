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

#include <unity.h>
#include <cmock.h>

#include "light_ctl_utils.h"
#include "light_ctl_common.h"

/* This factor must be 65535, for proper simplification of level<->temperature conversion
 * equations that ensure min/max values convert correctly back and forth after scaling. */
#define TEMPERATURE_SCALING_FACTOR	(65535LL)

/*****************************************************************************
* Setup functions
*****************************************************************************/

void setUp(void)
{
}

void tearDown(void)
{
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_light_ctl_utils_temperature_to_temperature32_and_reverse(void)
{
    uint64_t temp;
    uint32_t temp32;

    temp = LIGHT_CTL_TEMPERATURE_MIN_LIMIT;
    temp32 = light_ctl_utils_temperature_to_temperature32(temp);
    TEST_ASSERT_EQUAL(temp * TEMPERATURE_SCALING_FACTOR, temp32);
    TEST_ASSERT_EQUAL(temp, light_ctl_utils_temperature32_to_temperature(temp32));

    temp = LIGHT_CTL_TEMPERATURE_MAX_LIMIT;
    temp32 = light_ctl_utils_temperature_to_temperature32(temp);
    TEST_ASSERT_EQUAL(temp * TEMPERATURE_SCALING_FACTOR, temp32);
    TEST_ASSERT_EQUAL(temp, light_ctl_utils_temperature32_to_temperature(temp32));
}

void test_light_ctl_utils_level_to_temperature32(void)
{
    int16_t level;
    uint32_t range_min = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT);
    uint32_t range_max = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT);

    level = INT16_MIN;
    TEST_ASSERT_EQUAL(LIGHT_CTL_TEMPERATURE_MIN_LIMIT,
        light_ctl_utils_temperature32_to_temperature(light_ctl_utils_level_to_temperature32(level, range_min, range_max)));
    level = INT16_MAX;
    TEST_ASSERT_EQUAL(LIGHT_CTL_TEMPERATURE_MAX_LIMIT,
        light_ctl_utils_temperature32_to_temperature(light_ctl_utils_level_to_temperature32(level, range_min, range_max)));

    /* For level = -22768, corresponding temp = 800 + (-22768 + 32768) * 19200 / 65535 = 3729 */
    level = -22768;
    TEST_ASSERT_EQUAL(3729,
        light_ctl_utils_temperature32_to_temperature(light_ctl_utils_level_to_temperature32(level, range_min, range_max)));

    /* For level = 10000, corresponding temp = 800 + (10000 + 32768) * 19200 / 65535 = 13329 */
    level = 10000;
    TEST_ASSERT_EQUAL(13329,
        light_ctl_utils_temperature32_to_temperature(light_ctl_utils_level_to_temperature32(level, range_min, range_max)));

    /* If range min = range max, all level values map to single temperature32 value */
    range_min = range_max;
    level = INT16_MIN;
    TEST_ASSERT_EQUAL(LIGHT_CTL_TEMPERATURE_MAX_LIMIT,
    light_ctl_utils_temperature32_to_temperature(light_ctl_utils_level_to_temperature32(level, range_min, range_max)));

    level = INT16_MAX;
    TEST_ASSERT_EQUAL(LIGHT_CTL_TEMPERATURE_MAX_LIMIT,
    light_ctl_utils_temperature32_to_temperature(light_ctl_utils_level_to_temperature32(level, range_min, range_max)));
}

void test_light_ctl_utils_temperature32_to_level(void)
{
    uint32_t temp32;
    uint32_t range_min = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT);
    uint32_t range_max = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT);

    temp32 = range_min;
    TEST_ASSERT_EQUAL(INT16_MIN,
                      light_ctl_utils_temperature32_to_level(temp32, range_min, range_max));
    temp32 = range_max;
    TEST_ASSERT_EQUAL(INT16_MAX,
                      light_ctl_utils_temperature32_to_level(temp32, range_min, range_max));

    /* If temp = 10000, level = ((10000 - 800) * 65535 / 19200) - 32768
     *                       = (uint32_t)31402.1875 - 32768 = -1366
     */
    temp32 = SCALE_UP(10000, TEMPERATURE_SCALING_FACTOR);
    TEST_ASSERT_EQUAL(-1366, light_ctl_utils_temperature32_to_level(temp32, range_min, range_max));

    /* If range min = range max, level is zero -> special case */
    range_min = range_max;
    TEST_ASSERT_EQUAL(0, light_ctl_utils_temperature32_to_level(temp32, range_min, range_max));
}

void test_light_ctl_utils_level_delta_to_temperature_delta(void)
{
    /* level delta is signed offset indicating how much level value should be changed */
    int32_t level_delta;
    int32_t temp_delta;

    /* level delta of 0 should correspond to 0 temperature delta */
    level_delta = 0;
    temp_delta = 0;
    TEST_ASSERT_EQUAL(temp_delta, light_ctl_utils_level_delta_to_temperature_delta(level_delta,
                                                                LIGHT_CTL_TEMPERATURE_MIN_LIMIT,
                                                                LIGHT_CTL_TEMPERATURE_MAX_LIMIT));

    /* level delta of 65535 should correspond to maximum temperature delta of (tmax - tmin) */
    level_delta = UINT16_MAX;
    temp_delta = LIGHT_CTL_TEMPERATURE_MAX_LIMIT - LIGHT_CTL_TEMPERATURE_MIN_LIMIT;
    TEST_ASSERT_EQUAL(temp_delta, light_ctl_utils_level_delta_to_temperature_delta(level_delta,
                                                                LIGHT_CTL_TEMPERATURE_MIN_LIMIT,
                                                                LIGHT_CTL_TEMPERATURE_MAX_LIMIT));

    level_delta = -level_delta;
    temp_delta = -temp_delta;
    TEST_ASSERT_EQUAL(temp_delta, light_ctl_utils_level_delta_to_temperature_delta(level_delta,
                                                                LIGHT_CTL_TEMPERATURE_MIN_LIMIT,
                                                                LIGHT_CTL_TEMPERATURE_MAX_LIMIT));
}

void test_light_ctl_utils_temperature32_range_restrict(void)
{
    uint32_t temp32;
    uint32_t range_min = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT);
    uint32_t range_max = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT);

    temp32 = 0;
    TEST_ASSERT_EQUAL(range_min,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));

    temp32 = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT-1);
    TEST_ASSERT_EQUAL(range_min,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));

    temp32 = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT);
    TEST_ASSERT_EQUAL(temp32,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));

    temp32 = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT+1);
    TEST_ASSERT_EQUAL(temp32,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));

    temp32 = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT-1);
    TEST_ASSERT_EQUAL(temp32,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));

    temp32 = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT);
    TEST_ASSERT_EQUAL(temp32,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));

    temp32 = light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT+1);
    TEST_ASSERT_EQUAL(range_max,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));

    temp32 = UINT32_MAX;
    TEST_ASSERT_EQUAL(range_max,
                      light_ctl_utils_temperature32_range_restrict(temp32, range_min, range_max));
}
