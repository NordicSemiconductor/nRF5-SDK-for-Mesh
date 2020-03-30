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

#ifndef LIGHT_LC_LIGHT_PI_H__
#define LIGHT_LC_LIGHT_PI_H__

#include "light_lc_common.h"
#include "light_lc_setup_server.h"

/**
 * @internal
 * @defgroup LIGHT_LC_LIGHT_PI Light LC Light PI Feedback Regulator model interface
 * @ingroup LIGHT_LC_MODELS
 *
 * The purpose of the Light LC PI (Proportional-Integral) Feedback Regulator is setting the light to a level
 * that ensures the value of the Light LC Ambient LuxLevel state reported by an ambient light sensor
 * is equal to the required value for a given state. See @tagMeshMdlSp section 6.2.6 for more information.
 *
 *
 * @{
 */

/** Updates Light LC PI Feedback Regulator state.
 *
 * The Light LC PI Feedback Regulator is supposed to run periodically at the interval
 * specified in @ref LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MS.
 *
 * @param[in] p_s_server           Pointer to the model structure.
 */
void light_lc_light_pi_update(light_lc_setup_server_t * p_s_server);

/**@} end of LIGHT_LC_LIGHT_PI */
#endif /* LIGHT_LC_LIGHT_PI_H__ */
