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

#ifndef GENERIC_PONOFF_COMMON_H__
#define GENERIC_PONOFF_COMMON_H__

#include <stdint.h>

/**
 * @defgroup GENERIC_PONOFF_MODEL Generic Power OnOff model
 * @ingroup MESH_API_GROUP_GENERIC_MODELS
 * This model implements the message based interface required to set the Power OnOff value on the server. Server model
 * itself is a stateless model. The state information resides in the user application.
 *
 * @{
 */

/** Model Company ID */
#define GENERIC_PONOFF_COMPANY_ID 0xFFFF

/** Maximum value of the OnPowerUp state, as defined in @tagMeshMdlSp */
#define GENERIC_ON_POWERUP_MAX    (0x02)

/** Valid values of OnPowerUp state. */
typedef enum
{
    /** After being powered up, the element is in an off state. */
    GENERIC_ON_POWERUP_OFF = 0,
    /** After being powered up, the element is in an On state and uses default state values. */
    GENERIC_ON_POWERUP_DEFAULT,
    /** If a transition was in progress when powered down, the element restores the target state
     *  when powered up. Otherwise the element restores the state it was in when powered down. */
    GENERIC_ON_POWERUP_RESTORE,
    /* Invalid value */
    GENERIC_ON_POWERUP_INVALID
} generic_on_powerup_values_t;

/**
 * Unpacked message structure typedefs are used for API interfaces and for implementing model code. This helps to minimize code
 * footprint.
 */

/** Parameters for the Power OnOff Set message. */
typedef struct
{
    uint8_t on_powerup;                               /**< OnPowerUp value */
} generic_ponoff_set_params_t;

/** Parameters for the Power OnOff Status message. */
typedef struct
{
    uint8_t on_powerup;                               /**< OnPowerUp value */
} generic_ponoff_status_params_t;


/**@} end of GENERIC_PONOFF_MODEL */
#endif /* GENERIC_PONOFF_COMMON_H__ */
