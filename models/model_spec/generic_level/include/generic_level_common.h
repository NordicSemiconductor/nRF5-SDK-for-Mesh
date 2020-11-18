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

#ifndef GENERIC_LEVEL_COMMON_H__
#define GENERIC_LEVEL_COMMON_H__

#include <stdint.h>
#include "model_common.h"

/**
 * @defgroup GENERIC_LEVEL_MODEL Generic Level model
 * @ingroup MESH_API_GROUP_GENERIC_MODELS
 * This model implements the message based interface required to set the Level value on the server. Server model
 * itself is a stateless model. The state information resides in the user application.
 * @{
 */

/** Model Company ID */
#define GENERIC_LEVEL_COMPANY_ID 0xFFFF

/** Generic Level state minimum value. */
#define GENERIC_LEVEL_MIN (INT16_MIN)

/** Generic Level state maximum value. */
#define GENERIC_LEVEL_MAX (INT16_MAX)

/**
 * Defines default value for the Generic Level state.
 */
#ifndef GENERIC_LEVEL_DEFAULT_LEVEL
#define GENERIC_LEVEL_DEFAULT_LEVEL     (0x0)
#endif

/**
 * Unpacked message structure typedefs are used for API interfaces and for implementing model code. This helps to minimize code
 * footprint.
 */

/** Parameters for the generic_level Set message. */
typedef struct
{
    int16_t level;                                       /**< Value of the Level state */
    uint8_t tid;                                          /**< Transaction ID */
} generic_level_set_params_t;

/** Message format for the generic_level Delta Set message. */
typedef struct
{
    int32_t delta_level;                                  /**< Value of the Delta Level state */
    uint8_t tid;                                          /**< Transaction ID */
} generic_level_delta_set_params_t;

/** Message format for the generic_level Move Set message. */
typedef struct
{
    int16_t move_level;                                    /**< Value of the Move Level state */
    uint8_t tid;                                           /**< Transaction ID */
} generic_level_move_set_params_t;

/** Parameters for the generic_level Status message. */
typedef struct
{
    int16_t present_level;                                 /**< The present value of the Generic Level state */
    int16_t target_level;                                  /**< The target value of the Generic Level state (optional) */
    uint32_t remaining_time_ms;                            /**< Remaining time value in milliseconds */
} generic_level_status_params_t;

/**@} end of GENERIC_LEVEL_MODEL */
#endif /* GENERIC_LEVEL_COMMON_H__ */
