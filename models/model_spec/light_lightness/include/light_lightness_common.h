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

#ifndef LIGHT_LIGHTNESS_COMMON_H__
#define LIGHT_LIGHTNESS_COMMON_H__

#include <stdint.h>
#include "model_common.h"

/**
 * @defgroup LIGHT_LIGHTNESS_MODELS Light Lightness models
 * @ingroup MESH_API_GROUP_LIGHT_MODELS
 * This model implements the message based interface required to set the Lightness value on the server. Server model
 * itself is a stateless model. The state information resides in the user application.
 * @{
 */

/** Model Company ID */
#define LIGHT_LIGHTNESS_COMPANY_ID 0xFFFF

/**
 * Minimum allowed value of the Light Lightness Range state defined in
 * @tagMeshMdlSp section 6.1.2.5.
 * */
#define LIGHT_LIGHTNESS_RANGE_MIN (1)

/**
 * Minimum allowed value of the Light Lightness Last state defined in
 * @tagMeshMdlSp section 6.1.2.3.
 * */
#define LIGHT_LIGHTNESS_LAST_MIN (1)

/**
 * Defines minimum value for the Light Lightness Range state.
 *
 * @note Value less than @ref LIGHT_LIGHTNESS_RANGE_MIN is prohibited.
 */
#ifndef LIGHT_LIGHTNESS_DEFAULT_RANGE_MIN
#define LIGHT_LIGHTNESS_DEFAULT_RANGE_MIN              (0x1)
#endif

/**
 * Defines maximum value for the Light Lightness Range state.
 *
 * @note Value less than @ref LIGHT_LIGHTNESS_RANGE_MIN is prohibited.
 */
#ifndef LIGHT_LIGHTNESS_DEFAULT_RANGE_MAX
#define LIGHT_LIGHTNESS_DEFAULT_RANGE_MAX              (0xFFFF)
#endif

/**
 * Defines default value for the Generic OnPowerUp state.
 *
 * See @ref generic_on_powerup_values_t for the allowed values.
 *
 * It affects the initial value of the lightness level after the first device boot-up.
 * See @ref light_lightness_ponoff_binding_setup for more details.
 */
#ifndef LIGHT_LIGHTNESS_DEFAULT_ON_POWERUP
#define LIGHT_LIGHTNESS_DEFAULT_ON_POWERUP             (0x2)
#endif

/**
 * Defines default value for the Light Lightness Actual state.
 *
 * It affects the initial value of the lightness level after the first device boot-up.
 * See @ref light_lightness_ponoff_binding_setup for more details.
 */
#ifndef LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_ACTUAL
#define LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_ACTUAL       (0x0)
#endif

/**
 * Defines default value for the Light Lightness Last state.
 *
 * @note Value less than @ref LIGHT_LIGHTNESS_LAST_MIN is prohibited.
 *
 * It affects the initial value of the lightness level after the first device boot-up.
 * See @ref light_lightness_ponoff_binding_setup for more details.
 */
#ifndef LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_LAST
#define LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_LAST         (0xFFFF)
#endif

/**
 * Defines default value for the Light Lightness Default state.
 *
 * It affects the initial value of the lightness level after the first device boot-up.
 * See @ref light_lightness_ponoff_binding_setup for more details.
 */
#ifndef LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_DEFAULT
#define LIGHT_LIGHTNESS_DEFAULT_LIGHTNESS_DEFAULT      (0x0)
#endif

/**
 * Defines default value for the Generic Default Transition Time state.
 */
#ifndef LIGHT_LIGHTNESS_DEFAULT_DTT
#define LIGHT_LIGHTNESS_DEFAULT_DTT                    (0x0)
#endif

/** Status values for light_lightness_range_status_params_t  */
typedef enum
{
    /** The provided range set values are valid */
    LIGHT_LIGHTNESS_RANGE_STATUS_SUCCESS,
    /** The provided value for Range Min cannot be set */
    LIGHT_LIGHTNESS_RANGE_STATUS_CANNOT_SET_RANGE_MIN,
    /** The provided value for Range Max cannot be set */
    LIGHT_LIGHTNESS_RANGE_STATUS_CANNOT_SET_RANGE_MAX
} light_lightness_range_status_t;


/**
 * Unpacked message structure typedefs are used for API interfaces and for implementing model code. This helps to minimize code
 * footprint.
 */

/** Message format for the light_lightness set message. */
typedef struct
{
    uint16_t lightness;                    /**< Value of the Lightness state */
    uint8_t tid;                           /**< Transaction ID */
} light_lightness_set_params_t;

/** Message format for the light_lightness Linear set message. */
typedef struct
{
    uint16_t lightness;                    /**< Value of the Lightness Linear state */
    uint8_t tid;                           /**< Transaction ID */
} light_lightness_linear_set_params_t;

/** Message format for the light_lightness Default set message. */
typedef struct
{
    uint16_t lightness;                    /**< Value of the Lightness state */
} light_lightness_default_set_params_t;

/** Message format for the light_lightness Range set message. */
typedef struct
{
    uint16_t range_min;                    /**< The value of the lightness range min state */
    uint16_t range_max;                    /**< The value of the lightness range max state */
} light_lightness_range_set_params_t;

/** Message format for the light_lightness delta set "message". */
/** There is no official delta set message for light lightness server. */
/** This structure supports internal messages, specifically for server<->app interaction. */
typedef struct
{
    int32_t delta_lightness;               /**< Value of the Lightness delta value */
    uint8_t tid;                           /**< Transaction ID */
} light_lightness_delta_set_params_t;

/** Message format for the light_lightness move set "message". */
/** There is no official move set message for light lightness server. */
/** This structure supports internal messages, specifically for server<->app interaction. */
typedef struct
{
    int16_t delta;                        /**< Value of the Lightness move value */
    uint8_t tid;                          /**< Transaction ID */
} light_lightness_move_set_params_t;

/** Parameters for the Power OnOff Set message. */
typedef struct
{
    uint8_t on_powerup;                               /**< OnPowerUp value */
} light_lightness_ponoff_set_params_t;

/** Parameters for the Power OnOff Status message. */
typedef struct
{
    uint8_t on_powerup;                               /**< OnPowerUp value */
} light_lightness_ponoff_status_params_t;


/** Parameters for the DTT Set message. */
typedef struct
{
    uint32_t dtt_ms;                                   /**< DTT value */
} light_lightness_dtt_set_params_t;


/** Parameters for the DTT Status message. */
typedef struct
{
    uint32_t dtt_ms;                                   /**< DTT value */
} light_lightness_dtt_status_params_t;


/** Parameters for the light_lightness Status message. */
typedef struct
{
    uint16_t present_lightness;            /**< The present value of the Lightness state */
    uint16_t target_lightness;             /**< The target value of the Lightness state (optional) */
    uint32_t remaining_time_ms;            /**< Remaining time value in milliseconds */
} light_lightness_status_params_t;

/** Parameters for the light_lightness Linear Status message. */
typedef struct
{
    uint16_t present_lightness;            /**< The present value of the Linear Lightness state */
    uint16_t target_lightness;             /**< The target value of the Linear Lightness state (optional) */
    uint32_t remaining_time_ms;            /**< Remaining time value in milliseconds */
} light_lightness_linear_status_params_t;

/** Parameters for the light_lightness Last Status message. */
typedef struct
{
    uint16_t lightness;                    /**< The value of the Lightness last state */
} light_lightness_last_status_params_t;

/** Parameters for the light_lightness Default Status message. */
typedef struct
{
    uint16_t lightness;                    /**< The value of the Lightness default state */
} light_lightness_default_status_params_t;

/** Parameters for the light_lightness Range Status message. */
typedef struct
{
    uint8_t status;                        /**< The status of the last operation */
    uint16_t range_min;                    /**< The value of the lightness range min state */
    uint16_t range_max;                    /**< The value of the lightness range max state */
} light_lightness_range_status_params_t;

/** Parameter format for code while booting - the stored values will
 * be read and passed to the model to do the state binding and set the
 * current lightness based on these stored values. */
typedef struct
{
    uint8_t onpowerup;                              /**< The saved value of the generic on powerup state. */
    uint16_t actual_lightness;                      /**< The saved value of the light lightness actual state. */
    uint16_t last_lightness;                        /**< The saved value of the light lightness last state. */
    uint16_t default_lightness;                     /**< The saved value of the light lightness default state. */
    light_lightness_range_status_params_t range;    /**< The saved value of the light lightness range state. */
} light_lightness_saved_values_t;

/**@} end of LIGHT_LIGHTNESS_MODELS */
#endif /* LIGHT_LIGHTNESS_COMMON_H__ */
