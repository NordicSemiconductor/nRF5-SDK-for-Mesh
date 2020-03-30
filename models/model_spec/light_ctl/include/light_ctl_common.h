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

#ifndef LIGHT_CTL_COMMON_H__
#define LIGHT_CTL_COMMON_H__

#include <stdint.h>
#include "model_common.h"
#include "nrf_mesh_assert.h"

/**
 * @defgroup LIGHT_CTL_MODELS Light CTL models
 * @ingroup MESH_API_GROUP_LIGHT_MODELS
 *
 * This model implements the message based interface required to set the Lightness, Color
 * temperature and Delta UV value on the server. Server model itself is a stateless model. The state
 * information resides in the user application.
 * @{
 */

/** Model Company ID */
#define LIGHT_CTL_COMPANY_ID (0xFFFF)

/** Unknown Light CTL Temperature value */
#define LIGHT_CTL_TEMPERATURE_UNKNOWN (0xFFFF)

/** The Light CTL Temperature Range - Minimum limit allowed by mesh spec - @tagMeshMdlSp section 6.1.3.1 */
#define LIGHT_CTL_TEMPERATURE_MIN_LIMIT (0x0320)
/** The Light CTL Temperature Range - Maximum limit allowed by mesh spec - @tagMeshMdlSp section 6.1.3.1 */
#define LIGHT_CTL_TEMPERATURE_MAX_LIMIT (0x4E20)

/** Defines the default value for the Light CTL Temperature state from @tagMeshMdlSp section 6.1.3.1 */
#ifndef LIGHT_CTL_DEFAULT_TEMPERATURE
#define LIGHT_CTL_DEFAULT_TEMPERATURE (8000)
#endif

/** Defines the default value for the Light CTL Delta UV state from @tagMeshMdlSp section 6.1.3.2 */
#ifndef LIGHT_CTL_DEFAULT_TEMPERATURE_DEFAULT
#define LIGHT_CTL_DEFAULT_TEMPERATURE_DEFAULT (8000)
#endif

/** Defines the application specific value for the minumum Light CTL Temperature from
 * @tagMeshMdlSp section 6.1.3.3 */
#ifndef LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN
#define LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MIN (0x0320)
#endif

/** Defines the application specific value for the maximum Light CTL Temperature from
 * @tagMeshMdlSp section 6.1.3.3 */
#ifndef LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX
#define LIGHT_CTL_DEFAULT_ALLOWED_TEMPERATURE_MAX (0x4E20)
#endif

/** Defines the default value for the Light CTL Delta UV state from @tagMeshMdlSp section 6.1.3.4 */
#ifndef LIGHT_CTL_DEFAULT_DELTA_UV
#define LIGHT_CTL_DEFAULT_DELTA_UV (0)
#endif

/** Defines the default value for the Light CTL Delta UV Default state from @tagMeshMdlSp section 6.1.3.5 */
#ifndef LIGHT_CTL_DEFAULT_DELTA_UV_DEFAULT
#define LIGHT_CTL_DEFAULT_DELTA_UV_DEFAULT (0)
#endif

/** Status values for @ref light_ctl_temperature_range_status_params_t  */
typedef enum
{
    /** The provided range set values are valid */
    LIGHT_CTL_RANGE_STATUS_GOOD,
    /** The provided MINIMUM range set value is invalid */
    LIGHT_CTL_RANGE_STATUS_CANNOT_SET_MIN_RANGE,
    /** The provided MAXIMUM range set value is invalid */
    LIGHT_CTL_RANGE_STATUS_CANNOT_SET_MAX_RANGE
} light_ctl_range_status_t;


/**
 * Unpacked message structure typedefs are used for API interfaces and for implementing model code.
 * This helps to minimize code footprint.
 */

/** Message format for the CTL Set message. */
typedef struct
{
    uint16_t lightness;                    /**< Value of the Lightness state */
    uint32_t temperature32;                /**< Value of the Temperature32 state */
    uint16_t delta_uv;                     /**< Value of the Delta UV state */
    uint8_t tid;                           /**< Transaction ID */
} light_ctl_set_params_t;

/** Message format for the CTL Temperature Set message. */
typedef struct
{
    uint32_t temperature32;                /**< Value of the Temperature32 state */
    uint16_t delta_uv;                     /**< Value of the Delta UV state */
    uint8_t tid;                           /**< Transaction ID */
} light_ctl_temperature_set_params_t;

/** Message format for the CTL Temperature Range Set message. */
typedef struct
{
    uint32_t temperature32_range_min;     /**< Value of the Temperature32 range min state */
    uint32_t temperature32_range_max;     /**< Value of the Temperature32 range max state */
} light_ctl_temperature_range_set_params_t;

/** Message format for the CTL Default Set message. */
typedef struct
{
    uint16_t lightness;                    /**< Value of the Lightness Default state */
    uint32_t temperature32;                /**< Value of the Temperature32 Default state */
    uint16_t delta_uv;                     /**< Value of the Delta UV Default state */
} light_ctl_default_set_params_t;

/**
 * Message format for the CTL delta set "message".
 * There is no official delta set message for the CTL server.
 * This structure supports internal messages, specifically for server<->app interaction.
 */
typedef struct
{
    int32_t delta_temperature;             /**< Value of the Temperature delta value */
    uint8_t tid;                           /**< Transaction ID */
} light_ctl_temperature_delta_set_params_t;

/**
 * Message format for the Light CTL Temperature Move Set "message".
 * There is no official move set message for the CTL server.
 * This structure supports internal messages, specifically for server<->app interaction.
 */
typedef struct
{
    int32_t delta_temperature;            /**< Value of the temperature move value */
    uint8_t tid;                          /**< Transaction ID */
} light_ctl_temperature_move_set_params_t;

/** Parameters for the Light CTL Status message. */
typedef struct
{
    uint16_t present_lightness;            /**< The present value of the Lightness state */
    uint32_t present_temperature32;        /**< The present value of the Temperature32 state */
    uint16_t target_lightness;             /**< The target value of the Lightness state (optional) */
    uint32_t target_temperature32;         /**< The target value of the Temperature32 state (optional) */
    uint32_t remaining_time_ms;            /**< Remaining time value in milliseconds */
} light_ctl_status_params_t;

/** Parameters for the Light CTL Temperature Status message. */
typedef struct
{
    uint32_t present_temperature32;        /**< The present value of the Temperature32 state */
    uint16_t present_delta_uv;             /**< The present value of the Delta UV state */
    uint32_t target_temperature32;         /**< The target value of the Temperature32 state (optional) */
    uint16_t target_delta_uv;              /**< The target value of the Delta UV state (optional) */
    uint32_t remaining_time_ms;            /**< Remaining time value in milliseconds */
} light_ctl_temperature_status_params_t;

/** Parameters for the Light CTL Temperature Range Status message. */
typedef struct
{
    uint8_t status_code;                   /**< Status code for the requesting message */
    uint32_t temperature32_range_min;      /**< The value of Temperature32 Range min state */
    uint32_t temperature32_range_max;      /**< The value of Temperature32 Range max state */
} light_ctl_temperature_range_status_params_t;

/** Parameters for the Light CTL Default Status message. */
typedef struct
{
    uint16_t lightness;                    /**< The value of the Lightness state */
    uint32_t temperature32;                /**< The value of the Temperature32 state */
    uint16_t delta_uv;                     /**< The value of the Delta UV state */
} light_ctl_default_status_params_t;

/** Storage format for model states while booting - the stored values will be read and passed to
 * the model to do any state binding and set the current states based on these stored values. */
typedef struct
{
    /** Stores OnPowerUp state value */
    uint8_t onpowerup;
    /** Stores represented CTL Temperature state value */
    uint32_t temperature32;
    /** Stores represented Temperature32 state value */
    uint32_t default_temperature32;
    /** Stores Delta UV state value */
    uint16_t delta_uv;
    /** Stores Default UV state value */
    uint16_t default_delta_uv;
    /** Stores represented Temperature range value */
    light_ctl_temperature_range_set_params_t temperature32_range;
} light_ctl_saved_values_t;

/**@} end of LIGHT_CTL_MODELS */
#endif /* LIGHT_CTL_COMMON_H__ */
