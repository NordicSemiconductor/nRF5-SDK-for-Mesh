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

#ifndef LC_SERVER_COMMON_H__
#define LC_SERVER_COMMON_H__

#include <stdint.h>
#include "model_common.h"

/**
 * @defgroup LIGHT_LC_MODELS Light LC models
 * @ingroup MESH_API_GROUP_LIGHT_MODELS
 *
 * This model implements the message based interface required to set the LC Setup server states. The
 * model sends its state information to the mid app to be stored in flash (to be read at boot time)
 * and use it appropriately. The Light Lightness Setup server model must be used along with this
 * model for the complete LC Setup server model functionality. Refer to @ref APP_LIGHT_LC to see
 * how this can be done.
 * @{
 */

/** Model Company ID */
#define LIGHT_LC_SERVER_COMPANY_ID (0xFFFF)

/** Property size definitions
 *
 *  Although the property value field is a variable size, it is never bigger than 4 bytes. So we set
 *  4 bytes to be used as buffer size.
 */
#define LIGHT_LC_PROPERTY_BUF_SIZE (4)

/** Defines for LC Mode Off value. */
#define LIGHT_LC_MODE_OFF (0)

/**
 * Defines default value for the Light LC Mode state.
 *
 * The default value is defined in @tagMeshMdlSp section 6.2.3.1.
 */
#ifndef LIGHT_LC_DEFAULT_MODE
#define LIGHT_LC_DEFAULT_MODE (0x0)
#endif

/**
 * Defines default value for the Light LC Occupancy Mode state.
 *
 * The default value is defined in @tagMeshMdlSp section 6.2.3.2.
 */
#ifndef LIGHT_LC_DEFAULT_OCC_MODE
#define LIGHT_LC_DEFAULT_OCC_MODE (0x0)
#endif

/** Defines default value for the Light LC Light OnOff state */
#ifndef LIGHT_LC_DEFAULT_LIGHT_ONOFF
#define LIGHT_LC_DEFAULT_LIGHT_ONOFF (0x0)
#endif

/**
 * Defines default value for the Light Control Ambient LuxLevel On Property representing
 * the Ambient LuxLevel that determines if the controller transitions from the Standby to Run states.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_LUXLEVEL_ON
#define LIGHT_LC_DEFAULT_PR_LUXLEVEL_ON (75000)
#endif

/**
 * Defines default value for the Light Control Ambient LuxLevel Prolong Property representing
 * the Ambient LuxLevel in the Prolong state.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_LUXLEVEL_PROLONG
#define LIGHT_LC_DEFAULT_PR_LUXLEVEL_PROLONG (20000)
#endif

/**
 * Defines default value for the Light Control Ambient LuxLevel Standby Property representing
 * the lowest Ambient LuxLevel at the Standby state.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_LUXLEVEL_STANDBY
#define LIGHT_LC_DEFAULT_PR_LUXLEVEL_STANDBY (7000)
#endif

/**
 * Defines default value for the Light Control Lightness On Property representing
 * the minimum value for lightness in the Run and Occupancy states.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_LIGHTNESS_ON
#define LIGHT_LC_DEFAULT_PR_LIGHTNESS_ON (0xAFFF)
#endif

/**
 * Defines default value for the Light Control Lightness Prolong Property representing
 * the minimum value for lightness in the Prolong state.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_LIGHTNESS_PROLONG
#define LIGHT_LC_DEFAULT_PR_LIGHTNESS_PROLONG (0x3FFF)
#endif

/**
 * Defines default value for the Light Control Lightness Standby Property representing
 * the minimum light level for lightness in the Standby state.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_LIGHTNESS_STANDBY
#define LIGHT_LC_DEFAULT_PR_LIGHTNESS_STANDBY (0x1000)
#endif

/**
 * Defines default value for the Light Control Regulator Accuracy Property representing
 * the percentage accuracy of the Light LC PI Feedback Regulator.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_REGULATOR_ACCURACY
#define LIGHT_LC_DEFAULT_PR_REGULATOR_ACCURACY (4)
#endif

/**
 * Defines default value for the Light Control Regulator Kiu Property representing
 * the integral coefficient that determines the integral part of the equation defining
 * the output of the Light LC PI Feedback Regulator, when Light LC Ambient LuxLevel
 * is less than LuxLevel Out.
 *
 * The default value is defined in @tagMeshMdlSp section 6.2.4.14.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_REGULATOR_KIU
#define LIGHT_LC_DEFAULT_PR_REGULATOR_KIU (250.0)
#endif

/**
 * Defines default value for the Light Control Regulator Kid Property representing
 * the integral coefficient that determines the integral part of the equation defining
 * the output of the Light LC PI Feedback Regulator, when Light LC Ambient LuxLevel
 * is greater than or equal to the value of the LuxLevel Out state.
 *
 * The default value is defined in @tagMeshMdlSp section 6.2.4.15.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_REGULATOR_KID
#define LIGHT_LC_DEFAULT_PR_REGULATOR_KID (25.0)
#endif

/**
 * Defines default value for the Light Control Regulator Kpu Property representing
 * the proportional coefficient that determines the proportional part of
 * the equation defining the output of the Light LC PI Feedback Regulator,
 * when Light LC Ambient LuxLevel is less than the value of the LuxLevel Out state.
 *
 * The default value is defined in @tagMeshMdlSp section 6.2.4.16.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_REGULATOR_KPU
#define LIGHT_LC_DEFAULT_PR_REGULATOR_KPU (80.0)
#endif

/**
 * Defines default value for the Light Control Regulator Kpd Property representing
 * the proportional coefficient that determines the proportional part of the equation
 * defining the output of the Light LC PI Feedback Regulator, when Light LC Ambient
 * LuxLevel is greater than or equal to the value of the LuxLevel Out state.
 *
 * The default value is defined in @tagMeshMdlSp section 6.2.4.17.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_REGULATOR_KPD
#define LIGHT_LC_DEFAULT_PR_REGULATOR_KPD (80.0)
#endif

/**
 * Defines default value for the Light Control Time Fade Property that represents
 * the time in milliseconds a light takes to transition from the Run state to the Prolong state
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_TIME_FADE_MS
#define LIGHT_LC_DEFAULT_PR_TIME_FADE_MS (4500)
#endif

/**
 * Defines default value for the Light Control Time Fade On Property that represents
 * the time in milliseconds a light takes to transition from the Standby state to the Run state.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_TIME_FADE_ON_MS
#define LIGHT_LC_DEFAULT_PR_TIME_FADE_ON_MS (2000)
#endif

/**
 * Defines default value for the Light Control Time Fade Standby Property that represents
 * the time in milliseconds a light transition from the Prolong state to the Standby state
 * when the transition is automatic (such as when triggered by an occupancy or light sensor).
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_AUTO_MS
#define LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_AUTO_MS (3500)
#endif

/**
 * Defines default value for the Light Control Time Fade Standby Manual Property that represents
 * the time in milliseconds a light take to transition from the Prolong state to the Standby state
 * when the transition is triggered by a manual operation (for example by a user operating a light switch).
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_MANUAL_MS
#define LIGHT_LC_DEFAULT_PR_TIME_FADE_STANDBY_MANUAL_MS (3500)
#endif

/**
 * Defines default value for the Light Control Time Prolong Property that represents
 * the duration of the Prolong state which is the state of a device between its run state
 * and its standby state.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_TIME_PROLONG_MS
#define LIGHT_LC_DEFAULT_PR_TIME_PROLONG_MS (7000)
#endif

/**
 * Defines default value for the Light Control Time Run On Property that represents
 * the duration in milliseconds of the Run state after last occupancy was detected
 * (the occupancy input stopped detecting active occupancy information).
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_TIME_RUN_ON_MS
#define LIGHT_LC_DEFAULT_PR_TIME_RUN_ON_MS (10000)
#endif

/**
 * Defines default value for the Light Control Occupancy Delay Property that represents
 * the time delay in milliseconds between receiving a signal from an occupancy sensor
 * (Sensor Status message) and a light controller executing a state change as a result of the signal.
 *
 * See @link_MeshProperties about the data format of the characteristic referenced by this property.
 */
#ifndef LIGHT_LC_DEFAULT_PR_TIME_OCCUPANCY_DELAY_MS
#define LIGHT_LC_DEFAULT_PR_TIME_OCCUPANCY_DELAY_MS (0)
#endif

/**
 * Product-specific value for the summation interval used by the Light LC PI Feedback Regulator.
 *
 * Shall be within [@ref LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MIN_MS, @ref LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MAX_MS] range.
 */
#ifndef LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MS
#define LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MS     (100)
#endif

/** Defines minimum value for the summation interval used by the Light LC Pi Feedback Regulator */
#define LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MIN_MS (10)

/** Defines maximum value for the summation interval used by the Light LC Pi Feedback Regulator */
#define LIGHT_LC_LIGHT_PI_SUMMATION_INTERVAL_MAX_MS (100)

/**
 * Conversion divisor from a lux level to a linear lightness level.
 *
 * This conversion divisor is a tuning feature that allows the vendor to specify a divisor instead
 * of a conversion function. The divisor changes based on the sensitivity and the general distance
 * of the light sensor from the light, and the angle at which the light hits the sensor.
 *
 * @note This variable is not defined in the mesh model specification and there is no definition
 * of a conversion function to the lightness level.
 */
#ifndef LIGHT_LC_LIGHT_PI_CONVERSION_DIVISOR
#define LIGHT_LC_LIGHT_PI_CONVERSION_DIVISOR        (50)
#endif

/**
 * Enum for to determine which state data variable is being set/gotten.
 */
typedef enum
{
    LIGHT_LC_STATE_NULL_ENTRY,
    LIGHT_LC_STATE_LIGHT_LC_MODE,
    LIGHT_LC_STATE_LIGHT_LC_OCC_MODE,
    LIGHT_LC_STATE_LIGHT_LC_LIGHT_ONOFF,
    LIGHT_LC_STATE_AMBIENT_LUXLEVEL_ON,
    LIGHT_LC_STATE_AMBIENT_LUXLEVEL_PROLONG,
    LIGHT_LC_STATE_AMBIENT_LUXLEVEL_STANDBY,
    LIGHT_LC_STATE_LIGHTNESS_ON,
    LIGHT_LC_STATE_LIGHTNESS_PROLONG,
    LIGHT_LC_STATE_LIGHTNESS_STANDBY,
    LIGHT_LC_STATE_REGULATOR_ACCURACY,
    LIGHT_LC_STATE_REGULATOR_KID,
    LIGHT_LC_STATE_REGULATOR_KIU,
    LIGHT_LC_STATE_REGULATOR_KPD,
    LIGHT_LC_STATE_REGULATOR_KPU,
    LIGHT_LC_STATE_TIME_FADE,
    LIGHT_LC_STATE_TIME_FADE_ON,
    LIGHT_LC_STATE_TIME_FADE_STANDBY_AUTO,
    LIGHT_LC_STATE_TIME_FADE_STANDBY_MANUAL,
    LIGHT_LC_STATE_TIME_OCCUPANCY_DELAY,
    LIGHT_LC_STATE_TIME_PROLONG,
    LIGHT_LC_STATE_TIME_RUN_ON,
} light_lc_state_t;


/**
 * Unpacked message structure typedefs are used for API interfaces and
 * for implementing model code. This helps to minimize code footprint.
 */

/** Message format for the Light LC Mode set message. */
typedef struct
{
    bool mode;                     /**< The target value of the Mode state */
} light_lc_mode_set_params_t;

/** Message format for the Light LC Occupancy Mode set message. */
typedef struct
{
    bool occupancy_mode;           /**< The target value of the Occupancy Mode state */
} light_lc_occupancy_mode_set_params_t;

/** Message format for the Light LC Light OnOff set message. */
typedef struct
{
    bool light_on_off;             /**< The target value of the Light LC Light OnOff state */
    uint8_t tid;                   /**< Transaction ID */
} light_lc_light_onoff_set_params_t;

/** Message format for the Light LC Property set message. */
typedef struct
{
    uint16_t property_id;          /**< Property ID identifying a Light LC Property */
    /* Although the property value field is a variable size, it is
     * never bigger than 4 bytes, so we put a 4 byte buffer here to
     * use */
    uint8_t property_buffer[LIGHT_LC_PROPERTY_BUF_SIZE];
} light_lc_property_set_params_t;

/** Message format for the Light LC Property get message. */
typedef struct
{
    uint16_t property_id;          /**< Property ID identifying a Light LC Property */
} light_lc_property_get_params_t;

/** Parameters for the Light LC Mode Status message. */
typedef struct
{
    bool mode;                     /**< The present value of the Light LC Mode state */
} light_lc_mode_status_params_t;

/** Parameters for the Light LC Occupancy Mode Status message. */
typedef struct
{
    bool occupancy_mode;           /**< The present value of the Occupancy Mode state */
} light_lc_occupancy_mode_status_params_t;

/** Parameters for the Light LC Light OnOff Status message. */
typedef struct
{
    bool present_light_onoff;      /**< The present value of the Light OnOff state */
    bool target_light_onoff;       /**< The target value of the Light OnOff state */
    uint32_t remaining_time_ms;    /**< Remaining transition time - encoded */
} light_lc_light_onoff_status_params_t;

/** Parameters for the Light LC Property Status message. */
typedef struct
{
    uint16_t property_id;          /**< Property ID identifying a Light LC Property */
    /* Although the property value field is a variable size, it is
     * never bigger than 4 bytes, so we put a 4 byte buffer here to
     * use */
    uint8_t property_buffer[LIGHT_LC_PROPERTY_BUF_SIZE];
} light_lc_property_status_params_t;

/**@} end of LIGHT_LC_MODELS */
#endif /* LC_SERVER_COMMON_H__ */
