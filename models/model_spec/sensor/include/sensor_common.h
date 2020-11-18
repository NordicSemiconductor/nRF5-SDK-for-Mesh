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

#ifndef SENSOR_COMMON_H__
#define SENSOR_COMMON_H__

#include <stdint.h>


/**
 * @defgroup SENSOR_MODEL Sensor model
 * @ingroup MESH_API_GROUP_SENSOR_MODELS
 * This model implements the message based interface required to set the Sensor value on the server.
 * Server model itself is a stateless model. The state information resides in the user application.
 * @{
 */

/** Model Company ID */
#define SENSOR_COMPANY_ID (0xFFFF)

/**
 * Prohibited Property ID.
 *
 * See "@tagMeshDevPr, section 4.1.3".
 */
#define SENSOR_NO_PROPERTY_ID (0)

/**
 * Motion Sensed Property ID.
 *
 * See "@tagMeshDevPr, section 4.1.3".
 */
#define SENSOR_MOTION_SENSED_PROPERTY_ID (0x0042)

/**
 * Presence Detected Property ID.
 *
 * See "@tagMeshDevPr, section 4.1.3".
 */
#define SENSOR_PRESENCE_DETECT_PROPERTY_ID (0x004D)

/**
 * Sensor Descriptor size.
 *
 * See @tagMeshMdlSp, section 4.1.1.
 */
#define SENSOR_DESCRIPTOR_MSG_SIZE (8)

typedef uint8_t * sensor_property_data_size_t;

/**
 * Unpacked message structure typedefs are used for API interfaces and for implementing model code.
 * This helps to minimize codefootprint.
 */

/** Parameters for the Sensor Descriptor Get message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
} sensor_descriptor_get_params_t;

/** Parameters for the Sensor Cadence Get message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
} sensor_cadence_get_params_t;

/** Parameters for the Sensor Settings Get message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
} sensor_settings_get_params_t;

/** Parameters for the Sensor Settings Status message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
    uint16_t * setting_property_ids;    /**< Sequence of Setting Property IDs identifying settings within a sensor */
} sensor_settings_status_params_t;

/** Parameters for the Sensor Setting Get message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
    uint16_t setting_property_id;       /**< Setting Property ID identifying a setting within a sensor */
} sensor_setting_get_params_t;

/** Parameters for the Sensor Setting Set message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
    uint16_t setting_property_id;       /**< Setting Property ID identifying a setting within a sensor */
    uint8_t setting_raw;                /**< Raw value for the setting */
} sensor_setting_set_params_t;

/** Parameters for the Sensor Setting Status message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
    uint16_t setting_property_id;       /**< Setting Property ID identifying a setting within a sensor */
    uint8_t setting_access;             /**< Read/Write access rights for the setting (Optional) */
    uint8_t setting_raw;                /**< Raw value for the setting */
} sensor_setting_status_params_t;

/** Parameters for the Sensor Get message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
} sensor_get_params_t;

/** Parameters for the Sensor Column Get message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
    uint8_t raw_value_x;                /**< Raw value identifying a column */
} sensor_column_get_params_t;

/** Parameters for the Sensor Column Status message. */
typedef struct
{
    uint16_t property_id;               /**< Identifies a sensor  and the Y axis */
    uint8_t raw_value_x;                /**< Raw value representing the left corner of the column on the X axis */
    uint8_t column_width;               /**< Raw value representing the width of the column (Optional) */
    uint8_t raw_value_y;                /**< Raw value representing the height of the column on the Y axis */
} sensor_column_status_params_t;

/** Parameters for the Sensor Series Get message. */
typedef struct
{
    uint16_t property_id;               /**< Sensor property ID */
    uint8_t raw_value_x1;               /**< Raw value identifying a starting column (Optional) */
    uint8_t raw_value_x2;               /**< Raw value identifying an ending column */
} sensor_series_get_params_t;

/** Parameters for the Sensor Series Status message. */
typedef struct
{
    uint16_t property_id;               /**< Identifies a sensor*/
    uint8_t raw_value_x_n;              /**< The nth column X axis */
    uint8_t column_width_n;             /**< The nth column width */
    uint8_t raw_value_y_n;              /**< The nth column height */
} sensor_series_status_params_t;

/** Parameters for the Sensor Descriptor. */
typedef struct
{
    uint16_t property_id;               /**< Property ID for sensor */
    uint16_t positive_tolerance;        /**< 12-bit value for possible positive sensor error */
    uint16_t negative_tolerance;        /**< 12-bit value for possible negative sensor error */
    uint8_t sampling_function;          /**< Sampling function applied to measured sensor values */
    uint8_t measurement_period;         /**< Period over which measurement is taken in seconds */
    uint8_t update_interval;            /**< Interval between measurement updates in seconds */
} sensor_descriptor_t;

/**@} end of SENSOR_MODEL */
#endif /* SENSOR_COMMON_H__ */
