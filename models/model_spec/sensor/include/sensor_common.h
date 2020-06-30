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

/* See "@tagMeshDevPr, section 4.1.3".  0 is a prohibited value for property ID, so it can represent
 * "no property id" without concern for a collision.
 */
#define SENSOR_NO_PROPERTY_ID (0)

/* "@tagMeshDevPr"
 */
/* Motion Sensed Property
 */
#define SENSOR_MOTION_SENSED_PROPERTY_ID (0x0042)
/* "Presence Detected Property
 */
#define SENSOR_PRESENCE_DETECT_PROPERTY_ID (0x004D)

/* @tagMeshMdlSp section 4.1.1 descriptor is 8 bytes
 */
#define SENSOR_DESCRIPTOR_MSG_SIZE (8)

typedef uint8_t * sensor_property_data_size_t;

/**
 * Unpacked message structure typedefs are used for API interfaces and for implementing model code.
 * This helps to minimize codefootprint.
 */

/** Parameters for the Sensor Descriptor Get message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
} sensor_descriptor_get_params_t;


/** Parameters for the Sensor Cadence Get message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
} sensor_cadence_get_params_t;

/** Parameters for the Sensor Settings Get message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
} sensor_settings_get_params_t;

/** Parameters for the Sensor Settings Status message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
    /**< Sequence of Setting Property IDs identifying settings within a sensor
     */
    uint16_t * setting_property_ids;
} sensor_settings_status_params_t;

/** Parameters for the Sensor Setting Get message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
    /**< Setting Property ID identifying a setting within a sensor
     */
    uint16_t setting_property_id;
} sensor_setting_get_params_t;

/** Parameters for the Sensor Setting Set message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
    /**< Setting Property ID identifying a setting within a sensor
     */
    uint16_t setting_property_id;
    /**< Raw value for the setting
     */
    uint8_t setting_raw;
} sensor_setting_set_params_t;

/** Parameters for the Sensor Setting Status message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
    /**< Setting Property ID identifying a setting within a sensor
     */
    uint16_t setting_property_id;
    /**< Read/Write access rights for the setting (Optional)
     */
    uint8_t setting_access;
    /**< Raw value for the setting
     */
    uint8_t setting_raw;
} sensor_setting_status_params_t;

/** Parameters for the Sensor Get message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
} sensor_get_params_t;

/** Parameters for the Sensor Column Get message. */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
    /**< Raw value identifying a column
     */
    uint8_t raw_value_x;
} sensor_column_get_params_t;

/** Parameters for the Sensor Column Status message.
 */
typedef struct
{
    /**< Identifies a sensor  and the Y axis
     */
    uint16_t property_id;
    /**< Raw value representing the left corner of the column on the X axis
     */
    uint8_t raw_value_x;
    /**< Raw value representing the width of the column (Optional)
     */
    uint8_t column_width;
    /**< Raw value representing the height of the column on the Y axis
     */
    uint8_t raw_value_y;
} sensor_column_status_params_t;

/** Parameters for the Sensor Series Get message.
 */
typedef struct
{
    /**< Sensor property ID
     */
    uint16_t property_id;
    /**< Raw value identifying a starting column (Optional)
     */
    uint8_t raw_value_x1;
    /**< Raw value identifying an ending column
     */
    uint8_t raw_value_x2;
} sensor_series_get_params_t;

/** Parameters for the Sensor Series Status message.
 */
typedef struct
{
    /**< Identifies a sensor
     */
    uint16_t property_id;
    /**< The nth column X axis
     */
    uint8_t raw_value_x_n;
    /**< The nth column width
     */
    uint8_t column_width_n;
    /**< The nth column height
     */
    uint8_t raw_value_y_n;
} sensor_series_status_params_t;

/* just the property id in the error return
 */
#define SENSOR_SERIES_ERR_MSG_SIZE (2)
#define SENSOR_CADENCE_ERR_MSG_SIZE (2)
#define SENSOR_SETTINGS_ERR_MSG_SIZE (2)
#define SENSOR_SETTING_ERR_MSG_SIZE (4)
#define SENSOR_DESCRIPTOR_ERR_MSG_SIZE (2)

/** Sensor Descriptor - the sensor-specific sensor descriptor
 */
typedef struct
{
    /**< Property ID for sensor
     */
    uint16_t property_id;
    /**< 12-bit value for possible positive sensor error
     */
    uint16_t positive_tolerance;
    /**< 12-bit value for possible negative sensor error
     */
    uint16_t negative_tolerance;
    /**< Sampling function applied to measured sensor values
     */
    uint8_t sampling_function;
    /**< Period over which measurement is taken in seconds
     */
    uint8_t measurement_period;
    /**< Interval between measurement updates in seconds
     */
    uint8_t update_interval;
} sensor_descriptor_t;


/**@} end of SENSOR_MODEL */
#endif /* SENSOR_COMMON_H__ */
