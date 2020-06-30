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

#ifndef SENSOR_UTILS_H__
#define SENSOR_UTILS_H__

#include <stdint.h>

#include "utils.h"
#include "access_config.h"
#include "log.h"
#include "nrf_mesh_assert.h"

/* Common sensor status indices */
#define SENSOR_PROP_ID_INDEX0 0
#define SENSOR_PROP_ID_INDEX1 1

/* Sensor settings status constant */
#define SENSOR_SETTINGS_SPID_INDEX 2

/* Sensor setting status constants */
#define SENSOR_SETTING_PROP_ID_INDEX0 2
#define SENSOR_SETTING_PROP_ID_INDEX1 3
#define SENSOR_SETTING_ACCESS_INDEX 4
#define SENSOR_SETTING_RAW_INDEX 5


/* Constants used to pack/unpack status messages. */
#define SENSOR_MPID_A_BYTES 2
#define SENSOR_MPID_B_BYTES 3

#define SENSOR_FORMAT_A_BIT 0
#define SENSOR_FORMAT_B_BIT 1

#define SENSOR_MPID_B_ZERO_DATA_BYTES (0x7F)

/**
 * @defgroup SENSOR_UTILS Sensor model utility functions
 * @ingroup SENSOR_MODEL
 * 
 * Implements the various utilities needed at various locations in the sensor code, from model to
 * mid app to top app
 * @{
 */

/** Format A of the Marshalled Property ID (MPID) field 
 */
typedef struct __attribute((packed))
{
    uint16_t format      : 1;       /**< Format A tag, 0b0 */
    uint16_t length      : 4;       /**< Length of the Property Value */
    uint16_t property_id : 11;      /**< Property identifying a sensor */
} sensor_mpid_a_t;

/** Format B of the Marshalled Property ID (MPID) field
 */
typedef struct __attribute((packed))
{
    uint8_t  format       : 1;      /**< Format B tag, 0b1 */
    uint8_t  length       : 7;      /**< Length of the Property Value */
    uint16_t property_id  : 16;     /**< Property identifying a sensor */
} sensor_mpid_b_t;

/**
 * Look through the supported property_id list to check for property_id support
 *
 * @param[in] p_properties      Pointer to an array of all supported properties.
 * @param[in] property_id       The Property ID to look for.
 *
 * @returns true if property_id is found, false otherwise
 */
static inline bool sensor_property_id_supported(uint16_t * p_properties, uint16_t property_id)
{
    NRF_MESH_ASSERT(p_properties);

    /* The 0th element of p_properties gives the number of properties it defines.
     */
    for (uint16_t i = 1; i <= p_properties[0]; i++)
    {
        if (property_id == p_properties[i])
        {
            return true;
        }
    }

    __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "property_id 0x%04x not supported\n", property_id);
    return false;
}

/**
 * Gets the publish period for the given model in microseconds.
 *
 * @param[in] model_handle      Model handle assigned to this instance.
 *
 * @returns publish period in microseconds, or 0 if no publish period is set.
 */
static inline uint64_t publish_period_get(access_model_handle_t model_handle)
{
    access_publish_resolution_t publish_resolution;
    uint8_t publish_steps = 0;
    uint32_t status = access_model_publish_period_get(model_handle, &publish_resolution, &publish_steps);
    
    uint64_t resolution_ms = 0;

    if (NRF_SUCCESS == status)
    {
        switch (publish_resolution) {
        case ACCESS_PUBLISH_RESOLUTION_100MS:
            resolution_ms = 100;
            break;
        case ACCESS_PUBLISH_RESOLUTION_1S:
            resolution_ms = 1000;
            break;
        case ACCESS_PUBLISH_RESOLUTION_10S:
            resolution_ms = 10000;
            break;
        case ACCESS_PUBLISH_RESOLUTION_10MIN:
            resolution_ms = 10000 * 6;
            break;
        default:
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                  "invalid publish resolution (%d) = (value).\n",
                  publish_resolution);
        }
    }

    /* If access_model_publish_period_get() returns with failure or with publish_steps set to 0 or
     * with undefined publish_resolution, then the return value is 0. This means that no publish
     * period is set.
     */
    return MS_TO_US(resolution_ms * publish_steps);
}


/**
 * Create format A of the Marshalled Property ID (MPID) field.
 *
 * See @tagMeshMdlSp section 4.2.14 for more details.
 *
 * @param[in]  property_id      The Property ID identifying a sensor.
 * @param[in]  data_bytes       Length of the Property Value
 * @param[out] p_buffer         Pointer to buffer where the Marshalled Sensor data will be stored.
 */
static inline void sensor_mpid_a_create(uint16_t property_id,
                                       uint8_t data_bytes,
                                       uint8_t * p_buffer)
{
    sensor_mpid_a_t * p_mpid = (sensor_mpid_a_t *)p_buffer;

    p_mpid->format = SENSOR_FORMAT_A_BIT;
    p_mpid->length = data_bytes;
    p_mpid->property_id = property_id;
}

/**
 * Create format B of the Marshalled Property ID (MPID) field.
 *
 * See @tagMeshMdlSp section 4.2.14 for more details.
 *
 * @param[in]  property_id      The Property ID identifying a sensor.
 * @param[in]  data_bytes       Length of the Property Value
 * @param[out] p_buffer         Pointer to buffer where the Marshalled Sensor data will be stored.
 */
static inline void sensor_mpid_b_create(uint16_t property_id,
                                       uint8_t data_bytes,
                                       uint8_t * p_buffer)
{
    sensor_mpid_b_t * p_mpid = (sensor_mpid_b_t *)p_buffer;

    p_mpid->format = SENSOR_FORMAT_B_BIT;
    p_mpid->length = data_bytes;
    p_mpid->property_id = property_id;
}

/**@} end of SENSOR_UTILS */
#endif /* SENSOR_UTILS_H__ */
