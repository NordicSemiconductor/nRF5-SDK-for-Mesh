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
#include "app_sensor_utils.h"

#include <stdint.h>

#include "nrf_mesh_assert.h"
#include "log.h"

#include "access_config.h"
#include "mesh_mem.h"

#include "sensor_common.h"
#include "sensor_utils.h"


/** Structures and constants for the sensor cadence */
#define CADENCE_DIVISOR_MASK (0x7F)
#define CADENCE_TRIGGER_MASK (0x1)
#define CADENCE_TRIGGER_SHIFT (7)

#define MPID_A_MAX_DATA_BYTES (16)
#define FORMAT_A_MAX_PROPERTY_ID (2047)
#define FORMAT_MASK (0x01)

/* 4 bytes in the non-variable part of the cadence cmd */
#define SERIALIZED_CADENCE_SCALAR_BYTES (4)
#define SERIALIZED_CADENCE_DELTA_VECTORS (2)
#define SERIALIZED_CADENCE_VALUE_VECTORS (2)

#define MAXIMUM_MINIMUM_STATUS_INTERVAL_VALUE (26)
#define MAXIMIM_FAST_CADENCE_PERIOD_DIVISOR_VALUE (15)

static uint64_t m_minimum_publish_interval;
static uint8_t * mp_marshalled_data;

/* static function definitions.
 */
static uint16_t range_vector_bytes_get(uint16_t property_id)
{
    switch (property_id) {

    case SENSOR_MOTION_SENSED_PROPERTY_ID:   /* Intentionally fall through. */
    case SENSOR_PRESENCE_DETECT_PROPERTY_ID:
        return sizeof(pir_data_size_t);

    default:
        /* Property ID not supported */
        return 0;
    }
}

static uint16_t delta_vector_bytes_get(uint16_t property_id, uint8_t trigger_type)
{
    switch (property_id) {

    case SENSOR_MOTION_SENSED_PROPERTY_ID:   /* Intentionally fall through. */
    case SENSOR_PRESENCE_DETECT_PROPERTY_ID:
        return trigger_type ? sizeof(uint16_t) : sizeof(pir_data_size_t);

    default:
        /* Property ID not supported */
        return 0;
    }
}

static inline uint16_t delta_vector_bytes_allocated_get(uint16_t property_id)
{
    uint16_t bytes_trigger_type_0 = delta_vector_bytes_get(property_id, 0);
    uint16_t bytes_trigger_type_1 = delta_vector_bytes_get(property_id, 1);
    return MAX(bytes_trigger_type_0, bytes_trigger_type_1);
}

static uint16_t cadence_serialized_bytes(sensor_cadence_t * p)
{
    /*                                            bits     bytes
     * Property ID 16 Property ID for the sensor. 16         2
     * Fast Cadence Period Divisor                 7
     * Status Trigger Type                         1         1
     * Status Trigger Delta Down variable        variable   vector
     * Status Trigger Delta Up variable          variable   vector
     * Status Min Interval                         8         1
     * Fast Cadence Low variable                 variable   vector
     * Fast Cadence High variable                varaible   vector
    */

    /* 4.1.3.2
     *  Status Trigger Type:
     *  The Status Trigger Type field shall define the unit and format of the Status Trigger Delta Down
     *  and the Status Trigger Delta Up fields.
     *    -  The value of 0b0 means that the format shall be defined by the Format Type of the
     *       characteristic that the Sensor Property ID state references (see Section 4.1.1.1).
     *    -  The value of 0b1 means that the unit is «unitless», the format type is 0x06 (uint16), and the
     *       value is represented as a percentage change with a resolution of 0.01 percent.
     */

    return SERIALIZED_CADENCE_SCALAR_BYTES
        + SERIALIZED_CADENCE_DELTA_VECTORS * delta_vector_bytes_get(p->property_id, p->trigger_type)
        + SERIALIZED_CADENCE_VALUE_VECTORS * p->range_value_bytes_allocated;
}

static uint32_t cadence_update(sensor_cadence_t * p, uint8_t * p_in, uint16_t bytes)
{
    if (p == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (bytes <= sizeof(uint16_t))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    /* The serialized data at p_in begins with a property id.
     */

    uint16_t property_id;
    memcpy(&property_id, p_in, sizeof(uint16_t));

    if (property_id != p->property_id)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: the cadence instance ID (%d) != update ID (%d)\n",
              p->property_id,
              property_id);
        return NRF_ERROR_INVALID_DATA;
    }

    /* Consider data after the property id as a uint8_t array.
     */
    bytes -= sizeof(uint16_t);
    uint16_t offset = 0;
    p_in = &p_in[sizeof(uint16_t)];

    /* 4.1.3.1 Fast Cadence Period Divisor:
     * "The valid range for the Fast Cadence Period Divisor state is 0–15 and other values are
     * Prohibited."
     */
    if ((p_in[offset] & CADENCE_DIVISOR_MASK) > MAXIMIM_FAST_CADENCE_PERIOD_DIVISOR_VALUE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: prohibited value for cadence divisor ((0x%x (%d), %d) = (value, offset)\n",
              p_in[offset] & CADENCE_DIVISOR_MASK,
              p_in[offset] & CADENCE_DIVISOR_MASK,
              offset);
        return NRF_ERROR_INVALID_DATA;
    }

    uint8_t fast_period_exponent = (p_in[offset] & CADENCE_DIVISOR_MASK);
    uint8_t trigger_type = (p_in[offset++] >> CADENCE_TRIGGER_SHIFT) & CADENCE_TRIGGER_MASK;

    uint16_t delta_vector_bytes = delta_vector_bytes_get(p->property_id, trigger_type);
    if ((offset + delta_vector_bytes) > bytes)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    uint8_t * p_trigger_delta_down = &p_in[offset];
    offset += delta_vector_bytes;

    if ((offset + delta_vector_bytes) > bytes)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    uint8_t * p_trigger_delta_up = &p_in[offset];
    offset += delta_vector_bytes;

    if (offset > bytes)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    /* 4.1.3.5 Status Min Interval:
     * "The valid range for the Status Min Interval is 0–26 and other values are Prohibited."
     */
    if (p_in[offset] > MAXIMUM_MINIMUM_STATUS_INTERVAL_VALUE)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: prohibited value for Status Min Interval (0x%x (%d), %d) = (value, offset)\n",
              p_in[offset],
              p_in[offset],
              offset);
        return NRF_ERROR_INVALID_DATA;

    }
    uint8_t min_interval_exponent = p_in[offset++];

    if ((offset + p->range_value_bytes_allocated) > bytes)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    uint8_t * p_fast_cadence_low = &p_in[offset];
    offset += p->range_value_bytes_allocated;

    if ((offset + p->range_value_bytes_allocated) > bytes)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    uint8_t * p_fast_cadence_high = &p_in[offset];


    p->fast_period_exponent  = fast_period_exponent;
    p->trigger_type          = trigger_type;
    memcpy(p->p_trigger_delta_down, p_trigger_delta_down, delta_vector_bytes);
    memcpy(p->p_trigger_delta_up,   p_trigger_delta_up,   delta_vector_bytes);
    p->min_interval_exponent = min_interval_exponent;
    memcpy(p->p_fast_cadence_low,  p_fast_cadence_low,  p->range_value_bytes_allocated);
    memcpy(p->p_fast_cadence_high, p_fast_cadence_high, p->range_value_bytes_allocated);

    return NRF_SUCCESS;
}

static bool motion_sensor_in_fast_region(sensor_cadence_t * p)
{
    NRF_MESH_ASSERT(p);

    /* Percentage8 data representations specify the current value and the range.
     */
    uint8_t value = *p->p_current_value;

    /* @tagMeshMdlSp Section 4.1.3:
     * "If the Fast Cadence High value is equal or higher than the Fast Cadence Low value, and the
     * measured value is within the closed interval [Fast Cadence Low, Fast Cadence High], the Sensor
     * Status messages are published more frequently."
     *
     * "If the Fast Cadence High value is lower than the Fast Cadence Low value, and the measured
     * value is lower than the Fast Cadence High value or is higher than the Fast Cadence Low value,
     * the Sensor Status messages are published more frequently."
     */
    return  (*p->p_fast_cadence_high >= *p->p_fast_cadence_low)
        ? (value >= *p->p_fast_cadence_low) && (value <= *p->p_fast_cadence_high)
        : (value > *p->p_fast_cadence_low) || (value < *p->p_fast_cadence_high);
}


static bool motion_sensor_delta_value(uint16_t current,
                                      uint16_t previous,
                                      uint8_t  delta_up,
                                      uint8_t  delta_down)
{
    /* "... the format shall be defined by the Format Type of the characteristic that the Sensor
     * Property ID state references (see Section 4.1.1.1)."
     *
     * "The message shall be published when the value of the measured quantity is lower than the
     * previously published value decremented by the value of the Status Trigger Delta Down state
     * (see Section 4.1.3.3) or when it is higher than the previously published value incremented
     * by the value of the Status Trigger Delta Up state (see Section 4.1.3.4)."
     *
     */

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
              "Formatted trigger: current: %d, previous: %d, delta up: %d, delta down: %d, diff: %d\n",
              current, previous, delta_up, delta_down, current > previous ? current - previous : previous - current);

    return (current > previous) ? ((current - previous) >= delta_up)
                                : ((previous - current) >= delta_down);
}

static bool motion_sensor_delta_percent(uint16_t current,
                                        uint16_t previous,
                                        uint16_t delta_up,
                                        uint16_t delta_down)
{
    if (!previous)
    {
        return true;
    }

    uint16_t trigger;
    uint16_t difference;

    if (previous > current)
    {
        trigger = delta_down;
        difference = previous - current;
    }
    else
    {
        trigger = delta_up;
        difference = current - previous;
    }

    uint16_t diff_percentage = 100 * ((100 * difference) / previous);

    __LOG(LOG_SRC_APP, LOG_LEVEL_DBG1,
          "Unitless trigger: current: %d, previous: %d, delta up: %d, delta down: %d, diff percentage: %d\n",
          current, previous, delta_up, delta_down, diff_percentage);

    return diff_percentage >= trigger;
}

static bool motion_sensor_delta_trigger_fast(sensor_cadence_t * p)
{
    NRF_MESH_ASSERT(p);

    /*
     * Reference @tagMeshMdlSp, Section 4.1.3:
     * "... the Status Trigger Type (see Section 4.1.3.2), Status Trigger Delta Down (see Section
     * 4.1.3.3), and the Status Trigger Delta Up (see Section 4.1.3.4) ..." values specify fast
     * cadence triggers based upon rate of change.
     */
    if (p->trigger_type)
    {
        /*
         * Section 4.1.3.2:
         * For trigger_type = 1:
         * " ... the unit is "unitless", the format type is 0x06 (uint16), and the value is
         * represented as a percentage change with a resolution of 0.01 percent.
         */
        return motion_sensor_delta_percent((uint16_t)*p->p_current_value,
                                           (uint16_t)*p->p_previous_value,
                                           *(uint16_t *)p->p_trigger_delta_up,
                                           *(uint16_t *)p->p_trigger_delta_down);
    }
    else
    {
        /* "... the format shall be defined by the Format Type of the characteristic that the Sensor
         * Property ID state references (see Section 4.1.1.1)."
         */
        return motion_sensor_delta_value((uint16_t)*p->p_current_value,
                                         (uint16_t)*p->p_previous_value,
                                         *p->p_trigger_delta_up,
                                         *p->p_trigger_delta_down);
    }
}

static uint16_t mpid_a_value_marshall(sensor_cadence_t * p, uint8_t * buffer, uint16_t buffer_bytes)
{
    /* @tagMeshMdlSp section 4.2.14 Sensor Status defines the marshalled data format.
     */

    if (buffer_bytes < (SENSOR_MPID_A_BYTES + p->range_value_bytes_allocated))
    {
        return 0;
    }

    /* Copy the data into the buffer after the location of the MPID.
     */
    for (uint16_t i = 0; i < p->range_value_bytes_allocated; i++)
    {
        buffer[i + SENSOR_MPID_A_BYTES] = p->p_current_value[i];
    }

    /* Length is defined as 0-0xF representing 1-16, therefore subtract 1 before storing in the
       marshalled structure.
    */
    sensor_mpid_a_create(p->property_id, p->range_value_bytes_allocated - 1, buffer);

    return SENSOR_MPID_A_BYTES + p->range_value_bytes_allocated;
}


static uint16_t mpid_b_value_marshall(sensor_cadence_t * p, uint8_t * buffer, uint16_t buffer_bytes)
{
    /* @tagMeshMdlSp section 4.2.14 Sensor Status defines the marshalled data format.
     */

    if (buffer_bytes < (SENSOR_MPID_B_BYTES + p->range_value_bytes_allocated))
    {
        return 0;
    }

    /* Copy the data into the buffer after the location of the MPID.
     */
    for (uint16_t i = 0; i < p->range_value_bytes_allocated; i++)
    {
        buffer[i + SENSOR_MPID_B_BYTES] = p->p_current_value[i];
    }

    /* Length is defined as 0-0x7F representing 1-127, 0x7F = 0.
     */
    sensor_mpid_b_create(p->property_id,
                         p->range_value_bytes_allocated
                             ? p->range_value_bytes_allocated - 1 : SENSOR_MPID_B_ZERO_DATA_BYTES,
                         buffer);

    return SENSOR_MPID_B_BYTES + p->range_value_bytes_allocated;
}

static uint8_t * cadence_serialize(sensor_cadence_t * p, uint8_t * p_buffer)
{
    memcpy(p_buffer, &p->property_id, sizeof(uint16_t));
    uint16_t offset = sizeof(uint16_t);
    p_buffer[offset] = p->fast_period_exponent & CADENCE_DIVISOR_MASK;
    p_buffer[offset++] |= ((p->trigger_type & CADENCE_TRIGGER_MASK) << CADENCE_TRIGGER_SHIFT);

    /* For delta vectors, the number of bytes used depends upon the trigger type.
     */
    uint16_t delta_vector_bytes = delta_vector_bytes_get(p->property_id, p->trigger_type);

    for (uint16_t i = 0; i < delta_vector_bytes; i++)
    {
        p_buffer[offset++] = p->p_trigger_delta_down[i];
    }

    for (uint16_t i = 0; i < delta_vector_bytes; i++)
    {
        p_buffer[offset++] = p->p_trigger_delta_up[i];
    }

    p_buffer[offset++] = p->min_interval_exponent;

    /* For range vectors, the number of bytes used is the number of bytes allocated.
     */
    for (uint16_t i = 0; i < p->range_value_bytes_allocated; i++)
    {
        p_buffer[offset++] = p->p_fast_cadence_low[i];
    }

    for (uint16_t i = 0; i < p->range_value_bytes_allocated; i++)
    {
        p_buffer[offset++] = p->p_fast_cadence_high[i];
    }

    return p_buffer;
}

static void sensor_last_published_set(sensor_cadence_t * p)
{
    memcpy(p->p_previous_value, p->p_current_value, p->range_value_bytes_allocated);
}

static void cadence_unsolicited_status_send(sensor_cadence_t * p)
{
    app_sensor_server_t * p_server = (app_sensor_server_t *)p->p_server;

    if (p->marshalled_bytes == p->value_marshall(p,
                                                 mp_marshalled_data,
                                                 p_server->state.marshalled_list_bytes))
    {
        uint32_t status  = sensor_server_status_publish(&p_server->server.sensor_srv,
                                                        mp_marshalled_data,
                                                        p->marshalled_bytes,
                                                        SENSOR_OPCODE_STATUS);
        if (NRF_SUCCESS == status)
        {
            sensor_last_published_set(p);
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                  "ERR: publication failed (0x%04x (%d)) = (status)\n",
                  status,
                  status);
        }
    }
}

static void min_interval_limited_status_schedule(sensor_cadence_t * p_cadence)
{
    model_timer_t * p_timer = &p_cadence->min_interval_timer;

    if (model_timer_is_running(p_timer))
    {
        p_cadence->min_interval_publication_pending = true;
    }
    else
    {
        cadence_unsolicited_status_send(p_cadence);

        uint64_t min_us = MS_TO_US(1ull << p_cadence->min_interval_exponent);
        p_timer->timeout_rtc_ticks = (min_us < m_minimum_publish_interval)
                                   ? MODEL_TIMER_TIMEOUT_MIN_TICKS
                                   : MODEL_TIMER_TICKS_GET_US(min_us);
        p_timer->mode = MODEL_TIMER_MODE_SINGLE_SHOT;
        (void) model_timer_schedule(p_timer);
    }
}

static uint16_t marshalled_entry_bytes_get(uint16_t property_id, uint16_t data_bytes)
{
    if ((data_bytes <= MPID_A_MAX_DATA_BYTES) && (property_id <= FORMAT_A_MAX_PROPERTY_ID))
    {
        return SENSOR_MPID_A_BYTES + data_bytes;
    }
    else
    {
        return SENSOR_MPID_B_BYTES + data_bytes;
    }
}

static sensor_cadence_t * cadence_initialize(sensor_cadence_t * p,
                                             uint16_t property_id,
                                             uint16_t range_vector_bytes,
                                             uint16_t delta_vector_bytes,
                                             uint16_t total_bytes)
{
    if (p == NULL)
    {
        return p;
    }

    /* Initialize the instance data.
     */
    memset(p, 0, total_bytes);
    p->property_id = property_id;
    p->range_value_bytes_allocated = range_vector_bytes;
    p->delta_value_bytes_allocated = delta_vector_bytes;
    p->marshalled_bytes  = marshalled_entry_bytes_get(property_id, range_vector_bytes);
    p->value_marshall = ((range_vector_bytes <= MPID_A_MAX_DATA_BYTES) &&
                         (property_id <= FORMAT_A_MAX_PROPERTY_ID))
                              ? mpid_a_value_marshall
                              : mpid_b_value_marshall;

    /* Initialize the pointers.
     * The variable size data area follows the structure.
     */
    uint8_t * p_data = (uint8_t *)&p[1];

    p->p_fast_cadence_low   = &p_data[0];
    p->p_fast_cadence_high  = &p_data[1 * range_vector_bytes];
    p->p_previous_value     = &p_data[2 * range_vector_bytes];
    p->p_current_value      = &p_data[3 * range_vector_bytes];

    p->p_trigger_delta_down = &p_data[4 * range_vector_bytes];
    p->p_trigger_delta_up   = &p->p_trigger_delta_down[1 * delta_vector_bytes];

    return p;
}

static bool cadence_bytes_get(uint16_t property_id,
                              uint16_t * p_range_vector_bytes,
                              uint16_t * p_delta_vector_bytes,
                              uint16_t * p_total_bytes)
{
    *p_range_vector_bytes = range_vector_bytes_get(property_id);
    *p_delta_vector_bytes = delta_vector_bytes_allocated_get(property_id);

    /* There are 6 pointers into the data area, four to buffers of range_vector_bytes, two to buffers
     * of delta_vector_bytes.
     */
    *p_total_bytes = sizeof(sensor_cadence_t)
        + (4 * *p_range_vector_bytes) + (2 * *p_delta_vector_bytes);

    /*  Return success if and only if range_vector_bytes and delta_vector_bytes have non-zero values.
     *  A zero value for either indicates an unsupported property_id.
     */
    return *p_range_vector_bytes && *p_delta_vector_bytes;
}

static sensor_cadence_t * cadence_new(uint16_t property_id)
{
    sensor_cadence_t * p;     /* The cadence instance */
    uint16_t range_vector_bytes;
    uint16_t delta_vector_bytes;
    uint16_t total_bytes;

    if (!cadence_bytes_get(property_id, &range_vector_bytes, &delta_vector_bytes, &total_bytes))
    {
        /* Property ID not supported */
        NRF_MESH_ASSERT(false);
    }

    p = mesh_mem_alloc(total_bytes);

    if (p == NULL)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: could not allocate %d bytes.\n",
              total_bytes);

        NRF_MESH_ASSERT(false);
    }

    return cadence_initialize(p, property_id, range_vector_bytes, delta_vector_bytes, total_bytes);
}

static bool cadence_activate(sensor_cadence_t * p, uint64_t publish_period_us)
{
    uint64_t min_us;
    uint32_t status;

    NRF_MESH_ASSERT(p);

    /* Abort the instance's cadence timer.
     */
    model_timer_abort(&p->timer);

    /* @tagMeshMdlSp Section 4.1.3:
     * For sensor values in the fast region "... messages shall be published every Publish Period
     * (configured for the model) divided by the Fast Cadence Period Divisor state."
     *
     * "A value represented by the Fast Cadence Period Divisor state (see Section 4.1.3.1) is used
     * as a divider for the Publish Period (configured for the model) if ..." the rate of change
     * exceeds a specified value, the conditions determined by the Status Trigger Type (see Section
     * 4.1.3.2), Status Trigger Delta Down (see Section 4.1.3.3), and the Status Trigger Delta Up
     * (see Section 4.1.3.4).
     */

    /* If the value of the general sensor publish period, publish_period_us, is 0, then no general
     * publish period is set. In this case, the value of period_us becomes 0. This inhibits fast
     * cadence.
     */

    /* Initially, cadence period, period_us, is the same as publish_period_us.
     */
    uint64_t period_us = publish_period_us;
    /* Fast cadence selection reduces the period.
     */
    if (p->in_fast_region(p))
    {
        period_us >>= p->fast_period_exponent;
    }

    /* If fast cadence publish period is less than the general sensor publish period, set the timer
       for this sensor to deliver fast cadence publications.
     */
    if (period_us < publish_period_us)
    {
        /* @tagMeshMdlSp Section 4.1.3.5:
         * "The Status Min Interval field is a 1-octet value that shall control the minimum interval
         * between publishing two consecutive Sensor Status messages. The value is represented as 2^n
         * milliseconds."
         */
        if (period_us < (min_us = MS_TO_US(1ull << p->min_interval_exponent)))
        {
            period_us = min_us;
        }

        p->timer.timeout_rtc_ticks = (period_us < m_minimum_publish_interval)
                                   ? MODEL_TIMER_TIMEOUT_MIN_TICKS
                                   : MODEL_TIMER_TICKS_GET_US(period_us);

        p->timer.mode = MODEL_TIMER_MODE_SINGLE_SHOT;

        if (NRF_SUCCESS != (status = model_timer_schedule(&p->timer)))
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                  "ERR: Failed to start cadence timer (%d) = (status).\n",
                  status);
            return false;
        }
    }

    return true;
}

static sensor_cadence_t * cadence_instance_get(app_sensor_server_t * p_server,
                                               uint16_t property_id)
{

    NRF_MESH_ASSERT(p_server);

    LIST_FOREACH(p_node, p_server->state.p_cadence_list)
    {
        sensor_cadence_t * p;     /* The cadence instance */

        p = PARENT_BY_FIELD_GET(sensor_cadence_t, list_node, p_node);;

        if (property_id == p->property_id)
        {
            /* The property ID maps to the cadence instance at p.
             */
            return p;
        }
    }

    /* The property ID does not map to a cadence instance.
     */
    return NULL;
}

static uint8_t * sensor_cadence_serialize(sensor_cadence_t * p, uint16_t * p_buffered_bytes)
{
    NRF_MESH_ASSERT(p);

    *p_buffered_bytes = cadence_serialized_bytes(p);
    return cadence_serialize(p, p->p_server->p_message_buffer);
}

static void sensor_current_value_set(sensor_cadence_t * p)
{
    uint16_t bytes = p->range_value_bytes_allocated;

    ((app_sensor_server_t *)p->p_server)->sensor_get_cb(p->p_server, p->property_id,
                                                        p->p_current_value, &bytes);

    if (bytes != p->range_value_bytes_allocated)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: sensor_get_cb() failed (%d, %d) = (bytes expected, bytes returned).\n",
              p->range_value_bytes_allocated,
              bytes);

        NRF_MESH_ASSERT(false);
    }
}

/* A common callback services all cadence timers. p_context identifies the cadence instance.
 */
static void cadence_timer_cb(void * p_context)
{
    sensor_cadence_t * p;   /* The cadence instance */

    /* Establish context.
     */
    p = (sensor_cadence_t *)p_context;
    app_sensor_server_t * p_server = (app_sensor_server_t *)p->p_server;
    uint64_t publish_period_us = publish_period_get(p_server->server.sensor_srv.model_handle);

    /* Update the sensor's current value.
     */
    sensor_current_value_set(p);

    if (publish_period_us == 0)
    {
        // No publish period set, should not publish cadence timer status
        return;
    }

    /* Attempt to send an unsolicited status message. Then re-set the cadence timer.
     */
    min_interval_limited_status_schedule(p);
    (void) cadence_activate(p, publish_period_us);
}

static void min_interval_timer_cb(void * p_context)
{
    sensor_cadence_t * p_cadence = (sensor_cadence_t *)p_context;
    model_timer_t * p_timer = &(p_cadence->min_interval_timer);

    if (p_cadence->min_interval_publication_pending)
    {
        p_cadence->min_interval_publication_pending = false;
        cadence_unsolicited_status_send(p_cadence);

        uint64_t min_us = MS_TO_US(1ull << p_cadence->min_interval_exponent);
        p_timer->timeout_rtc_ticks = (min_us < m_minimum_publish_interval)
                                   ? MODEL_TIMER_TIMEOUT_MIN_TICKS
                                   : MODEL_TIMER_TICKS_GET_US(min_us);

        p_timer->mode = MODEL_TIMER_MODE_SINGLE_SHOT;
        (void) model_timer_schedule(p_timer);
    }
    else
    {
        model_timer_abort(p_timer);
    }
}

static void cadence_status_get(sensor_cadence_t * p,
                               sensor_status_msg_pkt_t * p_out,
                               uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_out_bytes && p_out);

    if (p == NULL)
    {
        *p_out_bytes = 0;
        return;
    }

    *p_out_bytes = p->marshalled_bytes;

    sensor_current_value_set(p);
    if (p->marshalled_bytes != p->value_marshall(p, p_out, *p_out_bytes))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "ERR:inconsistent state.\n");
        NRF_MESH_ASSERT(false);
    }
}

/* Entry-point definitions.
 */

uint8_t * sensor_marshalled_entry_parse(uint8_t * p_data_buf, uint8_t * p_format, uint8_t * p_data_length,
                                        uint16_t * p_property_id, uint8_t ** pp_data_value)
{
    NRF_MESH_ASSERT(p_data_buf && p_format && p_data_length && p_property_id && pp_data_value);

    /* figure out which format of marshalled data this is first */
    if ((p_data_buf[0] & FORMAT_MASK) == SENSOR_FORMAT_A_BIT)
    {
        /* It's format a */
        sensor_mpid_a_t * p_mpid = (sensor_mpid_a_t *)p_data_buf;
        /* Length is defined as 0-0xF representing 1-16, therefore + 1 */
        uint8_t length = p_mpid->length + 1;
        *p_property_id = p_mpid->property_id;
        *pp_data_value = &p_data_buf[SENSOR_MPID_A_BYTES];
        *p_data_length = length;
        *p_format = 0;
        return *pp_data_value + length;
    }
    else
    {
        /* It's format b */
        sensor_mpid_b_t * p_mpid = (sensor_mpid_b_t *)p_data_buf;
        /* Length is defined as 0-0x7F representing 1-127, 0x7F = 0 */
        uint8_t length = p_mpid->length;
        length = (length == SENSOR_MPID_B_ZERO_DATA_BYTES) ? 0 : length + 1;
        *p_property_id = p_mpid->property_id;

        *pp_data_value = &p_data_buf[SENSOR_MPID_B_BYTES];
        *p_data_length = length;
        *p_format = 1;
        return *pp_data_value + length;
    }
}

uint8_t sensor_percentage8_create(uint8_t value, uint8_t b_exp)
{
    NRF_MESH_ASSERT((b_exp == 0) || (b_exp == 1));

    value = (value << 1);
    value |= b_exp;
    return value;
}

uint8_t sensor_percentage8_parse(uint8_t value, uint8_t * b_exp)
{
    NRF_MESH_ASSERT(b_exp);
    *b_exp = value & 0x1;
    value = value >> 1;

    return value;
}

sensor_cadence_t * sensor_cadence_create(uint16_t property_id,
                                         uint8_t * p_buffer,
                                         uint16_t * p_bytes)
{
    uint16_t range_vector_bytes;
    uint16_t delta_vector_bytes;

    NRF_MESH_ASSERT(p_buffer && p_bytes);

    uint16_t available_bytes = *p_bytes;

    if (!cadence_bytes_get(property_id, &range_vector_bytes, &delta_vector_bytes, p_bytes)
     || (*p_bytes > available_bytes))
    {
        return NULL;
    }

    return cadence_initialize((sensor_cadence_t *)p_buffer,
                              property_id,
                              range_vector_bytes,
                              delta_vector_bytes,
                              *p_bytes);
}

sensor_cadence_t * sensor_cadence_to_buffer_deserialize(uint8_t * p_in_buffer,
                                                        uint16_t in_bytes,
                                                        uint8_t * p_out_buffer,
                                                        uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_in_buffer && p_out_buffer && p_out_bytes);

    if (in_bytes < 2)
    {
        return NULL;
    }

    uint16_t property_id;
    memcpy(&property_id, p_in_buffer, sizeof(uint16_t));
    sensor_cadence_t * p_cadence = sensor_cadence_create(property_id,
                                                         p_out_buffer,
                                                         p_out_bytes);

    uint32_t status = cadence_update(p_cadence, p_in_buffer, in_bytes);

    if (status == NRF_SUCCESS)
    {
        return p_cadence;
    }
    else
    {
        return NULL;
    }
}

void sensor_cadence_to_buffer_serialize(sensor_cadence_t * p, uint8_t * p_buffer, uint16_t * p_bytes)
{
    NRF_MESH_ASSERT(p && p_buffer && p_bytes);

    uint16_t serialized_bytes = cadence_serialized_bytes(p);

    *p_bytes = (serialized_bytes <= *p_bytes) ? serialized_bytes : 0;
    if (*p_bytes)
    {
        (void) cadence_serialize(p, p_buffer);
    }
}

bool sensor_cadence_set(app_sensor_server_t * p_server, const sensor_cadence_set_msg_pkt_t * p_in,
                        uint16_t in_bytes, sensor_cadence_status_msg_pkt_t * p_out,
                        uint16_t * p_out_bytes)
{
    sensor_cadence_t * p;     /* The cadence instance */

    NRF_MESH_ASSERT(p_server && p_in && p_out && p_out_bytes);

    /* Cadence activation depends upon the server's publish period.
     */
    uint64_t publish_period_us = publish_period_get(p_server->server.sensor_srv.model_handle);

    p = cadence_instance_get(p_server, p_in->property_id);
    if (!(p
          && cadence_update(p, (uint8_t *) p_in, in_bytes) == NRF_SUCCESS
          && cadence_activate(p, publish_period_us)))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: could not activate cadence (0x%04x) = (property_id)\n",
              p_in->property_id);

        /* Any failure should inhibit publication. Thus the return of false on this path. If there is
         * no cadence instance associated with p_in->property_id, a response with the property id is
         * required. In this case, p will be NULL here. If the update failed, the operation should
         * elicit no response as this means that p_in presented a malformed cadence specification.
         * Thus the assignment of 0 to *p_out_bytes where p is not NULL.
         */
        *p_out_bytes = p ? 0 : sizeof(uint16_t);
        memcpy(p_out, (void *)&(p_in->property_id), sizeof(uint16_t));
        return false;
    }

    // data is serialized into sensor buffer via p.
    (void)sensor_cadence_serialize(p, p_out_bytes);

    return true;
}

void sensor_cadence_get(app_sensor_server_t * p_server, uint16_t property_id,
                        sensor_cadence_status_msg_pkt_t * p_out, uint16_t * p_out_bytes)
{
    sensor_cadence_t * p;     /* The cadence instance */

    NRF_MESH_ASSERT(p_server && p_out && p_out_bytes);

    *p_out_bytes = 0;
    p = cadence_instance_get(p_server, property_id);

    if (p == NULL)
    {
        *p_out_bytes = sizeof(uint16_t);
        memcpy(p_out, &property_id, sizeof(uint16_t));
        return;
    }

    // data is stored into server buffer getting access via p.
    (void)sensor_cadence_serialize(p, p_out_bytes);
}


void sensor_initialize(app_sensor_server_t *p_server)
{
    NRF_MESH_ASSERT(p_server);
    m_minimum_publish_interval = MS_TO_US(MODEL_TIMER_PERIOD_MS_GET(MODEL_TIMER_TIMEOUT_MIN_TICKS));

    p_server->state.marshalled_list_bytes = 0;

    /* Create a list of cadence instances, one for each sensor.
     *   Note
     *
     *     the number of sensors is the same as the number of properties identified by the sensor
     *     property array,
     *
     *     the first byte of the sensor property array gives this value,
     *
     *     and the cadence list anchor resides in the server state structure.
     */
    p_server->state.p_cadence_list = NULL;
    for (uint8_t i = 0; i < (uint8_t) p_server->p_sensor_property_array[0]; i++)
    {
        uint16_t property_id;
        sensor_cadence_t * p;     /* The cadence instance */

        switch ((property_id = p_server->p_sensor_property_array[i + 1]))
        {
            case SENSOR_MOTION_SENSED_PROPERTY_ID:
            {
                p = cadence_new(property_id);

                /* Should a .h file specify the initial values? */
                p->fast_period_exponent  = 0;
                p->trigger_type          = 0;
                p->min_interval_exponent = 10;  /*2^10ms = 512ms */

                p->p_server = p_server;
                p->timer.p_timer_id = &p_server->p_cadence_timer_ids[i];
                p->timer.p_context = p;
                p->timer.cb = cadence_timer_cb;
                (void) model_timer_create(&p->timer);

                p->min_interval_timer.p_timer_id = &p_server->p_min_interval_timer_ids[i];
                p->min_interval_timer.p_context = p;
                p->min_interval_timer.cb = min_interval_timer_cb;
                (void) model_timer_create(&p->min_interval_timer);

                p->in_fast_region     = motion_sensor_in_fast_region;
                p->delta_trigger_fast = motion_sensor_delta_trigger_fast;

                p_server->state.marshalled_list_bytes += p->marshalled_bytes;

                list_add(&p_server->state.p_cadence_list, &p->list_node);
                break;
            }

            default :
            {
                __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                      "ERR: property id 0x%04x (%d) not supported \n",
                      property_id,
                      property_id);
            }
        }
    }

    mp_marshalled_data = mesh_mem_alloc(p_server->state.marshalled_list_bytes);
}

void sensor_activate(app_sensor_server_t * p_server,
                     uint16_t property_id,
                     sensor_status_msg_pkt_t * p_out,
                     uint16_t * p_out_bytes)
{
    /* Update the sensor value, then set the cadence instance's timer.
     */
    NRF_MESH_ASSERT(p_server && p_out && p_out_bytes);

    sensor_cadence_t * p_cadence = cadence_instance_get(p_server, property_id);
    cadence_status_get(p_cadence, p_out, p_out_bytes);

    if (p_cadence != NULL)
    {
        uint64_t period = publish_period_get(p_server->server.sensor_srv.model_handle);
        (void) cadence_activate(p_cadence, period);
    }
}

app_sensor_server_t * sensor_list_activate(app_sensor_server_t * p_server,
                                           sensor_status_msg_pkt_t * p_out,
                                           uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_server && p_out && p_out_bytes);

    uint64_t publish_period_us = publish_period_get(p_server->server.sensor_srv.model_handle);
    *p_out_bytes = 0;

    if (!p_server)
    {
        return NULL;
    }

    uint16_t i = 0;
    uint16_t bytes_remaining = p_server->state.marshalled_list_bytes;
    LIST_FOREACH(p_node, p_server->state.p_cadence_list)
    {
        sensor_cadence_t * p;     /* The cadence instance */

        p = PARENT_BY_FIELD_GET(sensor_cadence_t, list_node, p_node);

        sensor_current_value_set(p);
        if (p->marshalled_bytes != p->value_marshall(p, &mp_marshalled_data[i], bytes_remaining))
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "ERR: inconsistent state.\n");
            NRF_MESH_ASSERT(false);
        }

        (void) cadence_activate(p, publish_period_us);
        i += p->marshalled_bytes;
        bytes_remaining -= p->marshalled_bytes;
    }

    *p_out_bytes = p_server->state.marshalled_list_bytes - bytes_remaining;
    memcpy(p_out, mp_marshalled_data, *p_out_bytes);

    return p_server;
}

uint32_t sensor_cadence_publication_abort(app_sensor_server_t * p_server)
{
    NRF_MESH_ASSERT(p_server);

    uint32_t maximum_minimum_publish_interval_ms = 0;

    LIST_FOREACH(p_node, p_server->state.p_cadence_list)
    {
        sensor_cadence_t * p;     /* The cadence instance */

        uint32_t minimum_publish_interval_ms;

        p = PARENT_BY_FIELD_GET(sensor_cadence_t, list_node, p_node);

        minimum_publish_interval_ms = 1ul << p->min_interval_exponent;

        /* Abort the instance's timer.
         */
        model_timer_abort(&p->timer);

        if (minimum_publish_interval_ms > maximum_minimum_publish_interval_ms)
        {
            maximum_minimum_publish_interval_ms = minimum_publish_interval_ms;
        }
    }

    return maximum_minimum_publish_interval_ms;
}

uint32_t sensor_status_publish(app_sensor_server_t * p_server, uint16_t property_id)
{
    sensor_status_msg_pkt_t * p_out = (sensor_status_msg_pkt_t *)p_server->p_message_buffer;
    uint16_t bytes;
    sensor_cadence_t * p = cadence_instance_get(p_server, property_id);

    if (p == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    cadence_status_get(p, p_out, &bytes);
    if (!p->delta_trigger_fast(p))
    {
        return NRF_SUCCESS;
    }

    min_interval_limited_status_schedule(p);

    return NRF_SUCCESS;
}
