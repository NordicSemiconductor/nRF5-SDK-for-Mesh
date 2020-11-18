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

#ifndef APP_SENSOR_UTILS_H__
#define APP_SENSOR_UTILS_H__

#include "list.h"
#include "model_common.h"

#include "app_sensor.h"
#include "sensor_utils.h"

typedef struct __sensor_cadence_t sensor_cadence_t;

typedef bool (*sensor_in_fast_region_t)(sensor_cadence_t *);
typedef bool (*sensor_delta_trigger_fast_t)(sensor_cadence_t *);
typedef uint16_t (*cadence_value_marshall_t)(sensor_cadence_t *, uint8_t *, uint16_t);

struct __sensor_cadence_t
{
    list_node_t list_node;
    model_timer_t timer;                             /**< For providing cadence */
    model_timer_t min_interval_timer;                /**< For enforcing status min interval */
    bool min_interval_publication_pending;           /**< Has a publication been scheduled waiting for min interval to expire */
    app_sensor_server_t * p_server;                  /**< Identifies the owning server */
    uint8_t * p_trigger_delta_down;                  /**< Value triggers fast cadence */
    uint8_t * p_trigger_delta_up;                    /**< Value triggers fast cadence */
    uint8_t * p_fast_cadence_low;                    /**< Value for fast cadence range */
    uint8_t * p_fast_cadence_high;                   /**< Value for fast cadence range */
    uint8_t * p_previous_value;                      /**< The previous sensor value */
    uint8_t * p_current_value;                       /**< The current sensor value */
    sensor_in_fast_region_t in_fast_region;          /**< Function returning whether the value is in the fast cadence region */
    sensor_delta_trigger_fast_t delta_trigger_fast;  /**< Function returning whether the delta should trigger fast cadence */
    cadence_value_marshall_t value_marshall;         /**< Function to marshall the data in the supplied buffer into correct format */
    uint16_t property_id;                            /**< Sensor property ID */
    uint8_t fast_period_exponent;                    /**< Fast cadence exponent */
    uint8_t trigger_type;                            /**< Fast cadence trigger type */
    uint8_t min_interval_exponent;                   /**< Minimum cadence interval */
    uint16_t range_value_bytes_allocated;            /**< buffer size for value arrays */
    uint16_t delta_value_bytes_allocated;            /**< buffer size for delta arrays */
    uint16_t marshalled_bytes;                       /**< The marshalled value size  */
};

/**
 * Read through marshalled entry, returning parameters
 *
 * @note - if the Sensor Status contains multiple entries, call this function with data_buf pointer
 * at the next MPID location
 *
 * @param[in]  p_data_buf     buffer containing the marshalled entry
 * @param[out] p_format       0 = format A, 1 = format B
 * @param[out] p_data_length  length of the raw value (parsed from entry)
 * @param[out] p_property_id  property id for this entry
 * @param[out] pp_data_value  pointer to the data_buf starting at the raw value
 *
 * @returns  Address of next MPID (if there is one - caller needs to compare to buf length)
 */
uint8_t * sensor_marshalled_entry_parse(uint8_t * p_data_buf,
                                        uint8_t * p_format,
                                        uint8_t * p_data_length,
                                        uint16_t * p_property_id,
                                        uint8_t ** pp_data_value);

/**
 * Convert a 7 bit value + binary exponent to a percentage 8 value.
 *
 * Percentage 8 has a binary exponent of -1
 *
 * @param[in]     value    7 bit value to be converted
 * @param[in]     b_exp    binary exponent (0 or 1) - 1 bit value
 *
 * @returns  the percentage 8 value
 */
uint8_t sensor_percentage8_create(uint8_t value, uint8_t b_exp);

/**
 * Convert a percentage 8 value to a 7 bit value + binary exponent
 *
 * @param[in]      value      percentage 8 value to convert
 * @param[out]     p_b_exp    binary exponent (0 or 1) - 1 bit value
 *
 * @returns  the 7 bit value (caller needs to combine with the b_exp)
 */
uint8_t sensor_percentage8_parse(uint8_t value, uint8_t * p_b_exp);

/**
 * Creates a cadence instance in the provided buffer.
 *
 * @param[in]       property_id  The property id
 * @param[in]       p_buffer     The base of the destination buffer.
 * @param[in, out]  p_bytes      The number of bytes at p_buffer on input; the number of bytes
 *                               required on output.
 *
 * @returns a pointer to the sensor cadence instance on success; NULL otherwise
 *
 * @note *p_bytes gives the number of bytes at p_buffer consumed on success; the number of bytes that
 * are needed for success otherwise.
 */
sensor_cadence_t * sensor_cadence_create(uint16_t property_id,
                                         uint8_t * p_buffer,
                                         uint16_t * p_bytes);

/**
 * Applies a sensor cadence set message.
 *
 * @param[in]   p_server    The server context.
 * @param[in]   p_in        The message to apply.
 * @param[in]   in_bytes    The number of bytes at p_in.
 * @param[out]  p_out       The serialized cadence instance.
 * @param[out]  p_out_bytes The number of bytes at p_out; 0 means no response is in order.
 *
 * @note Note that this entry-point supports the cadence set operation. Where p_in->property_id is
 * not supported the cadence set operation requires a reponse. In this case, sensor_cadence_set()
 * will return false with *p_out_bytes non-zero. Where the property id is supported, but p_in does
 * not specify a valid cadence setting--for example, due to a prohibited value--no response is
 * valid. In this case, sensor_cadence_set() will return false with *p_out_bytes zero.
 *
 * @returns  true if a publication is in order; false otherwise.
 */
bool sensor_cadence_set(app_sensor_server_t * p_server,
                        const sensor_cadence_set_msg_pkt_t * p_in,
                        uint16_t in_bytes,
                        sensor_cadence_status_msg_pkt_t * p_out,
                        uint16_t * p_out_bytes);


/**
 * Provides a serialized sensor cadence instance.
 *
 * @param[in]   p_server    The server context.
 * @param[in]   property_id Identifies the cadence instance of interest.
 * @param[out]  p_out       The serialized sensor cadence instance.
 * @param[out]  p_out_bytes The number of bytes at p_out.
 *
 */
void sensor_cadence_get(app_sensor_server_t * p_server,
                        uint16_t property_id,
                        sensor_cadence_status_msg_pkt_t * p_out,
                        uint16_t * p_out_bytes);

/**
 * Serializes the given cadence instance into the given buffer.
 *
 * @param[in]   p_cadence The cadence instance.
 * @param[in]   p_buffer  The address of the buffer to receive the serialized cadence instance.
 * @param[out]  p_bytes   The size in bytes of the buffer at p_buffer on input;
 *              the number of bytes of serialized data in p_buffer on output.
 */
void sensor_cadence_to_buffer_serialize(sensor_cadence_t * p_cadence,
                                        uint8_t * p_buffer,
                                        uint16_t * p_bytes);

/**
 * Deserializes a given serialized cadence into a given buffer.
 *
 * @param[in]  p_in_buffer  The serialized cadence instance to deserialize.
 * @param[in]  in_bytes     The number of bytes of serialized data at p_in_buf.
 * @param[in]  p_out_buffer The base of the destination buffer.
 * @param[in]  p_out_bytes  The number of bytes at p_out_buffer on input; the number of bytes
 *                          required on output.
 *
 * @returns a pointer to the sensor cadence instance on success; NULL otherwise
 *
 * @note On output, *p_out_bytes gives the number of bytes at p_buffer consumed on success; the
 *                   number of bytes that are needed for success otherwise.
 *
 * @returns  the deserialized cadence instance
 */
sensor_cadence_t * sensor_cadence_to_buffer_deserialize(uint8_t * p_in_buffer,
                                                        uint16_t in_bytes,
                                                        uint8_t * p_out_buffer,
                                                        uint16_t * p_out_bytes);

/**
 * Initializes p_server's sensor list.
 *
 * @param[in]  p_server  The server of interest.
 *
 */
void sensor_initialize(app_sensor_server_t *p_server);


/**
 * Activates p_server's cadence instances.
 * Allocates and returns sensor status in an *p_out_byte buffer at p_out.
 * @param[in]   p_server    The server of interest.
 * @param[out]  p_out       The return status message.
 * @param[out]  p_out_bytes The number of bytes at p_out.
 *
 * @returns  p_server
 */
app_sensor_server_t * sensor_list_activate(app_sensor_server_t * p_server,
                                           sensor_status_msg_pkt_t * p_out,
                                           uint16_t * p_out_bytes);

/**
 * Activates p_server's property_id cadence instance.
 * Allocates and returns sensor status in an *p_out_byte buffer at p_out.
 *
 * @param[in]   p_server     The server of interest.
 * @param[in]   property_id  The sensor property ID of interest.
 * @param[out]  p_out        The return status message.
 * @param[out]  p_out_bytes  The number of bytes at p_out.
 */
void sensor_activate(app_sensor_server_t * p_server,
                     uint16_t property_id,
                     sensor_status_msg_pkt_t * p_out,
                     uint16_t * p_out_bytes);

/**
 * Aborts cadence-based publications for a server's sensors.
 *
 * @param[in]  p_server  The server of interest.
 *
 * @returns The maximum minimum publication interval for the server's sensors.
 */
uint32_t sensor_cadence_publication_abort(app_sensor_server_t * p_server);

/** Publishes a Sensor Status message.
 *
 * This api is called to publish sensor data to the mesh
 *
 * @param[in] p_server        Pointer to @ref __app_sensor_server_t [app_sensor_server_t] context
 * @param[in] property_id     The sensor property id for the sensor of interest.
 * 
 * @retval NRF_SUCCESS              The message is published successfully.
 * @retval NRF_ERROR_NULL           NULL pointer given to function.
 * @retval NRF_ERROR_NO_MEM         Not enough memory available for message.
 * @retval NRF_ERROR_NOT_FOUND      Cadence instance with property_id has not been found, invalid
 *                                  model handle or model not bound to element.
 * @retval NRF_ERROR_INVALID_PARAM  Model not bound to appkey, publish address not set or wrong
 *                                  opcode format.
 * @retval NRF_ERROR_FORBIDDEN      Failed to allocate a sequence number from network.
 * @retval NRF_ERROR_INVALID_STATE  There's already a segmented packet that is being to sent to this
 *                                  destination. Wait for the transmission to finish before sending
 *                                  new segmented packets.
 */
uint32_t sensor_status_publish(app_sensor_server_t * p_server, uint16_t property_id);

#endif
