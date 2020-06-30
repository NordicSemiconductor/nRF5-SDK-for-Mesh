/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are perxomitted provided that the following conditions are met:
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
#include "app_sensor.h"

#include <stdint.h>
#include <stdbool.h>

/* Logging and RTT */
#include "log.h"
#include "rtt_input.h"

#include "nrf_assert.h"
#include "mesh_app_utils.h"

#include "sensor_utils.h"
#include "sensor_common.h"
#include "app_sensor_utils.h"

#include "mesh_mem.h"

#include "sdk_config.h"

#include "app_timer.h"
#include "nrf_mesh_assert.h"

/** This sample implementation shows how the model behavior requirements of the sensor server can be
 *  implemented.
 */

/* Forward declaration */
static void sensor_descriptor_get_cb(const sensor_setup_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     uint16_t property_id,
                                     sensor_descriptor_status_msg_pkt_t ** pp_out,
                                     uint16_t * p_out_bytes);

static void sensor_state_get_cb(const sensor_setup_server_t * p_self,
                                const access_message_rx_meta_t * p_meta,
                                uint16_t property_id,
                                sensor_status_msg_pkt_t ** pp_out,
                                uint16_t * p_out_bytes);

static void sensor_column_get_cb(const sensor_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const sensor_column_get_msg_pkt_t * p_in,
                                 uint16_t in_len,
                                 sensor_column_status_msg_pkt_t ** p_out,
                                 uint16_t * p_out_bytes);

static void sensor_series_get_cb(const sensor_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const sensor_series_get_msg_pkt_t * p_in,
                                 uint16_t in_len,
                                 sensor_series_status_msg_pkt_t ** p_out,
                                 uint16_t * p_out_bytes);

static void sensor_cadence_get_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  sensor_cadence_status_msg_pkt_t ** p_out,
                                  uint16_t * p_out_bytes);

static void sensor_cadence_set_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  const sensor_cadence_set_msg_pkt_t * p_in,
                                  uint16_t in_bytes,
                                  sensor_cadence_status_msg_pkt_t ** pp_out,
                                  uint16_t * p_out_bytes);

static void sensor_settings_get_cb(const sensor_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   uint16_t property_id,
                                   sensor_settings_status_msg_pkt_t ** p_out,
                                   uint16_t * p_out_bytes);

static void sensor_setting_get_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  uint16_t setting_property_id,
                                  sensor_setting_status_msg_pkt_t ** p_out,
                                  uint16_t * p_out_bytes);

static void sensor_setting_set_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  uint16_t setting_property_id,
                                  const sensor_setting_set_msg_pkt_t * p_in,
                                  uint16_t in_len,
                                  sensor_setting_status_msg_pkt_t ** p_out,
                                  uint16_t * p_out_bytes);

static void sensor_publication_schedule_cb(const sensor_setup_server_t * p_self);


static const sensor_setup_server_callbacks_t m_sensor_srv_cbs =
{
    .sensor_cbs.descriptor_get_cb = sensor_descriptor_get_cb,
    .sensor_cbs.get_cb = sensor_state_get_cb,
    .sensor_cbs.column_get_cb = sensor_column_get_cb,
    .sensor_cbs.series_get_cb = sensor_series_get_cb,
    .sensor_cbs.cadence_get_cb = sensor_cadence_get_cb,
    .sensor_cbs.cadence_set_cb = sensor_cadence_set_cb,
    .sensor_cbs.settings_get_cb = sensor_settings_get_cb,
    .sensor_cbs.setting_get_cb = sensor_setting_get_cb,
    .sensor_cbs.setting_set_cb = sensor_setting_set_cb,
    .sensor_cbs.publication_schedule_cb = sensor_publication_schedule_cb
};

/* Utils for callbacks */

/* Copy the top-app's descriptor into the msg buffer to be sent back to the mesh when requested.
 */
static void desc_msg_init(app_sensor_server_t * p_server)
{
    const sensor_descriptor_t * p_descriptor = p_server->p_sensor_descriptor;
    sensor_descriptor_status_msg_pkt_t * p_pkt_data = (sensor_descriptor_status_msg_pkt_t *)p_server->p_descriptor_message;
    sensor_descriptor_pkt_t * p_desc_buf = &p_pkt_data->descriptors;

    /* Pack the non-byte aligned descriptor into a buffer for sending
     * across the mesh. (@tagMeshMdlSp - 4.1.1 Sensor Descriptor) */
    for (uint16_t i = 0; i < p_server->sensor_num_desc; i++)
    {
        p_desc_buf[i].sensor_property_id = p_descriptor[i].property_id;
        p_desc_buf[i].sensor_positive_tolerance = p_descriptor[i].positive_tolerance;
        p_desc_buf[i].sensor_negative_tolerance = p_descriptor[i].negative_tolerance;
        p_desc_buf[i].sensor_sampling_function = p_descriptor[i].sampling_function;
        p_desc_buf[i].sensor_measurement_period = p_descriptor[i].measurement_period;
        p_desc_buf[i].sensor_update_interval = p_descriptor[i].update_interval;
    }
}

static descriptor_status_t * supported_descriptor_find(uint16_t property_id,
                                                       app_sensor_server_t * p_server)
{
    descriptor_status_t * p_sensor_descriptor = NULL;

    /* The number of descriptors is sent to us by the top app */
    uint16_t num_descriptors = p_server->sensor_num_desc;

    /* Search the top app-supplied list of descriptors and see if we match one of them.
     */
    for (uint16_t i = 0; i < num_descriptors; i++)
    {
        if (property_id == p_server->p_sensor_descriptor[i].property_id)
        {
            /* We matched the unpacked descriptor, so now index into the packed descriptor array
             *  (message-appropriate) and return that
             */
            p_sensor_descriptor = &p_server->p_descriptor_message[SENSOR_DESCRIPTOR_MSG_SIZE * i];
            break;
        }
    }
    return p_sensor_descriptor;
}

/***** Sensor model interface callbacks *****/

static void sensor_descriptor_get_cb(const sensor_setup_server_t * p_self,
                                     const access_message_rx_meta_t * p_meta,
                                     uint16_t property_id,
                                     sensor_descriptor_status_msg_pkt_t ** pp_out,
                                     uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_descriptor_status_msg_pkt_t * p_out = (sensor_descriptor_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    /*  There is no need to callback to get descriptor data. It is at p_server->p_descriptor_message.
     */
    uint16_t bytes = p_app_server->sensor_num_desc * SENSOR_DESCRIPTOR_MSG_SIZE;

    if (SENSOR_NO_PROPERTY_ID == property_id)
    {
        /* The property id is the wildcard. Return all descriptors */
        *p_out_bytes = bytes;
        memcpy(p_out, p_app_server->p_descriptor_message, *p_out_bytes);
    }
    else
    {
        sensor_descriptor_status_msg_pkt_t * p_local_out =
                (sensor_descriptor_status_msg_pkt_t *)supported_descriptor_find(property_id, p_app_server);

        if (p_local_out == NULL)
        {
            /* This property_id is not supported. Return the property id.
             */
            *p_out_bytes = sizeof(property_id);
            memcpy(p_out, &property_id, sizeof(uint16_t));
        }
        else
        {
            /* This property_id is supported. Return its descriptor data.
             */
            *p_out_bytes = SENSOR_DESCRIPTOR_MSG_SIZE;
            memcpy(p_out, p_local_out, *p_out_bytes);
        }
    }
}

static void sensor_state_get_cb(const sensor_setup_server_t * p_self,
                                const access_message_rx_meta_t * p_meta,
                                uint16_t property_id,
                                sensor_status_msg_pkt_t ** pp_out,
                                uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_status_msg_pkt_t * p_out = (sensor_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    if (SENSOR_NO_PROPERTY_ID == property_id)
    {
        (void) sensor_list_activate(p_app_server, p_out, p_out_bytes);
    }
    else
    {
        sensor_activate(p_app_server, property_id, p_out, p_out_bytes);
    }
    /* On success, sensor_list_activate() and sensor_activate() provide a message suitable for
     * publication. *pp_out gives the message base; *p_out_bytes gives the size of the message in
     * bytes.
     */

    if (*p_out_bytes == 0)
    {
        /* The property id is not supported. Respond with the requested property id in a
         * marshalled packet header. Use a format b MPID to support zero data bytes.
        */
        *p_out_bytes = SENSOR_MPID_B_BYTES;
        sensor_mpid_b_create(property_id, SENSOR_MPID_B_ZERO_DATA_BYTES, p_out);
    }
}

static void sensor_column_get_cb(const sensor_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const sensor_column_get_msg_pkt_t * p_in,
                                 uint16_t in_len,
                                 sensor_column_status_msg_pkt_t ** pp_out,
                                 uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_column_status_msg_pkt_t * p_out = (sensor_column_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;
    /* Callback to fetch requested column data.     */
    p_app_server->sensor_column_get_cb(p_app_server,
                                       p_in,
                                       in_len,
                                       p_out,
                                       p_out_bytes);
}


static void sensor_series_get_cb(const sensor_setup_server_t * p_self,
                                 const access_message_rx_meta_t * p_meta,
                                 const sensor_series_get_msg_pkt_t * p_in,
                                 uint16_t in_len,
                                 sensor_series_status_msg_pkt_t ** pp_out,
                                 uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_series_status_msg_pkt_t * p_out = (sensor_series_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    /* Callback to fetch requested series data. */
    p_app_server->sensor_series_get_cb(p_app_server,
                                       p_in,
                                       in_len,
                                       p_out,
                                       p_out_bytes);
}

static void sensor_cadence_get_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  sensor_cadence_status_msg_pkt_t ** pp_out,
                                  uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_cadence_status_msg_pkt_t * p_out = (sensor_cadence_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    sensor_cadence_get(p_app_server, property_id, p_out, p_out_bytes);
    /* On success, sensor_cadence_get() provides a message suitable for publication. *pp_out gives
     * the message base; *p_out_bytes gives the size of the message in bytes.
     */
}

static void sensor_cadence_set_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  const sensor_cadence_set_msg_pkt_t * p_in,
                                  uint16_t in_bytes,
                                  sensor_cadence_status_msg_pkt_t ** pp_out,
                                  uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_self && p_in && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_cadence_status_msg_pkt_t * p_out = (sensor_cadence_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    /* The property ID must be in the message.
     */
    if (p_in->property_id != property_id)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: property ids don't match %d != %d\n",
              property_id,
              p_in->property_id);

        NRF_MESH_ASSERT(false);
    }

    /* If sensor_cadence_set() returns true, a publication is appropriate.
     */
    if (sensor_cadence_set(p_app_server, p_in, in_bytes, p_out, p_out_bytes))
    {
        (void) sensor_server_setup_status_publish(p_self,
                                                  (const uint8_t *)p_out,
                                                  *p_out_bytes,
                                                  SENSOR_OPCODE_CADENCE_STATUS);
    }

    /* If *p_out_bytes > 0, sensor_cadence_set() provided a message suitable for a response. *pp_out
     * gives the message base; *p_out_bytes gives the size of the message in bytes.
     */
}


static void sensor_settings_get_cb(const sensor_setup_server_t * p_self,
                                   const access_message_rx_meta_t * p_meta,
                                   uint16_t property_id,
                                   sensor_settings_status_msg_pkt_t ** pp_out,
                                   uint16_t * p_out_bytes)

{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_settings_status_msg_pkt_t * p_out = (sensor_settings_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    p_app_server->sensor_settings_get_cb(p_app_server, property_id, p_out, p_out_bytes);
    /* On success, sensor_settings_get_cb() provides a message suitable for publication. *pp_out
     * gives the message base; *p_out_bytes gives the size of the message in bytes.
     */
}

static void sensor_setting_get_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  uint16_t setting_property_id,
                                  sensor_setting_status_msg_pkt_t ** pp_out,
                                  uint16_t * p_out_bytes)

{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_setting_status_msg_pkt_t * p_out = (sensor_setting_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    p_app_server->sensor_setting_get_cb(p_app_server,
                                        property_id,
                                        setting_property_id,
                                        p_out,
                                        p_out_bytes);
}

static void sensor_setting_set_cb(const sensor_setup_server_t * p_self,
                                  const access_message_rx_meta_t * p_meta,
                                  uint16_t property_id,
                                  uint16_t setting_property_id,
                                  const sensor_setting_set_msg_pkt_t * p_in,
                                  uint16_t in_len,
                                  sensor_setting_status_msg_pkt_t ** pp_out,
                                  uint16_t * p_out_bytes)
{
    NRF_MESH_ASSERT(p_self && pp_out && p_out_bytes);

    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_setting_status_msg_pkt_t * p_out = (sensor_setting_status_msg_pkt_t *)p_app_server->p_message_buffer;
    *pp_out = p_out;

    p_app_server->sensor_setting_set_cb(p_app_server,
                                        property_id,
                                        setting_property_id,
                                        p_in,
                                        in_len,
                                        p_out,
                                        p_out_bytes);
    /* On success, sensor_setting_set_cb() provides a message suitable for publication.
     * p_setting_status gives the message base; setting_status_bytes gives the size of the message in
     * bytes.
     */

    (void) sensor_server_setup_status_publish(p_self,
                                              (const uint8_t *)p_out,
                                              *p_out_bytes,
                                              SENSOR_OPCODE_SETTING_STATUS);
}

static void sensor_publication_schedule_cb(const sensor_setup_server_t * p_self)
{
    app_sensor_server_t * p_app_server = PARENT_BY_FIELD_GET(app_sensor_server_t, server, p_self);
    sensor_status_msg_pkt_t * p_out = (sensor_status_msg_pkt_t *)p_app_server->p_message_buffer;
    uint16_t bytes;

    /* A periodic publication request has arrived.
     *
     * Abort cadence-based publications.
     */
    (void) sensor_cadence_publication_abort(p_app_server);

    /* Collect status for all sensors; publish combined status.
     */
    (void) sensor_list_activate(p_app_server, p_out, &bytes);
    (void) sensor_server_status_publish(&p_app_server->server.sensor_srv,
                                        p_out,
                                        bytes,
                                        SENSOR_OPCODE_STATUS);
}


/***** Interface functions *****/
uint32_t app_sensor_init(app_sensor_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL)
    {
        return NRF_ERROR_NULL;
    }

    sensor_initialize(p_server);

    desc_msg_init(p_server);

    p_server->server.settings.p_callbacks = &m_sensor_srv_cbs;
    p_server->server.settings.property_array = p_server->p_sensor_property_array;

    return sensor_setup_server_init(&p_server->server, element_index);
}
