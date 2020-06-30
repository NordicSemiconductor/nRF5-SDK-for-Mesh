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
#include "sensor_setup_server.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>

/* Logging and RTT */
#include "log.h"

#include "sensor_common.h"
#include "sensor_utils.h"
#include "sensor_messages.h"

#include "mesh_mem.h"

#include "utils.h"
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"


static uint32_t server_publish(const sensor_server_t * p_server,
                               const sensor_status_msg_pkt_t * p_status_message,
                               uint16_t bytes,
                               sensor_opcode_t status_opcode)
{
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(status_opcode),
        .p_buffer = (const uint8_t *)p_status_message,
        .length = bytes,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    return access_model_publish(p_server->model_handle, &reply);
}

static uint32_t server_respond(sensor_server_t * p_server,
                               const access_message_rx_t * p_access_message,
                               const uint8_t * p_message,
                               uint16_t bytes,
                               sensor_opcode_t status_opcode)
{
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(status_opcode),
        .p_buffer = p_message,
        .length = bytes,
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    return access_model_reply(p_server->model_handle, p_access_message, &reply);
}


static uint32_t setup_server_publish(const sensor_setup_server_t * p_s_server,
                                     const uint8_t * p_message,
                                     uint16_t bytes,
                                     sensor_opcode_t status_opcode)
{
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(status_opcode),
        .p_buffer = p_message,
        .length = bytes,
        .force_segmented = p_s_server->settings.force_segmented,
        .transmic_size = p_s_server->settings.transmic_size
    };

    return access_model_publish(p_s_server->model_handle, &reply);
}

static uint32_t setup_server_respond(sensor_setup_server_t * p_s_server,
                                     const access_message_rx_t * p_access_message,
                                     const uint8_t * p_message,
                                     uint16_t bytes,
                                     sensor_opcode_t status_opcode)
{
    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(status_opcode),
        .p_buffer = p_message,
        .length = bytes,
        .force_segmented = p_s_server->settings.force_segmented,
        .transmic_size = p_s_server->settings.transmic_size
    };

    return access_model_reply(p_s_server->model_handle, p_access_message, &reply);
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    const sensor_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(sensor_setup_server_t,
                                                                   sensor_srv,
                                                                   p_args);

    /* Publish if the server's publish period is set.
     */
    if (publish_period_get(handle))
    {
        /* Resynchronize sensor publications to the periodic publication period. Publication
         * will occur for each sensor after its specified minimum publication interval expires.
         */
        p_s_server->settings.p_callbacks->sensor_cbs.publication_schedule_cb(p_s_server);
    }
}


/**************************************************************************************************/
/**** Sensor Server ****/
/** Opcode Handlers */

/* Zero is a prohibited property id value for messages to the opcode handlers. However, the callbacks
 * used by handle_get() and handle_descriptor_get() use this value as a wildcard meaning "for all
 * supported property id values". For these handlers, if the meassage includes a property id, its
 * value cannot be prohibited; if the message does not include a property id, these handlers will pass
 * the wildcard property id value to their callbacks.
 *
 * The signature of all callbacks includes **buffer and *bytes. If any callback returns, the value at
 * *buffer will provide the base of a message buffer, and bytes will give the number of bytes of
 * response data stored in this buffer. The data at buffer will be an appropriate server response.
 * The buffer is not dynacically allocated. Do not attempt to free it.
 */

static void handle_get(access_model_handle_t model_handle,
                       const access_message_rx_t * p_rx_msg,
                       void * p_args)
{
    sensor_server_t * p_server = (sensor_server_t *)p_args;
    sensor_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(sensor_setup_server_t,
                                                             sensor_srv,
                                                             p_args);
    sensor_get_msg_pkt_t * p_in = (sensor_get_msg_pkt_t *) p_rx_msg->p_data;
    sensor_status_msg_pkt_t * p_out = NULL;
    uint16_t property_id = 0;
    uint16_t bytes = 0;

    /* If the message does not include a property id, use the wildcard property id value on the
     * get_cb() call. If the message includes a property id and its value is prohibited, take no
     * further action. Otherwise, use the provided property id value.
     */
    if (p_rx_msg->length)
    {
        if (!p_in->property_id)
        {
            /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
             * Property ID.
             */
            return;
        }

        property_id = p_in->property_id;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.get_cb(p_s_server,
                                                        NULL,
                                                        property_id,
                                                        &p_out,
                                                        &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (bytes != 0)
    {
        (void) server_respond(p_server, p_rx_msg, (const uint8_t *)p_out, bytes, SENSOR_OPCODE_STATUS);
    }
}

static void handle_descriptor_get(access_model_handle_t model_handle,
                                  const access_message_rx_t * p_rx_msg,
                                  void * p_args)
{
    sensor_server_t * p_server = (sensor_server_t *)p_args;
    sensor_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(sensor_setup_server_t,
                                         sensor_srv,
                                         p_args);
    sensor_descriptor_get_msg_pkt_t * p_in = (sensor_descriptor_get_msg_pkt_t *) p_rx_msg->p_data;
    sensor_descriptor_status_msg_pkt_t * p_out = NULL;
    uint16_t bytes = 0;
    uint16_t property_id = 0;

    /* If the message does not include a property id, use the wildcard property id value on the
     * descriptor_get_cb() call. If the message includes a property id and its value is
     * prohibited, take no further action. Otherwise, use the provided property id value.
     */
    if (p_rx_msg->length)
    {
        if (!p_in->property_id)
        {
            /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
             * Property ID.
             */
            return;
        }

        property_id = p_in->property_id;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.descriptor_get_cb(p_s_server,
                                                                   &p_rx_msg->meta_data,
                                                                   property_id,
                                                                   &p_out,
                                                                   &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (bytes != 0)
    {
        (void) server_respond(p_server, p_rx_msg, (const uint8_t *)p_out, bytes, SENSOR_OPCODE_DESCRIPTOR_STATUS);
    }
}

static void handle_column_get(access_model_handle_t model_handle,
                              const access_message_rx_t * p_rx_msg,
                              void * p_args)
{
    sensor_server_t * p_server = (sensor_server_t *)p_args;
    sensor_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(sensor_setup_server_t,
                                                             sensor_srv,
                                                             p_args);
    sensor_column_get_msg_pkt_t * p_in = (sensor_column_get_msg_pkt_t *) p_rx_msg->p_data;
    uint16_t bytes = 0;
    sensor_column_status_msg_pkt_t * p_out = NULL;

    /* If the message does not include a property_id value, or includes a property id value that
     * is prohibited, take no further action.
     */
    if (!(p_rx_msg->length && p_in->property_id))
    {
        /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
         * Property ID.
         */
        return;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.column_get_cb(p_s_server,
                                                               &p_rx_msg->meta_data,
                                                               p_in,
                                                               p_rx_msg->length,
                                                               &p_out,
                                                               &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (bytes != 0)
    {
        (void) server_respond(p_server, p_rx_msg, (const uint8_t *)p_out, bytes, SENSOR_OPCODE_COLUMN_STATUS);
    }
}

static void handle_series_get(access_model_handle_t model_handle,
                              const access_message_rx_t * p_rx_msg,
                              void * p_args)
{
    sensor_server_t * p_server = (sensor_server_t *)p_args;
    sensor_setup_server_t * p_s_server = PARENT_BY_FIELD_GET(sensor_setup_server_t,
                                                             sensor_srv,
                                                             p_args);
    sensor_series_get_msg_pkt_t * p_in = (sensor_series_get_msg_pkt_t *) p_rx_msg->p_data;;
    sensor_series_status_msg_pkt_t * p_out = NULL;
    uint16_t bytes = 0;

    /* If the message does not include a property_id value, or includes a property id value that
     * is prohibited, take no further action.
     */
    if (!(p_rx_msg->length && p_in->property_id))
    {
        /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
         * Property ID.
         */
        return;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.series_get_cb(p_s_server,
                                                               &p_rx_msg->meta_data,
                                                               p_in,
                                                               p_rx_msg->length,
                                                               &p_out,
                                                               &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (bytes != 0)
    {
        (void) server_respond(p_server, p_rx_msg, (const uint8_t *)p_out, bytes, SENSOR_OPCODE_SERIES_STATUS);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_DESCRIPTOR_GET), handle_descriptor_get},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_GET), handle_get},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_COLUMN_GET), handle_column_get},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SERIES_GET), handle_series_get},
};


/**************************************************************************************************/
/**** Setup Server */
/** Opcode Handlers */

static void handle_cadence_get(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    sensor_setup_server_t * p_s_server = (sensor_setup_server_t *)p_args;
    sensor_cadence_get_msg_pkt_t * p_in =(sensor_cadence_get_msg_pkt_t *)p_rx_msg->p_data;
    uint16_t bytes = 0;
    sensor_cadence_status_msg_pkt_t * p_out = NULL;

    /* If the message does not include a property_id value, or includes a property id value that
     * is prohibited, take no further action.
     */
    if (!(p_rx_msg->length && p_in->property_id))
    {
        /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
         * Property ID.
         */
        return;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.cadence_get_cb(p_s_server,
                                                                NULL,
                                                                p_in->property_id,
                                                                &p_out,
                                                                &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (bytes != 0)
    {
        (void) setup_server_respond(p_s_server, p_rx_msg, (const uint8_t *)p_out, bytes,
                                    SENSOR_OPCODE_CADENCE_STATUS);
    }
}

static void handle_cadence_set(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    sensor_setup_server_t * p_s_server = (sensor_setup_server_t *) p_args;
    sensor_cadence_set_msg_pkt_t * p_in = (sensor_cadence_set_msg_pkt_t *) p_rx_msg->p_data;
    uint16_t bytes = 0;
    sensor_cadence_status_msg_pkt_t * p_out = NULL;

    /* If the message length is not sufficient, or the message includes a property id value that
     * is prohibited, take no further action.
     */
    if (!(p_rx_msg->length >= SENSOR_CADENCE_SET_MINLEN && p_in->property_id))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: prohibited (%d, 0x%04x (%d)) = (p_rx_msg->length, p_in->property_id)\n",
              p_rx_msg->length,
              p_in->property_id,
              p_in->property_id);

        /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
         * Property ID.
         */
        return;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.cadence_set_cb(p_s_server,
                                                                NULL,
                                                                p_in->property_id,
                                                                p_in,
                                                                p_rx_msg->length,
                                                                &p_out,
                                                                &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (SENSOR_OPCODE_CADENCE_SET == p_rx_msg->opcode.opcode && bytes != 0)
    {
        (void) setup_server_respond(p_s_server, p_rx_msg, (const uint8_t *)p_out, bytes,
                                    SENSOR_OPCODE_CADENCE_STATUS);
    }
}

static void handle_settings_get(access_model_handle_t model_handle,
                                const access_message_rx_t * p_rx_msg,
                                void * p_args)
{
    sensor_setup_server_t * p_s_server = (sensor_setup_server_t *)p_args;
    sensor_settings_get_msg_pkt_t * p_in = (sensor_settings_get_msg_pkt_t *) p_rx_msg->p_data;
    uint16_t bytes = 0;
    sensor_settings_status_msg_pkt_t * p_out = NULL;

    /* If the message does not include a property_id value, or includes a property id value that
     * is prohibited, take no further action.
     */
    if (!(p_rx_msg->length && p_in->property_id))
    {
        /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
         * Property ID.
         */
        return;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.settings_get_cb(p_s_server,
                                                                 NULL,
                                                                 p_in->property_id,
                                                                 &p_out,
                                                                 &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (bytes != 0)
    {
        (void) setup_server_respond(p_s_server, p_rx_msg, (const uint8_t *)p_out, bytes,
                                    SENSOR_OPCODE_SETTINGS_STATUS);
    }
}

/* NOTE that this is *setting* not *settings*.
 */
static void handle_setting_get(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    sensor_setup_server_t * p_s_server = (sensor_setup_server_t *)p_args;
    sensor_setting_get_msg_pkt_t * p_in = (sensor_setting_get_msg_pkt_t *) p_rx_msg->p_data;
    uint16_t bytes = 0;
    sensor_setting_status_msg_pkt_t * p_out = NULL;

    /* If the message does not include a property_id value, or includes a property id value that
     * is prohibited, take no further action.
     */
    if (!(p_rx_msg->length && p_in->property_id))
    {
        /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
         * Property ID.
         */
        return;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.setting_get_cb(p_s_server,
                                                                NULL,
                                                                p_in->property_id,
                                                                p_in->setting_property_id,
                                                                &p_out,
                                                                &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (bytes != 0)
    {
        (void) setup_server_respond(p_s_server, p_rx_msg, (const uint8_t *)p_out, bytes,
                                    SENSOR_OPCODE_SETTING_STATUS);
    }
}

static void handle_setting_set(access_model_handle_t model_handle,
                               const access_message_rx_t * p_rx_msg,
                               void * p_args)
{
    sensor_setup_server_t * p_s_server = (sensor_setup_server_t *) p_args;
    sensor_setting_set_msg_pkt_t * p_in = (sensor_setting_set_msg_pkt_t *) p_rx_msg->p_data;
    uint16_t bytes = 0;
    sensor_setting_status_msg_pkt_t * p_out = NULL;

    /* If the message length is not sufficient, or the message includes a property id value that
     * is prohibited, take no further action.
     */
    if (!(p_rx_msg->length >= SENSOR_SETTING_SET_MINLEN && p_in->property_id))
    {
        /* 0 is a prohibited sensor property id value, see @tagMeshMdlSp 4.1.1.1 Sensor
         * Property ID.
         */
        return;
    }

    p_s_server->settings.p_callbacks->sensor_cbs.setting_set_cb(p_s_server,
                                                                NULL,
                                                                p_in->property_id,
                                                                p_in->setting_property_id,
                                                                p_in,
                                                                p_rx_msg->length,
                                                                &p_out,
                                                                &bytes);
    NRF_MESH_ASSERT(p_out != NULL);
    /* The call-back should provide a response, but don't depend upon it.
     */
    if (p_rx_msg->opcode.opcode == SENSOR_OPCODE_SETTING_SET && bytes != 0)
    {
        (void) setup_server_respond(p_s_server, p_rx_msg, (const uint8_t *)p_out, bytes,
                                    SENSOR_OPCODE_SETTING_STATUS);
    }
}

static const access_opcode_handler_t m_opcode_handlers_setup[] =
{
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_CADENCE_GET), handle_cadence_get},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_CADENCE_SET), handle_cadence_set},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_CADENCE_SET_UNACKNOWLEDGED), handle_cadence_set},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SETTINGS_GET), handle_settings_get},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SETTING_GET), handle_setting_get},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SETTING_SET), handle_setting_set},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SETTING_SET_UNACKNOWLEDGED), handle_setting_set},
};

/** Interface functions */

static uint32_t sensor_server_init(sensor_server_t * p_server, uint16_t element_index)
{
    uint32_t status = NRF_SUCCESS;

    if (NULL == p_server)
    {
        return NRF_ERROR_NULL;
    }

    if (NRF_SUCCESS == status)
    {

        access_model_add_params_t init_params =
        {
            .model_id = ACCESS_MODEL_SIG(SENSOR_SERVER_MODEL_ID),
            .element_index =  element_index,
            .p_opcode_handlers = &m_opcode_handlers[0],
            .opcode_count = ARRAY_SIZE(m_opcode_handlers),
            .p_args = p_server,
            .publish_timeout_cb = periodic_publish_cb
        };

        status = access_model_add(&init_params, &p_server->model_handle);
    }

    return status;
}

uint32_t sensor_server_status_publish(const sensor_server_t * p_server,
                                      const sensor_status_msg_pkt_t * p_data,
                                      uint16_t data_length,
                                      sensor_opcode_t status_opcode)
{
    if (!(p_server && p_data))
    {
        return NRF_ERROR_NULL;
    }

    return server_publish(p_server, p_data, data_length, status_opcode);
}

uint32_t sensor_server_setup_status_publish(const sensor_setup_server_t * p_s_server,
                                            const uint8_t * p_message,
                                            uint16_t data_length,
                                            sensor_opcode_t status_opcode)
{
    if (!(p_s_server && p_message))
    {
        return NRF_ERROR_NULL;
    }

    return setup_server_publish(p_s_server, p_message, data_length, status_opcode);
}

uint32_t sensor_setup_server_init(sensor_setup_server_t * p_server, uint16_t element_index)
{
    uint32_t status = NRF_SUCCESS;

    if (!(p_server
       && p_server->settings.p_callbacks
       && p_server->settings.p_callbacks->sensor_cbs.descriptor_get_cb
       && p_server->settings.p_callbacks->sensor_cbs.get_cb
       && p_server->settings.p_callbacks->sensor_cbs.column_get_cb
       && p_server->settings.p_callbacks->sensor_cbs.series_get_cb
       && p_server->settings.p_callbacks->sensor_cbs.cadence_get_cb
       && p_server->settings.p_callbacks->sensor_cbs.cadence_set_cb
       && p_server->settings.p_callbacks->sensor_cbs.settings_get_cb
       && p_server->settings.p_callbacks->sensor_cbs.setting_get_cb
       && p_server->settings.p_callbacks->sensor_cbs.setting_set_cb
       && p_server->settings.p_callbacks->sensor_cbs.publication_schedule_cb))
    {
        return NRF_ERROR_NULL;
    }

    /* Initialize parent model instances - Sensor */
    p_server->sensor_srv.settings.force_segmented = p_server->settings.force_segmented;
    p_server->sensor_srv.settings.transmic_size = p_server->settings.transmic_size;
    status = sensor_server_init(&p_server->sensor_srv, element_index);


    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(SENSOR_SETUP_SERVER_MODEL_ID),
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers_setup[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers_setup),
        .p_args = p_server
    };

    status = access_model_add(&init_params, &p_server->model_handle);

    if (NRF_SUCCESS == status)
    {
        status = access_model_subscription_list_alloc(p_server->model_handle);
    }

    if (NRF_SUCCESS == status)
    {
        status = access_model_subscription_lists_share(p_server->model_handle,
                                                       p_server->sensor_srv.model_handle);
    }

    return status;
}
