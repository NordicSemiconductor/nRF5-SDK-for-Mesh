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
#include "sensor_client.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"
#include "log.h"
#include "mesh_mem.h"

#include "sensor_common.h"
#include "sensor_utils.h"
#include "sensor_messages.h"
#include "model_common.h"

static sensor_descriptor_t m_descriptor_list[SENSOR_DESCRIPTOR_MAXLEN / SENSOR_DESCRIPTOR_STATE_SIZE
                                              * sizeof(sensor_descriptor_t)];

/* Note: the property that we are using in the server to send data to the sensor client for a PIR
 * sensor.  We have a simple PIR sensor that sets a line if it detects movement, then clears the
 * line when it doesn't sense movement.  The @tagMeshDevPr spec says that the "Motion Sensed"
 * property ranges from 0 to 100% with a property ID of 0x42. It is characteristic "Percentage 8".
 * The BLE sig says that "Percentage 8" is an 8 bit value with binary exponent - "... a percentage
 * with a resolution of 0.5" And a value of 0xFF represents "value is not known". Percentage 8 is
 * assigned the number 0x2B04.
 */

/* The spec says: "The combined size of the Sensor Data state shall not exceed the message payload
 * size." This would be 379 bytes (384 bytes - 4 byte MIC - 1 byte Mesh SIG opcode = 379 bytes).
 */

/** Opcode Handlers */

/* Sensor status */
static void status_handle(access_model_handle_t handle,
                          const access_message_rx_t * p_rx_msg,
                          void * p_args)
{
    sensor_client_t * p_client = (sensor_client_t *) p_args;
    sensor_status_msg_pkt_t * p_msg_data = (sensor_status_msg_pkt_t *) p_rx_msg->p_data;
    uint16_t length = p_rx_msg->length;

    if (length >= SENSOR_STATUS_MINLEN || length <= SENSOR_STATUS_MAXLEN)
    {
        p_client->settings.p_callbacks->sensor_status_cb(p_client, &p_rx_msg->meta_data, p_msg_data,
                                                         length);
    }
}

static void descriptor_status_handle(access_model_handle_t handle,
                                     const access_message_rx_t * p_rx_msg,
                                     void * p_args)
{
    sensor_client_t * p_client = (sensor_client_t *) p_args;

    sensor_descriptor_status_msg_pkt_t * p_msg_data =  (sensor_descriptor_status_msg_pkt_t *) p_rx_msg->p_data;
    uint16_t length = p_rx_msg->length;

    if ((length > SENSOR_DESCRIPTOR_MINLEN) && (length <= SENSOR_DESCRIPTOR_MAXLEN))
    {
        uint16_t num_descriptors = length / SENSOR_DESCRIPTOR_STATE_SIZE;

        if (length % SENSOR_DESCRIPTOR_STATE_SIZE != 0)
        {
            /* section 4.2.2 of @tagMeshMdlSp says message is 8*N */
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "ERR: descriptor message is not 8*N\n");
            return;
        }

        if (num_descriptors > sizeof(m_descriptor_list))
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR, "ERR: inadequate buffer size\n");
            return;
        }

        /* Unpack the message into our descriptor structure */
        sensor_descriptor_pkt_t * p_pkt_desc = &p_msg_data->descriptors;

        for (uint16_t i = 0; i < num_descriptors; i++)
        {
            m_descriptor_list[i].property_id =  p_pkt_desc[i].sensor_property_id;
            m_descriptor_list[i].positive_tolerance = p_pkt_desc[i].sensor_positive_tolerance;
            m_descriptor_list[i].negative_tolerance = p_pkt_desc[i].sensor_negative_tolerance;
            m_descriptor_list[i].sampling_function = p_pkt_desc[i].sensor_sampling_function;
            m_descriptor_list[i].measurement_period = p_pkt_desc[i].sensor_measurement_period;
            m_descriptor_list[i].update_interval = p_pkt_desc[i].sensor_update_interval;
        }
        p_client->settings.p_callbacks->sensor_descriptor_status_cb(p_client, &p_rx_msg->meta_data,
                                                                    m_descriptor_list, num_descriptors);
    }
    else
    {
        if (length == SENSOR_DESCRIPTOR_MINLEN)
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                  "ERR: Server didn't recognize the Property ID 0x%X\n",
                  p_msg_data->property_id);
        }
        else
        {
            __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
                  "ERR: Sensor server: 0x%04x responded with msg length %d\n",
                  p_rx_msg->meta_data.src.value,
                  length);
        }

    }
}

static void cadence_status_handle(access_model_handle_t handle,
                                  const access_message_rx_t * p_rx_msg,
                                  void * p_args)
{
    sensor_cadence_status_msg_pkt_t * p_msg_data =
            (sensor_cadence_status_msg_pkt_t *) p_rx_msg->p_data;
    sensor_client_t * p_client = (sensor_client_t *) p_args;
    uint8_t length = p_rx_msg->length;

    /* this can't be unpacked for the app since it depends on
     * knowledge of the property ids */
    p_client->settings.p_callbacks->sensor_cadence_status_cb(p_client, &p_rx_msg->meta_data,
                                                             p_msg_data, length);
}

static void column_status_handle(access_model_handle_t handle,
                                 const access_message_rx_t * p_rx_msg,
                                 void * p_args)
{
    sensor_column_status_msg_pkt_t * p_msg_data =
            (sensor_column_status_msg_pkt_t *) p_rx_msg->p_data;
    sensor_client_t * p_client = (sensor_client_t *) p_args;

    /* this can't be unpacked for the app since it depends on knowledge of the property ids
     */
    p_client->settings.p_callbacks->sensor_column_status_cb(p_client,
                                                            &p_rx_msg->meta_data,
                                                            p_msg_data,
                                                            p_rx_msg->length - sizeof(sensor_column_status_msg_pkt_t));
}

static void series_status_handle(access_model_handle_t handle,
                                 const access_message_rx_t * p_rx_msg,
                                 void * p_args)
{
    sensor_series_status_msg_pkt_t * p_msg_data =
            (sensor_series_status_msg_pkt_t *) p_rx_msg->p_data;
    sensor_client_t * p_client = (sensor_client_t *) p_args;

    /* this can't be unpacked for the app since it depends on
     * knowledge of the property ids */

    p_client->settings.p_callbacks->sensor_series_status_cb(p_client,
                                                            &p_rx_msg->meta_data,
                                                            p_msg_data,
                                                            p_rx_msg->length - sizeof(sensor_series_status_msg_pkt_t));
}

static void settings_status_handle(access_model_handle_t handle,
                                   const access_message_rx_t * p_rx_msg,
                                   void * p_args)
{
    sensor_settings_status_msg_pkt_t * p_msg_data =
            (sensor_settings_status_msg_pkt_t *) p_rx_msg->p_data;
    sensor_client_t * p_client = (sensor_client_t *) p_args;
    uint8_t length = p_rx_msg->length;

    /* this can't be unpacked for the app since it depends on knowledge of the property ids
     */
    p_client->settings.p_callbacks->sensor_settings_status_cb(p_client, &p_rx_msg->meta_data,
                                                              p_msg_data, length);
}

static void setting_status_handle(access_model_handle_t handle,
                                  const access_message_rx_t * p_rx_msg,
                                  void * p_args)
{
    sensor_setting_status_msg_pkt_t * p_msg_data =
            (sensor_setting_status_msg_pkt_t *) p_rx_msg->p_data;
    sensor_client_t * p_client = (sensor_client_t *) p_args;
    uint8_t length = p_rx_msg->length;

    /* this can't be unpacked for the app since it depends on knowledge of the property ids
     */
    p_client->settings.p_callbacks->sensor_setting_status_cb(p_client, &p_rx_msg->meta_data,
                                                             p_msg_data, length);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_DESCRIPTOR_STATUS), descriptor_status_handle},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_STATUS), status_handle},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_COLUMN_STATUS), column_status_handle},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SERIES_STATUS), series_status_handle},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_CADENCE_STATUS), cadence_status_handle},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SETTINGS_STATUS), settings_status_handle},
    {ACCESS_OPCODE_SIG(SENSOR_OPCODE_SETTING_STATUS), setting_status_handle},
};


static void message_create(sensor_client_t * p_client,
                           uint16_t tx_opcode,
                           const uint8_t * p_buffer,
                           uint16_t length,
                           access_message_tx_t * p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_client->settings.force_segmented;
    p_message->transmic_size = p_client->settings.transmic_size;
    p_message->access_token = nrf_mesh_unique_token_get();
}

static void reliable_context_create(sensor_client_t * p_client,
                                    uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_client->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->timeout = p_client->settings.timeout;
    p_reliable->status_cb = p_client->settings.p_callbacks->ack_transaction_status_cb;
}

static uint32_t message_cache(sensor_client_t * p,
                              uint8_t ** pp_message,
                              uint8_t * p_buffer,
                              uint8_t bytes)
{
    if (bytes > p->message_buffer_bytes)
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "ERR: size exceeds resource, bytes (%d, %d) = (needed, available)\n",
              bytes,
              p->message_buffer_bytes);

        return NRF_ERROR_NULL;
    }

    memcpy(p->p_message_buffer, p_buffer, bytes);
    *pp_message = p->p_message_buffer;

    return NRF_SUCCESS;
}

/** Interface functions */
uint32_t sensor_client_init(sensor_client_t * p_client, uint16_t element_index)
{
    if (!(p_client
       && p_client->settings.p_callbacks
       && p_client->settings.p_callbacks->sensor_status_cb
       && p_client->settings.p_callbacks->sensor_descriptor_status_cb
       && p_client->settings.p_callbacks->sensor_cadence_status_cb
       && p_client->settings.p_callbacks->sensor_column_status_cb
       && p_client->settings.p_callbacks->sensor_series_status_cb
       && p_client->settings.p_callbacks->sensor_settings_status_cb
       && p_client->settings.p_callbacks->sensor_setting_status_cb
       && p_client->settings.p_callbacks->periodic_publish_cb
       && p_client->p_message_buffer
       && p_client->message_buffer_bytes))
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->settings.timeout == 0)
    {
        p_client->settings.timeout = MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params =
    {
        .model_id = ACCESS_MODEL_SIG(SENSOR_CLIENT_MODEL_ID),
        .element_index = element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_client,
        .publish_timeout_cb = p_client->settings.p_callbacks->periodic_publish_cb
    };

    uint32_t status = access_model_add(&add_params, &p_client->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_client->model_handle);
    }

    return status;
}

uint32_t sensor_client_cadence_set(sensor_client_t * p_client,
                                   const sensor_cadence_set_msg_pkt_t * p_params,
                                   uint16_t length)
{
    if (!(p_client && p_params))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "NULL parameter (%p, %p) = (p_client, p_params)\n",
              p_client,
              p_params);

        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        __LOG(LOG_SRC_APP, LOG_LEVEL_ERROR,
              "access_reliable_model_is_free(%d) returned false\n",
              p_client->model_handle);

        return NRF_ERROR_BUSY;
    }

    uint8_t * p_message = NULL;
    uint32_t status;

    if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)p_params, length)))
    {
        return status;
    }

    NRF_MESH_ASSERT(p_message != NULL);
    message_create(p_client, SENSOR_OPCODE_CADENCE_SET, p_message, length,
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_CADENCE_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_cadence_set_unack(sensor_client_t * p_client,
                                         const sensor_cadence_set_msg_pkt_t * p_params,
                                         uint16_t length,
                                         uint8_t repeats)
{
    if (!(p_client && p_params))
    {
        return NRF_ERROR_NULL;
    }

    message_create(p_client, SENSOR_OPCODE_CADENCE_SET_UNACKNOWLEDGED, (const uint8_t *) p_params,
                   length, &p_client->access_message.message);

    uint32_t status = NRF_ERROR_INVALID_PARAM;
    for (uint32_t i = 0; i <= repeats; ++i)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }
    return status;
}

uint32_t sensor_client_setting_set(sensor_client_t * p_client,
                                   const sensor_setting_set_msg_pkt_t * p_params,
                                   uint16_t length)
{
    if (!(p_client && p_params))
    {
        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        return NRF_ERROR_BUSY;
    }

    uint8_t * p_message = NULL;
    uint32_t status;

    if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)p_params, length)))
    {
        return status;
    }

    NRF_MESH_ASSERT(p_message != NULL);
    message_create(p_client, SENSOR_OPCODE_SETTING_SET, (const uint8_t *) p_message, length,
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_SETTING_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_setting_set_unack(sensor_client_t * p_client,
                                         const sensor_setting_set_msg_pkt_t * p_params,
                                         uint16_t length,
                                         uint8_t repeats)
{
    if (!(p_client && p_params))
    {
        return NRF_ERROR_NULL;
    }

    message_create(p_client, SENSOR_OPCODE_SETTING_SET_UNACKNOWLEDGED, (const uint8_t *) p_params,
                   length, &p_client->access_message.message);

    uint32_t status = NRF_ERROR_INVALID_PARAM;
    for (uint32_t i = 0; i <= repeats; ++i)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }
    return status;
}

uint32_t sensor_client_get(sensor_client_t * p_client, uint16_t property_id)
{
    uint16_t buffered_bytes = 0;
    uint8_t * p_message = NULL;
    sensor_get_msg_pkt_t msg;
    uint32_t status;

    if (!(p_client && (p_client->message_buffer_bytes >= sizeof(property_id))))
    {
        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        return NRF_ERROR_BUSY;
    }

    if (property_id != 0)
    {
        msg.property_id = property_id;
        buffered_bytes = sizeof(sensor_get_msg_pkt_t);

        if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)&msg, buffered_bytes)))
        {
            return status;
        }
    }
    /* If property_id is 0, buffered_bytes is 0, so the message is correctly sent */

    message_create(p_client, SENSOR_OPCODE_GET, p_message, buffered_bytes,
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_descriptor_get(sensor_client_t * p_client, uint16_t property_id)
{
    sensor_descriptor_get_msg_pkt_t msg;

    if (!p_client)
    {
        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        return NRF_ERROR_BUSY;
    }

    uint8_t * p_message = NULL;
    uint16_t buffered_bytes = 0;

    if (property_id > 0)
    {
        msg.property_id = property_id;
        buffered_bytes = sizeof(sensor_descriptor_get_msg_pkt_t);
        uint32_t status = message_cache(p_client, &p_message, (uint8_t *)&msg, buffered_bytes);

        if (NRF_SUCCESS != status)
        {
            return status;
        }
    }
    /* If property_id is 0, buffered_bytes is 0, so the message is correctly sent */

    message_create(p_client, SENSOR_OPCODE_DESCRIPTOR_GET, p_message, buffered_bytes,
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_DESCRIPTOR_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_cadence_get(sensor_client_t * p_client, uint16_t property_id)
{
    uint8_t * p_message = NULL;
    uint32_t status;
    sensor_cadence_get_msg_pkt_t msg;

    if (!p_client)
    {
        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        return NRF_ERROR_BUSY;
    }

    msg.property_id = property_id;

    if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)&msg, sizeof(sensor_cadence_get_msg_pkt_t))))
    {
        return status;
    }

    message_create(p_client, SENSOR_OPCODE_CADENCE_GET, p_message, sizeof(sensor_cadence_get_msg_pkt_t),
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_CADENCE_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_settings_get(sensor_client_t * p_client, uint16_t property_id)
{
    uint8_t * p_message = NULL;
    uint32_t status;
    sensor_settings_get_msg_pkt_t msg;

    if (!p_client)
    {
        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        return NRF_ERROR_BUSY;
    }

    msg.property_id = property_id;

    if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)&msg, sizeof(sensor_settings_get_msg_pkt_t))))
    {
        return status;
    }

    message_create(p_client, SENSOR_OPCODE_SETTINGS_GET, p_message, sizeof(sensor_settings_get_msg_pkt_t),
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_SETTINGS_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_setting_get(sensor_client_t * p_client,
                                   uint16_t property_id,
                                   uint16_t setting_property_id)
{
    uint8_t * p_message = NULL;
    uint32_t status;
    sensor_setting_get_msg_pkt_t msg;

    if (!p_client)
    {
        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        return NRF_ERROR_BUSY;
    }

    msg.property_id = property_id;
    msg.setting_property_id = setting_property_id;

    if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)&msg, sizeof(sensor_setting_get_msg_pkt_t))))
    {
        return status;
    }

    message_create(p_client, SENSOR_OPCODE_SETTING_GET, p_message, sizeof(sensor_setting_get_msg_pkt_t),
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_SETTING_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_column_get(sensor_client_t * p_client, sensor_column_get_msg_pkt_t * p_rawbuf, uint16_t length)
{
    uint8_t * p_message = NULL;
    uint32_t status;

    if (!(p_client && p_rawbuf))
    {
        return NRF_ERROR_NULL;
    }

    if (!access_reliable_model_is_free(p_client->model_handle))
    {
        return NRF_ERROR_BUSY;
    }

    if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)p_rawbuf, length)))
    {
        return status;
    }

    message_create(p_client, SENSOR_OPCODE_COLUMN_GET, p_message, length,
                   &p_client->access_message.message);

    reliable_context_create(p_client, SENSOR_OPCODE_COLUMN_STATUS, &p_client->access_message);

    return access_model_reliable_publish(&p_client->access_message);
}

uint32_t sensor_client_series_get(sensor_client_t * p_client, sensor_series_get_msg_pkt_t * p_raw_colbuf, uint16_t length)
{
    uint8_t * p_message = NULL;
    uint32_t status;

    if (!(p_client && p_raw_colbuf))
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        if (NRF_SUCCESS != (status = message_cache(p_client, &p_message, (uint8_t *)p_raw_colbuf, length)))
        {
            return status;
        }

        message_create(p_client, SENSOR_OPCODE_SERIES_GET, p_message, length,
                       &p_client->access_message.message);

        reliable_context_create(p_client, SENSOR_OPCODE_SERIES_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}
