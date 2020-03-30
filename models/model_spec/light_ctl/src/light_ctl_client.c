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

#include "light_ctl_client.h"
#include "model_common.h"

#include <stdint.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"
#include "light_ctl_utils.h"

/** Opcode Handlers */

static void status_handle(access_model_handle_t handle,
                          const access_message_rx_t * p_rx_msg, void * p_args)
{
    light_ctl_client_t * p_client = (light_ctl_client_t *) p_args;
    light_ctl_status_params_t in_data = {0};

    if (p_rx_msg->length == LIGHT_CTL_STATUS_MINLEN ||
        p_rx_msg->length == LIGHT_CTL_STATUS_MAXLEN)
    {
        light_ctl_status_msg_pkt_t * p_msg_params_packed =
            (light_ctl_status_msg_pkt_t *) p_rx_msg->p_data;

        in_data.present_lightness = p_msg_params_packed->present_lightness;
        in_data.present_temperature32 =
            light_ctl_utils_temperature_to_temperature32(p_msg_params_packed->present_temperature);

        if (p_rx_msg->length == LIGHT_CTL_STATUS_MINLEN)
        {
            in_data.target_lightness = in_data.present_lightness;
            in_data.target_temperature32 = in_data.present_temperature32;
            in_data.remaining_time_ms = 0;
        }
        else
        {
            in_data.target_lightness = p_msg_params_packed->target_lightness;
            in_data.target_temperature32 =
                light_ctl_utils_temperature_to_temperature32(p_msg_params_packed->target_temperature);
            in_data.remaining_time_ms =
                model_transition_time_decode(p_msg_params_packed->remaining_time);
        }

        p_client->settings.p_callbacks->light_ctl_status_cb(p_client,
                                                            &p_rx_msg->meta_data,
                                                            &in_data);
    }
}

static void temperature_status_handle(access_model_handle_t handle,
                                 const access_message_rx_t * p_rx_msg,
                                 void * p_args)
{
    light_ctl_client_t * p_client = (light_ctl_client_t *) p_args;
    light_ctl_temperature_status_params_t in_data = {0};

    if (p_rx_msg->length == LIGHT_CTL_TEMPERATURE_STATUS_MINLEN ||
        p_rx_msg->length == LIGHT_CTL_TEMPERATURE_STATUS_MAXLEN)
    {
        light_ctl_temperature_status_msg_pkt_t * p_msg_params_packed =
            (light_ctl_temperature_status_msg_pkt_t *) p_rx_msg->p_data;

        in_data.present_temperature32 =
            light_ctl_utils_temperature_to_temperature32(p_msg_params_packed->present_temperature);
        in_data.present_delta_uv = p_msg_params_packed->present_delta_uv;
        if (p_rx_msg->length == LIGHT_CTL_TEMPERATURE_STATUS_MINLEN)
        {
            in_data.target_temperature32 = in_data.present_temperature32;
            in_data.target_delta_uv = in_data.present_delta_uv;
            in_data.remaining_time_ms = 0;
        }
        else
        {
            in_data.target_temperature32 =
                light_ctl_utils_temperature_to_temperature32(p_msg_params_packed->target_temperature);
            in_data.target_delta_uv = p_msg_params_packed->target_delta_uv;
            in_data.remaining_time_ms =
                model_transition_time_decode(p_msg_params_packed->remaining_time);
        }

        p_client->settings.p_callbacks->light_ctl_temperature_status_cb(p_client,
                                                                        &p_rx_msg->meta_data,
                                                                        &in_data);
    }
}

static void default_status_handle(access_model_handle_t handle,
                                  const access_message_rx_t * p_rx_msg,
                                  void * p_args)
{
    light_ctl_client_t * p_client = (light_ctl_client_t *) p_args;
    light_ctl_default_status_params_t in_data = {0};

    if (p_rx_msg->length == LIGHT_CTL_DEFAULT_STATUS_LEN)
    {
        light_ctl_default_status_msg_pkt_t * p_msg_params_packed =
            (light_ctl_default_status_msg_pkt_t *) p_rx_msg->p_data;

        in_data.lightness = p_msg_params_packed->lightness;
        in_data.temperature32 = light_ctl_utils_temperature_to_temperature32(p_msg_params_packed->temperature);
        in_data.delta_uv = p_msg_params_packed->delta_uv;

        p_client->settings.p_callbacks->light_ctl_default_status_cb(p_client,
                                                                    &p_rx_msg->meta_data,
                                                                    &in_data);
    }
}

static void temperature_range_status_handle(access_model_handle_t handle,
                                            const access_message_rx_t * p_rx_msg,
                                            void * p_args)
{
    light_ctl_client_t * p_client = (light_ctl_client_t *) p_args;
    light_ctl_temperature_range_status_params_t in_data = {0};

    if (p_rx_msg->length == LIGHT_CTL_TEMPERATURE_RANGE_STATUS_LEN)
    {
        light_ctl_temperature_range_status_msg_pkt_t * p_msg_params_packed =
            (light_ctl_temperature_range_status_msg_pkt_t *) p_rx_msg->p_data;

        in_data.status_code = p_msg_params_packed->status_code;
        in_data.temperature32_range_min =
            light_ctl_utils_temperature_to_temperature32(p_msg_params_packed->range_min);
        in_data.temperature32_range_max =
            light_ctl_utils_temperature_to_temperature32(p_msg_params_packed->range_max);
        p_client->settings.p_callbacks->light_ctl_temperature_range_status_cb(p_client,
                                                                              &p_rx_msg->meta_data,
                                                                              &in_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_STATUS), status_handle},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_STATUS), temperature_status_handle},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_DEFAULT_STATUS), default_status_handle},
    {ACCESS_OPCODE_SIG(LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_STATUS), temperature_range_status_handle},
};

static uint8_t message_set_packet_create(light_ctl_set_msg_pkt_t * p_set,
                                         const light_ctl_set_params_t * p_params,
                                         const model_transition_t * p_transition)
{
    p_set->lightness = p_params->lightness;
    p_set->temperature = light_ctl_utils_temperature32_to_temperature(p_params->temperature32);
    p_set->delta_uv = p_params->delta_uv;
    p_set->tid = p_params->tid;

    if (p_transition != NULL)
    {
        p_set->transition_time =
            model_transition_time_encode(p_transition->transition_time_ms);
        p_set->delay = model_delay_encode(p_transition->delay_ms);
        return LIGHT_CTL_SET_MAXLEN;
    }
    else
    {
        return LIGHT_CTL_SET_MINLEN;
    }
}

static uint8_t message_temperature_set_packet_create(light_ctl_temperature_set_msg_pkt_t * p_set,
                                                     const light_ctl_temperature_set_params_t * p_params,
                                                     const model_transition_t * p_transition)
{
    p_set->temperature = light_ctl_utils_temperature32_to_temperature(p_params->temperature32);
    p_set->delta_uv = p_params->delta_uv;
    p_set->tid = p_params->tid;

    if (p_transition != NULL)
    {
        p_set->transition_time =
            model_transition_time_encode(p_transition->transition_time_ms);
        p_set->delay = model_delay_encode(p_transition->delay_ms);
        return LIGHT_CTL_TEMPERATURE_SET_MAXLEN;
    }
    else
    {
        return LIGHT_CTL_TEMPERATURE_SET_MINLEN;
    }
}

static uint8_t message_default_set_packet_create(light_ctl_default_set_msg_pkt_t * p_set,
                                                 const light_ctl_default_set_params_t * p_params)
{
    p_set->lightness = p_params->lightness;
    p_set->temperature = light_ctl_utils_temperature32_to_temperature(p_params->temperature32);
    p_set->delta_uv = p_params->delta_uv;

    return LIGHT_CTL_DEFAULT_SET_LEN;
}

static uint8_t message_temperature_range_set_packet_create(light_ctl_temperature_range_set_msg_pkt_t * p_set,
                                        const light_ctl_temperature_range_set_params_t * p_params)
{
    p_set->range_min =
        light_ctl_utils_temperature32_to_temperature(p_params->temperature32_range_min);
    p_set->range_max =
        light_ctl_utils_temperature32_to_temperature(p_params->temperature32_range_max);

    return LIGHT_CTL_TEMPERATURE_RANGE_SET_LEN;
}

static void message_create(light_ctl_client_t * p_client,
                           uint16_t tx_opcode, const uint8_t * p_buffer,
                           uint16_t length, access_message_tx_t * p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_client->settings.force_segmented;
    p_message->transmic_size = p_client->settings.transmic_size;
    p_message->access_token = nrf_mesh_unique_token_get();
}

static void reliable_context_create(light_ctl_client_t * p_client,
                                    uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_client->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->timeout = p_client->settings.timeout;
    p_reliable->status_cb = p_client->settings.p_callbacks->ack_transaction_status_cb;
}

static bool is_p_transition_invalid(const model_transition_t * p_transition)
{
    if (p_transition != NULL &&
        (p_transition->transition_time_ms > TRANSITION_TIME_MAX_MS ||
         p_transition->delay_ms > DELAY_TIME_MAX_MS))
    {
        return true;
    }

    return false;
}

/** Interface functions */


uint32_t light_ctl_client_init(light_ctl_client_t * p_client, uint8_t element_index)
{
    if (p_client == NULL ||
        p_client->settings.p_callbacks == NULL ||
        p_client->settings.p_callbacks->light_ctl_status_cb == NULL ||
        p_client->settings.p_callbacks->light_ctl_temperature_status_cb == NULL ||
        p_client->settings.p_callbacks->light_ctl_default_status_cb == NULL ||
        p_client->settings.p_callbacks->light_ctl_temperature_range_status_cb == NULL ||
        p_client->settings.p_callbacks->periodic_publish_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->settings.timeout == 0)
    {
        p_client->settings.timeout = MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params =
    {
        .model_id = ACCESS_MODEL_SIG(LIGHT_CTL_CLIENT_MODEL_ID),
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


uint32_t light_ctl_client_set(light_ctl_client_t * p_client,
                              const light_ctl_set_params_t * p_params,
                              const model_transition_t * p_transition)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params,
                                                              p_transition);

        message_create(p_client, LIGHT_CTL_OPCODE_SET,
                       (const uint8_t *) &p_client->msg_pkt.set,
                       server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}


uint32_t light_ctl_client_set_unack(light_ctl_client_t * p_client,
                                          const light_ctl_set_params_t * p_params,
                                          const model_transition_t * p_transition,
                                          uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    light_ctl_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_set_packet_create(&msg, p_params, p_transition);

    message_create(p_client, LIGHT_CTL_OPCODE_SET_UNACKNOWLEDGED,
                   (const uint8_t *) &msg, server_msg_length,
                   &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }
    return status;
}

uint32_t light_ctl_client_temperature_set(light_ctl_client_t * p_client,
                                          const light_ctl_temperature_set_params_t * p_params,
                                          const model_transition_t * p_transition)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length =
            message_temperature_set_packet_create(&p_client->msg_pkt.temperature_set,
                                                  p_params, p_transition);

        message_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_SET,
                       (const uint8_t *) &p_client->msg_pkt.temperature_set,
                        server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t light_ctl_client_temperature_set_unack(light_ctl_client_t * p_client,
                                                const light_ctl_temperature_set_params_t * p_params,
                                                const model_transition_t * p_transition,
                                                uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    light_ctl_temperature_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_temperature_set_packet_create(&msg, p_params, p_transition);

    message_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_SET_UNACKNOWLEDGED,
                   (const uint8_t *) &msg,
                   server_msg_length, &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }
    return status;
}

uint32_t light_ctl_client_default_set(light_ctl_client_t * p_client,
                                      const light_ctl_default_set_params_t * p_params)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length =
            message_default_set_packet_create(&p_client->msg_pkt.default_set, p_params);

        message_create(p_client, LIGHT_CTL_OPCODE_DEFAULT_SET,
                       (const uint8_t *) &p_client->msg_pkt.default_set,
                       server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_DEFAULT_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t light_ctl_client_default_set_unack(light_ctl_client_t * p_client,
                                            const light_ctl_default_set_params_t * p_params,
                                            uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    light_ctl_default_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_default_set_packet_create(&msg, p_params);

    message_create(p_client, LIGHT_CTL_OPCODE_DEFAULT_SET_UNACKNOWLEDGED,
                   (const uint8_t *) &msg,
                   server_msg_length, &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }
    return status;
}

uint32_t light_ctl_client_temperature_range_set(light_ctl_client_t * p_client,
                                                const light_ctl_temperature_range_set_params_t * p_params)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_params->temperature32_range_min < light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT) ||
        p_params->temperature32_range_max > light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length =
            message_temperature_range_set_packet_create(&p_client->msg_pkt.temperature_range_set,
                                                        p_params);

        message_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_SET,
                       (const uint8_t *) &p_client->msg_pkt.temperature_range_set,
                       server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t light_ctl_client_temperature_range_set_unack(light_ctl_client_t * p_client,
                                                const light_ctl_temperature_range_set_params_t * p_params,
                                                uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_params->temperature32_range_min < light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MIN_LIMIT) ||
        p_params->temperature32_range_max > light_ctl_utils_temperature_to_temperature32(LIGHT_CTL_TEMPERATURE_MAX_LIMIT))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    light_ctl_temperature_range_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_temperature_range_set_packet_create(&msg, p_params);

    message_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_SET_UNACKNOWLEDGED,
                   (const uint8_t *) &msg,
                   server_msg_length, &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }
    return status;
}

uint32_t light_ctl_client_get(light_ctl_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, LIGHT_CTL_OPCODE_GET, NULL, 0,
                       &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t light_ctl_client_temperature_get(light_ctl_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_GET, NULL, 0,
                       &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t light_ctl_client_default_get(light_ctl_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, LIGHT_CTL_OPCODE_DEFAULT_GET, NULL, 0,
                       &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_DEFAULT_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t light_ctl_client_temperature_range_get(light_ctl_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_GET, NULL, 0,
                       &p_client->access_message.message);
        reliable_context_create(p_client, LIGHT_CTL_OPCODE_TEMPERATURE_RANGE_STATUS,
                                &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}


