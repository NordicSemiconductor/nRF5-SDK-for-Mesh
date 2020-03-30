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

#include "generic_level_client.h"
#include "generic_level_common.h"
#include "generic_level_messages.h"
#include "model_common.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"


/** Opcode Handlers */

static void status_handle(access_model_handle_t handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_level_client_t * p_client = (generic_level_client_t *) p_args;
    generic_level_status_params_t in_data = {0};

    if (p_rx_msg->length == GENERIC_LEVEL_STATUS_MINLEN || p_rx_msg->length == GENERIC_LEVEL_STATUS_MAXLEN)
    {
        generic_level_status_msg_pkt_t * p_msg_params_packed = (generic_level_status_msg_pkt_t *) p_rx_msg->p_data;

        if (p_rx_msg->length == GENERIC_LEVEL_STATUS_MINLEN)
        {
            in_data.present_level = p_msg_params_packed->present_level;
            in_data.target_level = p_msg_params_packed->present_level;
            in_data.remaining_time_ms = 0;
        }
        else
        {
            in_data.present_level = p_msg_params_packed->present_level;
            in_data.target_level = p_msg_params_packed->target_level;
            in_data.remaining_time_ms = model_transition_time_decode(p_msg_params_packed->remaining_time);
        }

        p_client->settings.p_callbacks->level_status_cb(p_client, &p_rx_msg->meta_data, &in_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(GENERIC_LEVEL_OPCODE_STATUS), status_handle},
};

static uint8_t message_set_packet_create(generic_level_set_msg_pkt_t *p_set, const generic_level_set_params_t * p_params,
                                      const model_transition_t * p_transition)
{
        p_set->level = p_params->level;
        p_set->tid = p_params->tid;

        if (p_transition != NULL)
        {
            p_set->transition_time = model_transition_time_encode(p_transition->transition_time_ms);
            p_set->delay = model_delay_encode(p_transition->delay_ms);
            return GENERIC_LEVEL_SET_MAXLEN;
        }
        else
        {
            return GENERIC_LEVEL_SET_MINLEN;
        }
}

static uint8_t message_delta_set_packet_create(generic_level_delta_set_msg_pkt_t *p_delta_set, const generic_level_delta_set_params_t * p_params,
                                      const model_transition_t * p_transition)
{
        p_delta_set->delta_level = p_params->delta_level;
        p_delta_set->tid = p_params->tid;

        if (p_transition != NULL)
        {
            p_delta_set->transition_time = model_transition_time_encode(p_transition->transition_time_ms);
            p_delta_set->delay = model_delay_encode(p_transition->delay_ms);
            return GENERIC_LEVEL_DELTA_SET_MAXLEN;
        }
        else
        {
            return GENERIC_LEVEL_DELTA_SET_MINLEN;
        }
}

static uint8_t message_move_set_packet_create(generic_level_move_set_msg_pkt_t *p_move_set, const generic_level_move_set_params_t * p_params,
                                      const model_transition_t * p_transition)
{
        p_move_set->move_level = p_params->move_level;
        p_move_set->tid = p_params->tid;

        if (p_transition != NULL)
        {
            p_move_set->transition_time = model_transition_time_encode(p_transition->transition_time_ms);
            p_move_set->delay = model_delay_encode(p_transition->delay_ms);
            return GENERIC_LEVEL_MOVE_SET_MAXLEN;
        }
        else
        {
            return GENERIC_LEVEL_MOVE_SET_MINLEN;
        }
}

static void message_create(generic_level_client_t * p_client, uint16_t tx_opcode, const uint8_t * p_buffer,
                           uint16_t length, access_message_tx_t *p_message)
{
    p_message->opcode.opcode = tx_opcode;
    p_message->opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_message->p_buffer = p_buffer;
    p_message->length = length;
    p_message->force_segmented = p_client->settings.force_segmented;
    p_message->transmic_size = p_client->settings.transmic_size;
    p_message->access_token = nrf_mesh_unique_token_get();
}

static void reliable_context_create(generic_level_client_t * p_client, uint16_t reply_opcode,
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

uint32_t generic_level_client_init(generic_level_client_t * p_client, uint8_t element_index)
{
    if (p_client == NULL ||
        p_client->settings.p_callbacks == NULL ||
        p_client->settings.p_callbacks->level_status_cb == NULL ||
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
        .model_id = ACCESS_MODEL_SIG(GENERIC_LEVEL_CLIENT_MODEL_ID),
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

uint32_t generic_level_client_set(generic_level_client_t * p_client, const generic_level_set_params_t * p_params,
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
        uint8_t server_msg_length = message_set_packet_create(&p_client->msg_pkt.set, p_params, p_transition);

        message_create(p_client, GENERIC_LEVEL_OPCODE_SET, (const uint8_t *) &p_client->msg_pkt.set,
                        server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, GENERIC_LEVEL_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t generic_level_client_set_unack(generic_level_client_t * p_client, const generic_level_set_params_t * p_params,
                                        const model_transition_t * p_transition, uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    generic_level_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_set_packet_create(&msg, p_params, p_transition);

    message_create(p_client, GENERIC_LEVEL_OPCODE_SET_UNACKNOWLEDGED,
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

uint32_t generic_level_client_delta_set(generic_level_client_t * p_client, const generic_level_delta_set_params_t * p_params,
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
        uint8_t server_msg_length = message_delta_set_packet_create(&p_client->msg_pkt.delta_set, p_params, p_transition);

        message_create(p_client, GENERIC_LEVEL_OPCODE_DELTA_SET, (const uint8_t *) &p_client->msg_pkt.delta_set,
                        server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, GENERIC_LEVEL_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t generic_level_client_delta_set_unack(generic_level_client_t * p_client, const generic_level_delta_set_params_t * p_params,
                                        const model_transition_t * p_transition, uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    generic_level_delta_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_delta_set_packet_create(&msg, p_params, p_transition);

    message_create(p_client, GENERIC_LEVEL_OPCODE_DELTA_SET_UNACKNOWLEDGED,
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

uint32_t generic_level_client_move_set(generic_level_client_t * p_client, const generic_level_move_set_params_t * p_params,
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
        uint8_t server_msg_length = message_move_set_packet_create(&p_client->msg_pkt.move_set, p_params, p_transition);

        message_create(p_client, GENERIC_LEVEL_OPCODE_MOVE_SET, (const uint8_t *) &p_client->msg_pkt.move_set,
                        server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, GENERIC_LEVEL_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t generic_level_client_move_set_unack(generic_level_client_t * p_client, const generic_level_move_set_params_t * p_params,
                                        const model_transition_t * p_transition, uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition))
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    generic_level_move_set_msg_pkt_t msg;
    uint8_t server_msg_length = message_move_set_packet_create(&msg, p_params, p_transition);

    message_create(p_client, GENERIC_LEVEL_OPCODE_MOVE_SET_UNACKNOWLEDGED,
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

uint32_t generic_level_client_get(generic_level_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, GENERIC_LEVEL_OPCODE_GET, NULL, 0, &p_client->access_message.message);
        reliable_context_create(p_client, GENERIC_LEVEL_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

