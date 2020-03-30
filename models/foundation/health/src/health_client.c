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

#include <string.h>

#include "nrf_mesh_assert.h"
#include "access_reliable.h"
#include "access_config.h"
#include "mesh_mem.h"

#include "health_client.h"
#include "health_messages.h"
#include "health_opcodes.h"
#include "nrf_mesh_assert.h"

static void reliable_status_cb(access_model_handle_t model_handle, void * p_args, access_reliable_status_t status)
{
    health_client_t * p_client = p_args;
    health_client_evt_t event;

    if (p_client->p_buffer != NULL)
    {
        mesh_mem_free(p_client->p_buffer);
        p_client->p_buffer = NULL;
    }

    p_client->waiting_for_reply = false;

    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            /* Ignore */
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            event.type = HEALTH_CLIENT_EVT_TYPE_TIMEOUT;
            p_client->event_handler(p_client, &event);
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            event.type = HEALTH_CLIENT_EVT_TYPE_CANCELLED;
            p_client->event_handler(p_client, &event);
            break;
        default:
            NRF_MESH_ASSERT(false);
            break;
    }
}

static uint32_t reliable_send(health_client_t * p_client, uint16_t opcode, uint16_t reply_opcode,
        const void * p_message_data, uint16_t message_length)
{
    if (p_client->waiting_for_reply)
    {
        return NRF_ERROR_BUSY;
    }

    NRF_MESH_ASSERT(p_client->p_buffer == NULL);

    uint32_t status = NRF_SUCCESS;
    if (message_length > 0)
    {
        p_client->p_buffer = mesh_mem_alloc(message_length);
        if (p_client->p_buffer == NULL)
        {
            return NRF_ERROR_NO_MEM;
        }
        memcpy(p_client->p_buffer, p_message_data, message_length);
    }
    else
    {
        NRF_MESH_ASSERT(p_message_data == NULL);
    }

    access_reliable_t reliable =
    {
        .model_handle = p_client->model_handle,
        .message.p_buffer = p_client->p_buffer,
        .message.length = message_length,
        .message.opcode = ACCESS_OPCODE_SIG(opcode),
        .message.force_segmented = false,
        .message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT,
        .message.access_token = nrf_mesh_unique_token_get(), /*lint !e446: side effect in initializer */
        .reply_opcode = ACCESS_OPCODE_SIG(reply_opcode),
        .timeout = HEALTH_CLIENT_ACKED_TRANSACTION_TIMEOUT,
        .status_cb = reliable_status_cb
    };
    status = access_model_reliable_publish(&reliable);
    if (status != NRF_SUCCESS && p_client->p_buffer != NULL)
    {
        mesh_mem_free(p_client->p_buffer);
        p_client->p_buffer = NULL;
    }
    else if (status == NRF_SUCCESS)
    {
        p_client->waiting_for_reply = true;
    }

    return status;
}

/********** Incoming message handlers **********/

static void handle_fault_status(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length < sizeof(health_msg_fault_status_t))
    {
        return;
    }

    const health_msg_fault_status_t * p_pdu = (const health_msg_fault_status_t *) p_message->p_data;

    health_client_evt_t event;
    if (p_message->opcode.opcode == HEALTH_OPCODE_CURRENT_STATUS)
    {
        event.type = HEALTH_CLIENT_EVT_TYPE_CURRENT_STATUS_RECEIVED;
    }
    else
    {
        event.type = HEALTH_CLIENT_EVT_TYPE_FAULT_STATUS_RECEIVED;
    }

    event.p_meta_data = &p_message->meta_data;
    event.data.fault_status.test_id = p_pdu->test_id;
    event.data.fault_status.company_id = p_pdu->company_id;
    event.data.fault_status.fault_array_length = p_message->length - sizeof(health_msg_fault_status_t);
    event.data.fault_status.p_fault_array = event.data.fault_status.fault_array_length > 0 ? p_pdu->fault_array : NULL;
    event.data.fault_status.p_meta_data = &p_message->meta_data;

    const health_client_t * p_client = p_args;
    p_client->event_handler(p_client, &event);
}

static void handle_period_status(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(health_msg_period_status_t))
    {
        return;
    }

    const health_msg_period_status_t * p_pdu = (const health_msg_period_status_t *) p_message->p_data;

    health_client_evt_t event;
    event.type = HEALTH_CLIENT_EVT_TYPE_PERIOD_STATUS_RECEIVED;
    event.p_meta_data = &p_message->meta_data;
    event.data.period_status.fast_period_divisor = p_pdu->fast_period_divisor;

    const health_client_t * p_client = p_args;
    p_client->event_handler(p_client, &event);
}

static void handle_attention_status(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    if (p_message->length != sizeof(health_msg_attention_status_t))
    {
        return;
    }

    const health_msg_attention_status_t * p_pdu = (const health_msg_attention_status_t *) p_message->p_data;

    health_client_evt_t event;
    event.type = HEALTH_CLIENT_EVT_TYPE_ATTENTION_STATUS_RECEIVED;
    event.p_meta_data = &p_message->meta_data;
    event.data.attention_status.attention = p_pdu->attention;

    const health_client_t * p_client = p_args;
    p_client->event_handler(p_client, &event);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    { ACCESS_OPCODE_SIG(HEALTH_OPCODE_CURRENT_STATUS),   handle_fault_status },
    { ACCESS_OPCODE_SIG(HEALTH_OPCODE_FAULT_STATUS),     handle_fault_status },
    { ACCESS_OPCODE_SIG(HEALTH_OPCODE_PERIOD_STATUS),    handle_period_status },
    { ACCESS_OPCODE_SIG(HEALTH_OPCODE_ATTENTION_STATUS), handle_attention_status },
};

/********** Interface functions **********/

uint32_t health_client_fault_get(health_client_t * p_client, uint16_t company_id)
{
    health_msg_fault_get_t message = { .company_id = company_id };
    return reliable_send(p_client, HEALTH_OPCODE_FAULT_GET, HEALTH_OPCODE_FAULT_STATUS, &message, sizeof(health_msg_fault_get_t));
}

uint32_t health_client_fault_clear(health_client_t * p_client, uint16_t company_id, bool acked)
{
    health_msg_fault_clear_t message = { .company_id = company_id };

    if (!acked)
    {
        access_message_tx_t packet =
        {
            .opcode = ACCESS_OPCODE_SIG(HEALTH_OPCODE_FAULT_CLEAR_UNACKED),
            .p_buffer = (const uint8_t *) &message,
            .length = sizeof(health_msg_fault_clear_t),
            .force_segmented = false,
            .transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT,
            .access_token = nrf_mesh_unique_token_get()
        }; /*lint !e446: side effect in initializer */
        return access_model_publish(p_client->model_handle, &packet);
    }
    else
    {
        return reliable_send(p_client, HEALTH_OPCODE_FAULT_CLEAR, HEALTH_OPCODE_FAULT_STATUS, &message, sizeof(health_msg_fault_clear_t));
    }
}

uint32_t health_client_fault_test(health_client_t * p_client, uint16_t company_id, uint8_t test_id, bool acked)
{
    health_msg_fault_test_t message =
    {
        .test_id = test_id,
        .company_id = company_id
    };

    if (!acked)
    {
        access_message_tx_t packet =
        {
            .opcode = ACCESS_OPCODE_SIG(HEALTH_OPCODE_FAULT_TEST_UNACKED),
            .p_buffer = (const uint8_t *) &message,
            .length = sizeof(health_msg_fault_test_t),
            .force_segmented = false,
            .transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT,
            .access_token = nrf_mesh_unique_token_get()
        }; /*lint !e446: side effect in initializer */
        return access_model_publish(p_client->model_handle, &packet);
    }
    else
    {
        return reliable_send(p_client, HEALTH_OPCODE_FAULT_TEST, HEALTH_OPCODE_FAULT_STATUS, &message, sizeof(health_msg_fault_test_t));
    }
}

uint32_t health_client_period_get(health_client_t * p_client)
{
    return reliable_send(p_client, HEALTH_OPCODE_PERIOD_GET, HEALTH_OPCODE_PERIOD_STATUS, NULL, 0);
}

uint32_t health_client_period_set(health_client_t * p_client, uint8_t fast_period_divisor, bool acked)
{
    health_msg_period_set_t message = { .fast_period_divisor = fast_period_divisor };
    if (!acked)
    {
        access_message_tx_t packet =
        {
            .opcode = ACCESS_OPCODE_SIG(HEALTH_OPCODE_PERIOD_SET_UNACKED),
            .p_buffer = (const uint8_t *) &message,
            .length = sizeof(health_msg_period_set_t),
            .force_segmented = false,
            .transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT,
            .access_token = nrf_mesh_unique_token_get()
        }; /*lint !e446: side effect in initializer */
        return access_model_publish(p_client->model_handle, &packet);
    }
    else
    {
        return reliable_send(p_client, HEALTH_OPCODE_PERIOD_SET, HEALTH_OPCODE_PERIOD_STATUS, &message, sizeof(health_msg_period_set_t));
    }
}

uint32_t health_client_attention_get(health_client_t * p_client)
{
    return reliable_send(p_client, HEALTH_OPCODE_ATTENTION_GET, HEALTH_OPCODE_ATTENTION_STATUS, NULL, 0);
}

uint32_t health_client_attention_set(health_client_t * p_client, uint8_t attention_timer, bool acked)
{
    health_msg_attention_set_t message = { .attention = attention_timer };
    if (!acked)
    {
        access_message_tx_t packet =
        {
            .opcode = ACCESS_OPCODE_SIG(HEALTH_OPCODE_ATTENTION_SET_UNACKED),
            .p_buffer = (const uint8_t *) &message,
            .length = sizeof(health_msg_attention_set_t),
            .force_segmented = false,
            .transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT,
            .access_token = nrf_mesh_unique_token_get()
        }; /*lint !e446: side effect in initializer */
        return access_model_publish(p_client->model_handle, &packet);
    }
    else
    {
        return reliable_send(p_client, HEALTH_OPCODE_ATTENTION_SET, HEALTH_OPCODE_ATTENTION_STATUS, &message, sizeof(health_msg_attention_set_t));
    }
}

uint32_t health_client_init(health_client_t * p_client, uint16_t element_index, health_client_evt_cb_t evt_handler)
{
    if (evt_handler == NULL)
    {
        return NRF_ERROR_NULL;
    }

    p_client->event_handler = evt_handler;
    p_client->waiting_for_reply = false;
    p_client->p_buffer = NULL;

    access_model_add_params_t add_params =
    {
        .element_index = element_index,
        .model_id = ACCESS_MODEL_SIG(HEALTH_CLIENT_MODEL_ID), /*lint !e64 Type Mismatch */
        .p_opcode_handlers = m_opcode_handlers,
        .opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]),
        .p_args = p_client
    };
    uint32_t status = access_model_add(&add_params, &p_client->model_handle);
    if (status != NRF_SUCCESS)
    {
        return status;
    }

    return access_model_subscription_list_alloc(p_client->model_handle);
}

void health_client_pending_msg_cancel(health_client_t * p_client)
{
    (void)access_model_reliable_cancel(p_client->model_handle);
}
