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

#include "scene_client.h"
#include "model_common.h"

#include "access.h"
#include "access_config.h"
#include "utils.h"

/** Opcode Handlers */
static void status_handle(access_model_handle_t handle,
                          const access_message_rx_t * p_rx_msg,
                          void * p_args)
{
    scene_client_t * p_client = (scene_client_t *) p_args;
    scene_status_params_t in_data = {0};

    if ((p_rx_msg->length == SCENE_STATUS_MINLEN) || (p_rx_msg->length == SCENE_STATUS_MAXLEN))
    {
        scene_status_msg_pkt_t * p_msg_params_packed = (scene_status_msg_pkt_t *) p_rx_msg->p_data;

        in_data.status_code = p_msg_params_packed->status_code;
        in_data.current_scene = p_msg_params_packed->current_scene;

        if (p_rx_msg->length == SCENE_STATUS_MINLEN)
        {
            in_data.target_scene = SCENE_NUMBER_NO_SCENE;
            in_data.remaining_time_ms = 0;
        }
        else
        {
            in_data.target_scene = p_msg_params_packed->target_scene;
            in_data.remaining_time_ms =
                    model_transition_time_decode(p_msg_params_packed->remaining_time);
        }

        p_client->settings.p_callbacks->scene_status_cb(p_client, &p_rx_msg->meta_data, &in_data);
    }
}

static void register_status_handle(access_model_handle_t handle,
                                   const access_message_rx_t * p_rx_msg, 
                                   void * p_args)
{
    scene_client_t * p_client = (scene_client_t *) p_args;
    scene_register_status_params_t in_data = {0};

    if (IS_IN_RANGE(p_rx_msg->length, SCENE_REGISTER_STATUS_MINLEN, SCENE_REGISTER_STATUS_MAXLEN))
    {
        scene_register_status_msg_pkt_t * p_msg_params_packed =
                    (scene_register_status_msg_pkt_t *) p_rx_msg->p_data;

        in_data.status_code = p_msg_params_packed->status_code;
        in_data.current_scene = p_msg_params_packed->current_scene;

        if (p_rx_msg->length > SCENE_REGISTER_STATUS_MINLEN)
        {
            memcpy(in_data.scenes, (const void *)p_msg_params_packed->scenes,
                   (p_rx_msg->length - SCENE_REGISTER_STATUS_MINLEN));
        }

        p_client->settings.p_callbacks->scene_register_status_cb(p_client,
                                                                 &p_rx_msg->meta_data,
                                                                 &in_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_STATUS), status_handle},
    {ACCESS_OPCODE_SIG(SCENE_OPCODE_REGISTER_STATUS), register_status_handle},
};

static void message_create(scene_client_t * p_client, uint16_t tx_opcode, const uint8_t * p_buffer,
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

static void reliable_context_create(scene_client_t * p_client, uint16_t reply_opcode,
                                    access_reliable_t * p_reliable)
{
    p_reliable->model_handle = p_client->model_handle;
    p_reliable->reply_opcode.opcode = reply_opcode;
    p_reliable->reply_opcode.company_id = ACCESS_COMPANY_ID_NONE;
    p_reliable->timeout = p_client->settings.timeout;
    p_reliable->status_cb = p_client->settings.p_callbacks->ack_transaction_status_cb;
}

static uint8_t message_recall_packet_create(scene_recall_msg_pkt_t * p_recall,
                                            const scene_recall_params_t * p_params,
                                            const model_transition_t * p_transition)
{
        p_recall->scene_number = p_params->scene_number;
        p_recall->tid = p_params->tid;

        if (p_transition != NULL)
        {
            p_recall->transition_time =
                    model_transition_time_encode(p_transition->transition_time_ms);
            p_recall->delay = model_delay_encode(p_transition->delay_ms);
            return SCENE_RECALL_MAXLEN;
        }
        else
        {
            return SCENE_RECALL_MINLEN;
        }
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
uint32_t scene_client_init(scene_client_t * p_client, uint8_t element_index)
{
    if (p_client == NULL ||
        p_client->settings.p_callbacks == NULL ||
        p_client->settings.p_callbacks->scene_status_cb == NULL ||
        p_client->settings.p_callbacks->scene_register_status_cb == NULL ||
        p_client->settings.p_callbacks->periodic_publish_cb == NULL ||
        p_client->settings.p_callbacks->ack_transaction_status_cb == NULL )
    {
        return NRF_ERROR_NULL;
    }

    if (p_client->settings.timeout == 0)
    {
        p_client->settings.timeout = MODEL_ACKNOWLEDGED_TRANSACTION_TIMEOUT;
    }

    access_model_add_params_t add_params =
    {
        .model_id = ACCESS_MODEL_SIG(SCENE_CLIENT_MODEL_ID),
        .element_index = element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_client,
        .publish_timeout_cb = p_client->settings.p_callbacks->periodic_publish_cb
    };

    uint32_t status  = access_model_add(&add_params, &p_client->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_client->model_handle);
    }

    return status;
}

uint32_t scene_client_store(scene_client_t * p_client, const scene_store_params_t * p_params)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_params->scene_number == SCENE_NUMBER_NO_SCENE)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        p_client->msg_pkt.store.scene_number = p_params->scene_number;

        message_create(p_client, SCENE_OPCODE_STORE, (const uint8_t *) &p_client->msg_pkt.store,
                        sizeof(p_client->msg_pkt.store), &p_client->access_message.message);
        reliable_context_create(p_client, SCENE_OPCODE_REGISTER_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t scene_client_store_unack(scene_client_t * p_client, const scene_store_params_t * p_params,
                                  uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_params->scene_number == SCENE_NUMBER_NO_SCENE)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    scene_store_msg_pkt_t msg;
    msg.scene_number = p_params->scene_number;

    message_create(p_client, SCENE_OPCODE_STORE_UNACKNOWLEDGED, (const uint8_t *) &msg,
                    sizeof(scene_store_msg_pkt_t), &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }

    return status;
}

uint32_t scene_client_delete(scene_client_t * p_client, const scene_delete_params_t * p_params)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_params->scene_number == SCENE_NUMBER_NO_SCENE)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        p_client->msg_pkt.store.scene_number = p_params->scene_number;

        message_create(p_client, SCENE_OPCODE_DELETE, (const uint8_t *) &p_client->msg_pkt.store,
                        sizeof(p_client->msg_pkt.store), &p_client->access_message.message);
        reliable_context_create(p_client, SCENE_OPCODE_REGISTER_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t scene_client_delete_unack(scene_client_t * p_client, const scene_delete_params_t * p_params,
                                   uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_params->scene_number == SCENE_NUMBER_NO_SCENE)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    scene_delete_msg_pkt_t msg;
    msg.scene_number = p_params->scene_number;

    message_create(p_client, SCENE_OPCODE_DELETE_UNACKNOWLEDGED, (const uint8_t *) &msg,
                    sizeof(scene_store_msg_pkt_t), &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }

    return status;
}

uint32_t scene_client_recall(scene_client_t * p_client, const scene_recall_params_t * p_params,
                             const model_transition_t * p_transition)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition) || p_params->scene_number == SCENE_NUMBER_NO_SCENE)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        uint8_t server_msg_length =
                message_recall_packet_create(&p_client->msg_pkt.recall, p_params, p_transition);
        
        message_create(p_client, SCENE_OPCODE_RECALL, (const uint8_t *) &p_client->msg_pkt.recall,
                        server_msg_length, &p_client->access_message.message);
        reliable_context_create(p_client, SCENE_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t scene_client_recall_unack(scene_client_t * p_client, const scene_recall_params_t * p_params,
                                   const model_transition_t * p_transition, uint8_t repeats)
{
    if (p_client == NULL || p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (is_p_transition_invalid(p_transition) || p_params->scene_number == SCENE_NUMBER_NO_SCENE)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    scene_recall_msg_pkt_t msg;
    uint8_t server_msg_length = message_recall_packet_create(&msg, p_params, p_transition);

    message_create(p_client, SCENE_OPCODE_RECALL_UNACKNOWLEDGED,
                   (const uint8_t *) &msg, server_msg_length, &p_client->access_message.message);

    uint32_t status = NRF_SUCCESS;
    repeats++;
    while (repeats-- > 0 && status == NRF_SUCCESS)
    {
        status = access_model_publish(p_client->model_handle, &p_client->access_message.message);
    }

    return NRF_SUCCESS;
}

uint32_t scene_client_get(scene_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, SCENE_OPCODE_GET, NULL, 0, &p_client->access_message.message);
        reliable_context_create(p_client, SCENE_OPCODE_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}

uint32_t scene_client_register_get(scene_client_t * p_client)
{
    if (p_client == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (access_reliable_model_is_free(p_client->model_handle))
    {
        message_create(p_client, SCENE_OPCODE_REGISTER_GET, NULL, 0,
                       &p_client->access_message.message);
        reliable_context_create(p_client, SCENE_OPCODE_REGISTER_STATUS, &p_client->access_message);

        return access_model_reliable_publish(&p_client->access_message);
    }
    else
    {
        return NRF_ERROR_BUSY;
    }
}
