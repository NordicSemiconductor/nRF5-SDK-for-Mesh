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

#include "simple_on_off_client.h"
#include "simple_on_off_common.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "access_config.h"
#include "access_reliable.h"
#include "device_state_manager.h"
#include "nrf_mesh.h"
#include "nrf_mesh_assert.h"
#include "log.h"

/*****************************************************************************
 * Static variables
 *****************************************************************************/

/** Keeps a single global TID for all transfers. */
static uint8_t m_tid;

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reliable_status_cb(access_model_handle_t model_handle,
                               void * p_args,
                               access_reliable_status_t status)
{
    simple_on_off_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->status_cb != NULL);

    p_client->state.reliable_transfer_active = false;
    switch (status)
    {
        case ACCESS_RELIABLE_TRANSFER_SUCCESS:
            /* Ignore */
            break;
        case ACCESS_RELIABLE_TRANSFER_TIMEOUT:
            p_client->status_cb(p_client, SIMPLE_ON_OFF_STATUS_ERROR_NO_REPLY, NRF_MESH_ADDR_UNASSIGNED);
            break;
        case ACCESS_RELIABLE_TRANSFER_CANCELLED:
            p_client->status_cb(p_client, SIMPLE_ON_OFF_STATUS_CANCELLED, NRF_MESH_ADDR_UNASSIGNED);
            break;
        default:
            /* Should not be possible. */
            NRF_MESH_ASSERT(false);
            break;
    }
}

static uint32_t send_reliable_message(const simple_on_off_client_t * p_client,
                                      simple_on_off_opcode_t opcode,
                                      const uint8_t * p_data,
                                      uint16_t length)
{
    access_reliable_t reliable;
    reliable.model_handle = p_client->model_handle;
    reliable.message.p_buffer = p_data;
    reliable.message.length = length;
    reliable.message.opcode.opcode = opcode;
    reliable.message.opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    reliable.message.force_segmented = false;
    reliable.message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reliable.message.access_token = nrf_mesh_unique_token_get();
    reliable.reply_opcode.opcode = SIMPLE_ON_OFF_OPCODE_STATUS;
    reliable.reply_opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    reliable.timeout = SIMPLE_ON_OFF_CLIENT_ACKED_TRANSACTION_TIMEOUT;
    reliable.status_cb = reliable_status_cb;

    return access_model_reliable_publish(&reliable);
}

/*****************************************************************************
 * Opcode handler callback(s)
 *****************************************************************************/

static void handle_status_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_client_t * p_client = p_args;
    NRF_MESH_ASSERT(p_client->status_cb != NULL);

    simple_on_off_msg_status_t * p_status =
        (simple_on_off_msg_status_t *) p_message->p_data;
    simple_on_off_status_t on_off_status = (p_status->present_on_off ?
                                              SIMPLE_ON_OFF_STATUS_ON : SIMPLE_ON_OFF_STATUS_OFF);
    p_client->status_cb(p_client, on_off_status, p_message->meta_data.src.value);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {{SIMPLE_ON_OFF_OPCODE_STATUS, SIMPLE_ON_OFF_COMPANY_ID}, handle_status_cb}
};

static void handle_publish_timeout(access_model_handle_t handle, void * p_args)
{
    simple_on_off_client_t * p_client = p_args;

    if (p_client->timeout_cb != NULL)
    {
        p_client->timeout_cb(handle, p_args);
    }
}
/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t simple_on_off_client_init(simple_on_off_client_t * p_client, uint16_t element_index)
{
    if (p_client == NULL ||
        p_client->status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.model_id.model_id = SIMPLE_ON_OFF_CLIENT_MODEL_ID;
    init_params.model_id.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    init_params.element_index = element_index;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_client;
    init_params.publish_timeout_cb = handle_publish_timeout;
    return access_model_add(&init_params, &p_client->model_handle);
}

uint32_t simple_on_off_client_set(simple_on_off_client_t * p_client, bool on_off)
{
    if (p_client == NULL || p_client->status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_client->state.reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    p_client->state.data.on_off = on_off ? 1 : 0;
    p_client->state.data.tid = m_tid++;

    uint32_t status = send_reliable_message(p_client,
                                            SIMPLE_ON_OFF_OPCODE_SET,
                                            (const uint8_t *)&p_client->state.data,
                                            sizeof(simple_on_off_msg_set_t));
    if (status == NRF_SUCCESS)
    {
        p_client->state.reliable_transfer_active = true;
    }
    return status;
}

uint32_t simple_on_off_client_set_unreliable(simple_on_off_client_t * p_client, bool on_off, uint8_t repeats)
{
    simple_on_off_msg_set_unreliable_t set_unreliable;
    set_unreliable.on_off = on_off ? 1 : 0;
    set_unreliable.tid = m_tid++;

    access_message_tx_t message;
    message.opcode.opcode = SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE;
    message.opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    message.p_buffer = (const uint8_t*) &set_unreliable;
    message.length = sizeof(set_unreliable);
    message.force_segmented = false;
    message.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    uint32_t status = NRF_SUCCESS;
    for (uint8_t i = 0; i < repeats; ++i)
    {
        message.access_token = nrf_mesh_unique_token_get();
        status = access_model_publish(p_client->model_handle, &message);
        if (status != NRF_SUCCESS)
        {
            break;
        }
    }
    return status;
}

uint32_t simple_on_off_client_get(simple_on_off_client_t * p_client)
{
    if (p_client == NULL || p_client->status_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    else if (p_client->state.reliable_transfer_active)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t status = send_reliable_message(p_client,
                                            SIMPLE_ON_OFF_OPCODE_GET,
                                            NULL,
                                            0);
    if (status == NRF_SUCCESS)
    {
        p_client->state.reliable_transfer_active = true;
    }
    return status;
}

/**
 * Cancel any ongoing reliable message transfer.
 *
 * @param[in] p_client Pointer to the client instance structure.
 */
void simple_on_off_client_pending_msg_cancel(simple_on_off_client_t * p_client)
{
    (void)access_model_reliable_cancel(p_client->model_handle);
}
