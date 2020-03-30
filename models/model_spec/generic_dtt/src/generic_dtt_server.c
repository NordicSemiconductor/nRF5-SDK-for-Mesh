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

#include "generic_dtt_server.h"
#include "generic_dtt_common.h"
#include "generic_dtt_messages.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "access.h"
#include "access_config.h"
#include "nrf_mesh_assert.h"
#include "nrf_mesh_utils.h"
#include "nordic_common.h"

static uint32_t status_send(generic_dtt_server_t * p_server,
                            const access_message_rx_t * p_message,
                            const generic_dtt_status_params_t * p_params)
{
    generic_dtt_status_msg_pkt_t msg_pkt;

    msg_pkt.transition_time = model_transition_time_encode(p_params->transition_time_ms);

    access_message_tx_t reply =
    {
        .opcode = ACCESS_OPCODE_SIG(GENERIC_DTT_OPCODE_STATUS),
        .p_buffer = (const uint8_t *) &msg_pkt,
        .length = sizeof(generic_dtt_status_msg_pkt_t),
        .force_segmented = p_server->settings.force_segmented,
        .transmic_size = p_server->settings.transmic_size
    };

    if (p_message == NULL)
    {
        return access_model_publish(p_server->model_handle, &reply);
    }
    else
    {
        return access_model_reply(p_server->model_handle, p_message, &reply);
    }
}

static void periodic_publish_cb(access_model_handle_t handle, void * p_args)
{
    generic_dtt_server_t * p_server = (generic_dtt_server_t *)p_args;
    generic_dtt_status_params_t out_data = {0};

    p_server->settings.p_callbacks->dtt_cbs.get_cb(p_server, NULL, &out_data);
    (void) status_send(p_server, NULL, &out_data);
}

/** Opcode Handlers */

static void handle_set(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_dtt_server_t * p_server = (generic_dtt_server_t *) p_args;
    generic_dtt_set_params_t in_data = {0};
    generic_dtt_status_params_t out_data = {0};

    if (p_rx_msg->length == sizeof(generic_dtt_set_msg_pkt_t))
    {
        generic_dtt_set_msg_pkt_t * p_msg_params_packed = (generic_dtt_set_msg_pkt_t *) p_rx_msg->p_data;
        if (!model_transition_time_is_valid(p_msg_params_packed->transition_time))
        {
            return;
        }

        in_data.transition_time_ms = model_transition_time_decode(p_msg_params_packed->transition_time);

        p_server->settings.p_callbacks->dtt_cbs.set_cb(p_server, &p_rx_msg->meta_data, &in_data,
                                  (p_rx_msg->opcode.opcode == GENERIC_DTT_OPCODE_SET) ? &out_data : NULL);

        if (p_rx_msg->opcode.opcode == GENERIC_DTT_OPCODE_SET)
        {
            (void) status_send(p_server, p_rx_msg, &out_data);
        }
    }
}

static void handle_get(access_model_handle_t model_handle, const access_message_rx_t * p_rx_msg, void * p_args)
{
    generic_dtt_server_t * p_server = (generic_dtt_server_t *) p_args;
    generic_dtt_status_params_t out_data = {0};

    if (p_rx_msg->length == 0)
    {
        p_server->settings.p_callbacks->dtt_cbs.get_cb(p_server, &p_rx_msg->meta_data, &out_data);
        (void) status_send(p_server, p_rx_msg, &out_data);
    }
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_SIG(GENERIC_DTT_OPCODE_SET), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_DTT_OPCODE_SET_UNACKNOWLEDGED), handle_set},
    {ACCESS_OPCODE_SIG(GENERIC_DTT_OPCODE_GET), handle_get},
};

/** Interface functions */

uint32_t generic_dtt_server_init(generic_dtt_server_t * p_server, uint8_t element_index)
{
    if (p_server == NULL ||
        p_server->settings.p_callbacks == NULL ||
        p_server->settings.p_callbacks->dtt_cbs.set_cb == NULL ||
        p_server->settings.p_callbacks->dtt_cbs.get_cb == NULL )
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params =
    {
        .model_id = ACCESS_MODEL_SIG(GENERIC_DTT_SERVER_MODEL_ID),
        .element_index =  element_index,
        .p_opcode_handlers = &m_opcode_handlers[0],
        .opcode_count = ARRAY_SIZE(m_opcode_handlers),
        .p_args = p_server,
        .publish_timeout_cb = periodic_publish_cb
    };

    uint32_t status = access_model_add(&init_params, &p_server->model_handle);

    if (status == NRF_SUCCESS)
    {
        status = access_model_subscription_list_alloc(p_server->model_handle);
    }

    return status;
}

uint32_t generic_dtt_server_status_publish(generic_dtt_server_t * p_server, const generic_dtt_status_params_t * p_params)
{
    if (p_server == NULL ||
        p_params == NULL)
    {
        return NRF_ERROR_NULL;
    }

    return status_send(p_server, NULL, p_params);
}

