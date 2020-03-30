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

#include "simple_on_off_server.h"
#include "simple_on_off_common.h"

#include <stdint.h>
#include <stddef.h>

#include "access.h"
#include "nrf_mesh_assert.h"
#include "log.h"

/*****************************************************************************
 * Static functions
 *****************************************************************************/

static void reply_status(const simple_on_off_server_t * p_server,
                         const access_message_rx_t * p_message,
                         bool present_on_off)
{
    simple_on_off_msg_status_t status;
    status.present_on_off = present_on_off ? 1 : 0;
    access_message_tx_t reply;
    reply.opcode.opcode = SIMPLE_ON_OFF_OPCODE_STATUS;
    reply.opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    reply.p_buffer = (const uint8_t *) &status;
    reply.length = sizeof(status);
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    reply.access_token = nrf_mesh_unique_token_get();

    (void) access_model_reply(p_server->model_handle, p_message, &reply);
}

/*****************************************************************************
 * Opcode handler callbacks
 *****************************************************************************/

static void handle_set_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->set_cb != NULL);

    bool value = (((simple_on_off_msg_set_t*) p_message->p_data)->on_off) > 0;
    value = p_server->set_cb(p_server, value);
    reply_status(p_server, p_message, value);
    (void) simple_on_off_server_status_publish(p_server, value); /* We don't care about status */
}

static void handle_get_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->get_cb != NULL);
    reply_status(p_server, p_message, p_server->get_cb(p_server));
}

static void handle_set_unreliable_cb(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    simple_on_off_server_t * p_server = p_args;
    NRF_MESH_ASSERT(p_server->set_cb != NULL);
    bool value = (((simple_on_off_msg_set_unreliable_t*) p_message->p_data)->on_off) > 0;
    value = p_server->set_cb(p_server, value);
    (void) simple_on_off_server_status_publish(p_server, value);
}

static const access_opcode_handler_t m_opcode_handlers[] =
{
    {ACCESS_OPCODE_VENDOR(SIMPLE_ON_OFF_OPCODE_SET,            SIMPLE_ON_OFF_COMPANY_ID), handle_set_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_ON_OFF_OPCODE_GET,            SIMPLE_ON_OFF_COMPANY_ID), handle_get_cb},
    {ACCESS_OPCODE_VENDOR(SIMPLE_ON_OFF_OPCODE_SET_UNRELIABLE, SIMPLE_ON_OFF_COMPANY_ID), handle_set_unreliable_cb}
};

static void handle_publish_timeout(access_model_handle_t handle, void * p_args)
{
    simple_on_off_server_t * p_server = p_args;

    (void) simple_on_off_server_status_publish(p_server, p_server->get_cb(p_server));
}

/*****************************************************************************
 * Public API
 *****************************************************************************/

uint32_t simple_on_off_server_init(simple_on_off_server_t * p_server, uint16_t element_index)
{
    if (p_server == NULL ||
        p_server->get_cb == NULL ||
        p_server->set_cb == NULL)
    {
        return NRF_ERROR_NULL;
    }

    access_model_add_params_t init_params;
    init_params.element_index =  element_index;
    init_params.model_id.model_id = SIMPLE_ON_OFF_SERVER_MODEL_ID;
    init_params.model_id.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    init_params.p_opcode_handlers = &m_opcode_handlers[0];
    init_params.opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]);
    init_params.p_args = p_server;
    init_params.publish_timeout_cb = handle_publish_timeout;
    return access_model_add(&init_params, &p_server->model_handle);
}

uint32_t simple_on_off_server_status_publish(simple_on_off_server_t * p_server, bool value)
{
    simple_on_off_msg_status_t status;
    status.present_on_off = value ? 1 : 0;
    access_message_tx_t msg;
    msg.opcode.opcode = SIMPLE_ON_OFF_OPCODE_STATUS;
    msg.opcode.company_id = SIMPLE_ON_OFF_COMPANY_ID;
    msg.p_buffer = (const uint8_t *) &status;
    msg.length = sizeof(status);
    msg.force_segmented = false;
    msg.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;
    msg.access_token = nrf_mesh_unique_token_get();
    return access_model_publish(p_server->model_handle, &msg);
}
