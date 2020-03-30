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
#include "rssi_client.h" 
#include "rssi_common.h"

/********** Incoming message handlers **********/

/* Function used to send a acknowledgement messages to confirm that the rssi data has arrived */
static uint32_t reply_status(const rssi_client_t * p_client, const access_message_rx_t * p_message)
{
    access_message_tx_t reply;
    reply.opcode.opcode = RSSI_OPCODE_RSSI_ACK;
    reply.opcode.company_id = ACCESS_COMPANY_ID_NORDIC;
    reply.p_buffer = NULL;
    reply.length = 0;
    reply.force_segmented = false;
    reply.transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT;

    return access_model_reply(p_client->model_handle, p_message, &reply);
}

/* Handles incoming rssi data from the servers */
static void handle_rssi_data(access_model_handle_t handle, const access_message_rx_t * p_message, void * p_args)
{
    const rssi_client_t * p_client = p_args;

    p_client->rssi_handler(p_message);

    (void)reply_status(p_client, p_message);
} 

static const access_opcode_handler_t m_opcode_handlers[] =
{
    { ACCESS_OPCODE_VENDOR(RSSI_OPCODE_SEND_RSSI_DATA, ACCESS_COMPANY_ID_NORDIC), handle_rssi_data },
};


/********** Interface functions **********/

uint32_t rssi_client_init(rssi_client_t * p_client, uint16_t element_index, rssi_evt_cb_t rssi_handler)
{
    if ((rssi_handler == NULL) || (p_client == NULL))
    {
        return NRF_ERROR_NULL;
    }

    p_client->rssi_handler = rssi_handler;
    access_model_add_params_t add_params =
    {
        .element_index = element_index,
        .model_id.model_id = RSSI_CLIENT_MODEL_ID, /*lint !e64 Type Mismatch */
        .model_id.company_id = ACCESS_COMPANY_ID_NORDIC,
        .p_opcode_handlers = m_opcode_handlers,
        .opcode_count = sizeof(m_opcode_handlers) / sizeof(m_opcode_handlers[0]),
        .p_args = p_client
    };
    return access_model_add(&add_params, &p_client->model_handle);
}
