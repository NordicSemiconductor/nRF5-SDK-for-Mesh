/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#include "serial_pb_remote_client.h"
#include "pb_remote_client.h"

#include <stdint.h>
#include <stddef.h>

#include "nrf_error.h"

#include "nrf_mesh_serial.h"
#include "serial.h"
#include "serial_handler_models.h"
#include "serial_cmd.h"

#include "nrf_mesh_assert.h"
#include "access_config.h"
#include "access.h"

/*****************************************************************************
 * Definitions
 *****************************************************************************/
/** Number of pb remote client instances: this will be relevant when we introduce concurrent clients:
  * As of 05.2017, pb_remote is capable of only 1 client instance.
  * @todo If this becomes useful, then it should be configurable by the application. */
#ifndef SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT
#define SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT 1
#endif


/*****************************************************************************
 * Static asserts: make sure that our assumptions on packet sizes are true
 *****************************************************************************/
NRF_MESH_STATIC_ASSERT(sizeof(serial_pbr_client_command_data_t) <= sizeof(serial_cmd_model_specific_command_t) - sizeof(serial_cmd_model_specific_command_header_t));

/*****************************************************************************
 * Static globals
 *****************************************************************************/
static pb_remote_client_t m_remote_client[SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT];
static serial_cmd_prov_keypair_t * mp_keypair;
static nrf_mesh_prov_ctx_t * mp_prov_ctxs;

/*****************************************************************************
 * Local helper functions
 *****************************************************************************/
static uint32_t get_pbr_instance_index(access_model_handle_t model_handle)
{
    uint32_t pbr_index;
    for (pbr_index = 0; pbr_index < SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT; ++pbr_index)
    {
        if (m_remote_client[pbr_index].state != PB_REMOTE_CLIENT_STATE_NONE && m_remote_client[pbr_index].model_handle == model_handle)
        {
            break;
        }
    }
    return pbr_index;
}

/*****************************************************************************
 * Event Handlers
 *****************************************************************************/
static void populate_model_specific_serial_header(serial_packet_t * p_serial_evt, uint8_t event_type)
{
    p_serial_evt->opcode = SERIAL_OPCODE_EVT_MODEL_SPECIFIC;

    serial_evt_model_specific_t * p_model_evt = &p_serial_evt->payload.evt.model;
    p_model_evt->model_evt_info.model_id.model_id = PB_REMOTE_CLIENT_MODEL_ID;
    p_model_evt->model_evt_info.model_id.company_id = ACCESS_COMPANY_ID_NONE;
    p_model_evt->model_evt_info.evt_type = event_type;
}

static void remote_client_event_cb(const pb_remote_event_t * p_evt)
{
    serial_packet_t * p_serial_evt;
    if (p_evt->type == PB_REMOTE_EVENT_REMOTE_UUID)
    {
        uint16_t packet_len = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_model_specific_header_t) + sizeof(serial_pbr_client_remote_uuid_event_t);
        NRF_MESH_ASSERT(NRF_SUCCESS == serial_packet_buffer_get( packet_len, &p_serial_evt));
        serial_pbr_client_remote_uuid_event_t * p_uuid_event = (serial_pbr_client_remote_uuid_event_t *) p_serial_evt->payload.evt.model.data;
        p_uuid_event->device_id = p_evt->remote_uuid.device_id;
        memcpy(p_uuid_event->uuid, p_evt->remote_uuid.p_uuid, NRF_MESH_UUID_SIZE);
    }
    else
    {
        NRF_MESH_ASSERT(NRF_SUCCESS == serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_model_specific_header_t), &p_serial_evt));
    }

    populate_model_specific_serial_header(p_serial_evt, p_evt->type);
    serial_tx(p_serial_evt);
}

/*****************************************************************************
 * Model specific serial handler callbacks given to serial_handler_models.c
 *****************************************************************************/
static uint32_t serial_pbr_client_init(const serial_cmd_model_specific_init_t * p_init_params, access_model_handle_t * p_model_handle)
{
    uint32_t pbr_index;
    for (pbr_index = 0; pbr_index < SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT; ++pbr_index)
    {
        if (m_remote_client[pbr_index].state == PB_REMOTE_CLIENT_STATE_NONE)
        {
            break;
        }
    }
    if (pbr_index == SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT)
    {
        return NRF_ERROR_NO_MEM;
    }
    uint32_t status = pb_remote_client_init(&m_remote_client[pbr_index], p_init_params->model_init_info.element_index, &mp_prov_ctxs[pbr_index], remote_client_event_cb);

    /* The user needs to execute:
     * access_model_application_bind(m_remote_client[pbr_index].model_handle, m_appkey_handles[0]);
     * access_model_publish_application_set(m_remote_client[pbr_index].model_handle, m_appkey_handles[0]);
     * access_model_publish_ttl_set(m_remote_client[pbr_index].model_handle, ttl); // ttl should be > 1
     */
    if (status == NRF_SUCCESS)
    {
        *p_model_handle = m_remote_client[pbr_index].model_handle;
    }
    return status;
}

static uint32_t serial_pbr_client_command(const  serial_cmd_model_specific_command_t * p_command_params, serial_evt_cmd_rsp_data_model_cmd_t * p_cmd_rsp)
{
    uint32_t pbr_index = get_pbr_instance_index(p_command_params->model_cmd_info.model_handle);
    p_cmd_rsp->data_len = 0;
    if (pbr_index >= SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    const serial_pbr_client_command_data_t * p_pbr_command = (serial_pbr_client_command_data_t *)&p_command_params->data[0];
    switch (p_pbr_command->cmd_type)
    {
        case SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_START:
            return pb_remote_client_remote_scan_start(&m_remote_client[pbr_index]);
        case SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_CANCEL:
            return pb_remote_client_remote_scan_cancel(&m_remote_client[pbr_index]);
        case SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_PROVISION:
        {
            return pb_remote_client_remote_provision(&m_remote_client[pbr_index],
                                                          &p_pbr_command->provisioning_data.prov_data,
                                                          mp_keypair->public_key,
                                                          mp_keypair->private_key,
                                                          &p_pbr_command->provisioning_data.capabilities,
                                                          p_pbr_command->provisioning_data.unprov_id);
        }
        default:
            return NRF_ERROR_NOT_SUPPORTED;
    }

}

/* Call this on SERIAL_OPCODE_CMD_PROV_KEYPAIR_SET */
// void serial_pb_remote_client_keypair_set(serial_cmd_prov_keypair_t * p_keypair)
// {
//     mp_keypair = p_keypair;
// }

/*****************************************************************************
 * Public functions
 *****************************************************************************/
void serial_pb_remote_client_init(nrf_mesh_prov_ctx_t * p_prov_ctxs, uint8_t prov_ctx_array_size, serial_cmd_prov_keypair_t * p_keypair)
{
    /* Number of available provisioning contexts need to be equal to or greater than the number of remote client instances*/
    NRF_MESH_ASSERT(prov_ctx_array_size >= SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT);
    NRF_MESH_ASSERT(p_prov_ctxs != NULL);
    NRF_MESH_ASSERT(p_keypair != NULL);

    mp_prov_ctxs = p_prov_ctxs;
    mp_keypair = p_keypair;

    serial_handler_models_info_t model_info =
    {
        .model_id = {.model_id = PB_REMOTE_CLIENT_MODEL_ID, .company_id = ACCESS_COMPANY_ID_NONE},
        .model_initialize = serial_pbr_client_init,
        .model_command = serial_pbr_client_command
    };
    NRF_MESH_ASSERT(serial_handler_models_register(&model_info) == NRF_SUCCESS);
}

#ifdef UNIT_TEST
void serial_pb_remote_client_reset(void)
{
    mp_prov_ctxs = NULL;
    mp_keypair = NULL;
    memset(m_remote_client, 0, sizeof(m_remote_client));
}
#endif
