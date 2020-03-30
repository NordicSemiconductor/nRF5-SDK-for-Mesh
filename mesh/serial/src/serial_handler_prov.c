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

#include <nrf_error.h>
#include <string.h>

#include "serial_evt.h"
#include "serial_cmd.h"
#include "serial_packet.h"
#include "serial_status.h"
#include "serial_handler_common.h"
#include "nrf_mesh_prov.h"
#include "nrf_mesh_prov_bearer_adv.h"
#include "nrf_mesh_defines.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_assert.h"
#include "log.h"

#include "serial_handler_prov.h"

#ifndef CONFIG_NUM_PROV_CONTEXTS
#define CONFIG_NUM_PROV_CONTEXTS 2
#endif

static bool                       m_scanning_running;
static nrf_mesh_prov_ctx_t        m_prov_contexts[CONFIG_NUM_PROV_CONTEXTS];
static uint8_t                    m_public_key[NRF_MESH_ECDH_PUBLIC_KEY_SIZE];
static uint8_t                    m_private_key[NRF_MESH_ECDH_PRIVATE_KEY_SIZE];
static nrf_mesh_prov_oob_caps_t   m_provisioning_caps;
static nrf_mesh_prov_bearer_adv_t m_pb_adv_contexts[CONFIG_NUM_PROV_CONTEXTS];

static uint8_t find_context_id(const nrf_mesh_prov_ctx_t * p_ctx)
{
    for (int i = 0; i < CONFIG_NUM_PROV_CONTEXTS; ++i)
    {
        if (&m_prov_contexts[i] == p_ctx)
        {
            return i;
        }
    }

    return 0xff;
}

static void serial_handler_prov_evt_in(const nrf_mesh_prov_evt_t * p_evt)
{
    serial_packet_t * p_packet;
    uint32_t status;
    switch (p_evt->type)
    {
        case NRF_MESH_PROV_EVT_UNPROVISIONED_RECEIVED:
            if (m_scanning_running)
            {
                status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_unprov_t), &p_packet);
                if (status == NRF_SUCCESS)
                {
                    p_packet->opcode = SERIAL_OPCODE_EVT_PROV_UNPROVISIONED_RECEIVED;
                    serial_evt_prov_unprov_t * p_unprov = &p_packet->payload.evt.prov.unprov;
                    memcpy(p_unprov->uuid, p_evt->params.unprov.device_uuid, NRF_MESH_UUID_SIZE);
                    p_unprov->rssi = p_evt->params.unprov.p_metadata->params.scanner.rssi;
                    p_unprov->gatt_supported = p_evt->params.unprov.gatt_supported;
                    p_unprov->adv_addr_type = p_evt->params.unprov.p_metadata->params.scanner.adv_addr.addr_type;
                    memcpy(p_unprov->adv_addr, p_evt->params.unprov.p_metadata->params.scanner.adv_addr.addr, BLE_GAP_ADDR_LEN);
                    serial_tx(p_packet);
                }
            }
            break;
        case NRF_MESH_PROV_EVT_LINK_ESTABLISHED:
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_link_established_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_LINK_ESTABLISHED;
                p_packet->payload.evt.prov.link_established.context_id = find_context_id(p_evt->params.link_established.p_context);
                serial_tx(p_packet);
            }
            break;
        case NRF_MESH_PROV_EVT_LINK_CLOSED:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_link_closed_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_LINK_CLOSED;
                serial_evt_prov_link_closed_t * p_link_closed = &p_packet->payload.evt.prov.link_closed;
                p_link_closed->context_id = find_context_id(p_evt->params.link_closed.p_context);
                p_link_closed->close_reason = p_evt->params.link_closed.close_reason;
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_CAPS_RECEIVED:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_caps_received_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_CAPS_RECEIVED;
                serial_evt_prov_caps_received_t * p_caps_received = &p_packet->payload.evt.prov.caps_received;
                p_caps_received->context_id         = find_context_id(p_evt->params.oob_caps_received.p_context);
                p_caps_received->num_elements       = p_evt->params.oob_caps_received.oob_caps.num_elements;
                p_caps_received->public_key_type    = p_evt->params.oob_caps_received.oob_caps.pubkey_type;
                p_caps_received->static_oob_types   = p_evt->params.oob_caps_received.oob_caps.oob_static_types;
                p_caps_received->output_oob_size    = p_evt->params.oob_caps_received.oob_caps.oob_output_size;
                p_caps_received->output_oob_actions = p_evt->params.oob_caps_received.oob_caps.oob_output_actions;
                p_caps_received->input_oob_size     = p_evt->params.oob_caps_received.oob_caps.oob_input_size;
                p_caps_received->input_oob_actions  = p_evt->params.oob_caps_received.oob_caps.oob_input_actions;
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_INVITE_RECEIVED:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_invite_received_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_INVITE_RECEIVED;
                serial_evt_prov_invite_received_t * p_invite_received = &p_packet->payload.evt.prov.invite_received;
                p_invite_received->context_id = find_context_id(p_evt->params.invite_received.p_context);
                p_invite_received->attention_duration_s = p_evt->params.invite_received.attention_duration_s;
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_START_RECEIVED:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_start_received_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_START_RECEIVED;
                serial_evt_prov_start_received_t * p_start_received = &p_packet->payload.evt.prov.start_received;
                p_start_received->context_id = find_context_id(p_evt->params.start_received.p_context);
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_COMPLETE:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_complete_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_COMPLETE;
                serial_evt_prov_complete_t * p_prov_complete = &p_packet->payload.evt.prov.complete;
                p_prov_complete->context_id = find_context_id(p_evt->params.complete.p_context);
                p_prov_complete->iv_index = p_evt->params.complete.p_prov_data->iv_index;
                p_prov_complete->address = p_evt->params.complete.p_prov_data->address;
                p_prov_complete->net_key_index = p_evt->params.complete.p_prov_data->netkey_index;
                p_prov_complete->iv_update_flag = p_evt->params.complete.p_prov_data->flags.iv_update;
                p_prov_complete->key_refresh_flag = p_evt->params.complete.p_prov_data->flags.key_refresh;
                memcpy(p_prov_complete->device_key, p_evt->params.complete.p_devkey, NRF_MESH_KEY_SIZE);
                memcpy(p_prov_complete->net_key, p_evt->params.complete.p_prov_data->netkey, NRF_MESH_KEY_SIZE);
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_STATIC_REQUEST:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_auth_request_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_AUTH_REQUEST;
                serial_evt_prov_auth_request_t * p_auth_req = &p_packet->payload.evt.prov.auth_request;
                p_auth_req->context_id = find_context_id(p_evt->params.static_request.p_context);
                p_auth_req->method = NRF_MESH_PROV_OOB_METHOD_STATIC;
                p_auth_req->action = 0;
                p_auth_req->size = NRF_MESH_KEY_SIZE;
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_ECDH_REQUEST:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_ecdh_request_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_ECDH_REQUEST;
                serial_evt_prov_ecdh_request_t * p_ecdh_req = &p_packet->payload.evt.prov.ecdh_request;
                p_ecdh_req->context_id = find_context_id(p_evt->params.ecdh_request.p_context);
                memcpy(p_ecdh_req->peer_public, p_evt->params.ecdh_request.p_peer_public, NRF_MESH_PROV_PUBKEY_SIZE);
                memcpy(p_ecdh_req->node_private, p_evt->params.ecdh_request.p_node_private, NRF_MESH_PROV_PRIVKEY_SIZE);
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_OUTPUT_REQUEST:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_output_request_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_OUTPUT_REQUEST;
                serial_evt_prov_output_request_t * p_output_req = &p_packet->payload.evt.prov.output_request;
                p_output_req->context_id = find_context_id(p_evt->params.output_request.p_context);
                p_output_req->output_action = p_evt->params.output_request.action;
                memcpy(p_output_req->data, p_evt->params.output_request.p_data,
                       p_evt->params.output_request.size);
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_INPUT_REQUEST:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_auth_request_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_AUTH_REQUEST;
                serial_evt_prov_auth_request_t * p_auth_req = &p_packet->payload.evt.prov.auth_request;
                p_auth_req->context_id = find_context_id(p_evt->params.input_request.p_context);
                p_auth_req->method = NRF_MESH_PROV_OOB_METHOD_INPUT;
                p_auth_req->action = p_evt->params.input_request.action;
                p_auth_req->size = p_evt->params.input_request.size;
                serial_tx(p_packet);
            }
            break;
        }
        case NRF_MESH_PROV_EVT_FAILED:
        {
            status = serial_packet_buffer_get(NRF_MESH_SERIAL_PACKET_OVERHEAD + sizeof(serial_evt_prov_failed_t), &p_packet);
            if (status == NRF_SUCCESS)
            {
                p_packet->opcode = SERIAL_OPCODE_EVT_PROV_FAILED;
                serial_evt_prov_failed_t * p_failed = &p_packet->payload.evt.prov.failed;
                p_failed->context_id = find_context_id(p_evt->params.failed.p_context);
                p_failed->error_code = p_evt->params.failed.failure_code;
                serial_tx(p_packet);
            }
            break;

        }
        default:
            break;
    }
}

uint32_t serial_handler_prov_init(void)
{
    __LOG(LOG_SRC_SERIAL, LOG_LEVEL_INFO, "Generating encryption keypair...\n");
    NRF_MESH_ERROR_CHECK(nrf_mesh_prov_generate_keys(m_public_key, m_private_key));
    for (uint32_t i = 0; i < CONFIG_NUM_PROV_CONTEXTS; ++i)
    {
        NRF_MESH_ERROR_CHECK(nrf_mesh_prov_bearer_add(
                                 &m_prov_contexts[i],
                                 nrf_mesh_prov_bearer_adv_interface_get(&m_pb_adv_contexts[i])));
    }
    return NRF_SUCCESS;
}

void serial_handler_prov_pkt_in(const serial_packet_t * p_incoming)
{
    switch (p_incoming->opcode)
    {
        case SERIAL_OPCODE_CMD_PROV_SCAN_START:
            if (m_scanning_running)
            {
                serial_cmd_rsp_send(p_incoming->opcode, SERIAL_STATUS_ERROR_INVALID_STATE, NULL, 0);
            }
            else
            {
                m_scanning_running = true;
                NRF_MESH_ERROR_CHECK(nrf_mesh_prov_scan_start(serial_handler_prov_evt_in));
                serial_cmd_rsp_send(p_incoming->opcode, SERIAL_STATUS_SUCCESS, NULL, 0);
            }
            break;
        case SERIAL_OPCODE_CMD_PROV_SCAN_STOP:
            if (!m_scanning_running)
            {
                serial_cmd_rsp_send(p_incoming->opcode, SERIAL_STATUS_ERROR_INVALID_STATE, NULL, 0);
            }
            else
            {
                m_scanning_running = false;
                nrf_mesh_prov_scan_stop();
                serial_cmd_rsp_send(p_incoming->opcode, SERIAL_STATUS_SUCCESS, NULL, 0);
            }
            break;
        case SERIAL_OPCODE_CMD_PROV_PROVISION:
        {
            uint32_t status;
            if (p_incoming->payload.cmd.prov.data.context_id < CONFIG_NUM_PROV_CONTEXTS)
            {
                nrf_mesh_prov_provisioning_data_t prov_data = {{0}};
                memcpy(prov_data.netkey, p_incoming->payload.cmd.prov.data.network_key, NRF_MESH_KEY_SIZE);
                memcpy((uint8_t *) &prov_data.netkey_index, (uint8_t *) &p_incoming->payload.cmd.prov.data.network_key_index, sizeof(uint16_t));
                memcpy((uint8_t *) &prov_data.iv_index, (uint8_t *) &p_incoming->payload.cmd.prov.data.iv_index, sizeof(uint32_t));
                memcpy((uint8_t *) &prov_data.address, (uint8_t *) &p_incoming->payload.cmd.prov.data.address, sizeof(uint16_t));
                prov_data.flags.iv_update = p_incoming->payload.cmd.prov.data.iv_update_flag & 0x01;
                prov_data.flags.key_refresh = p_incoming->payload.cmd.prov.data.key_refresh_flag & 0x01;

                status =
                    nrf_mesh_prov_init(&m_prov_contexts[p_incoming->payload.cmd.prov.data.context_id],
                                       m_public_key, m_private_key, &m_provisioning_caps, serial_handler_prov_evt_in);
                if (status == NRF_SUCCESS)
                {
                    status = nrf_mesh_prov_provision(&m_prov_contexts[p_incoming->payload.cmd.prov.data.context_id],
                                                     p_incoming->payload.cmd.prov.data.target_uuid,
                                                     p_incoming->payload.cmd.prov.data.attention_duration_s,
                                                     &prov_data, NRF_MESH_PROV_BEARER_ADV);
                }
            }
            else
            {
                status = NRF_ERROR_INVALID_DATA;
            }

            serial_evt_cmd_rsp_data_prov_ctx_t response = {p_incoming->payload.cmd.prov.data.context_id};
            serial_handler_common_cmd_rsp_nodata_on_error(SERIAL_OPCODE_CMD_PROV_PROVISION, status, (uint8_t *) &response, sizeof(response));
            break;
        }
        case SERIAL_OPCODE_CMD_PROV_LISTEN:
        {
            /* Default to context 0. */
            uint32_t status = nrf_mesh_prov_init(&m_prov_contexts[0], m_public_key, m_private_key, &m_provisioning_caps, serial_handler_prov_evt_in);
            if (status == NRF_SUCCESS)
            {
                status = nrf_mesh_prov_listen(&m_prov_contexts[0], NULL, 0, NRF_MESH_PROV_BEARER_ADV);
            }
            serial_cmd_rsp_send(p_incoming->opcode, serial_translate_error(status), NULL, 0);
            break;
        }
        case SERIAL_OPCODE_CMD_PROV_OOB_USE:
        {

            uint32_t status;
            if (p_incoming->payload.cmd.prov.oob_use.context_id < CONFIG_NUM_PROV_CONTEXTS)
            {
                status = nrf_mesh_prov_oob_use(&m_prov_contexts[p_incoming->payload.cmd.prov.oob_use.context_id],
                                               (nrf_mesh_prov_oob_method_t) p_incoming->payload.cmd.prov.oob_use.oob_method,
                                               p_incoming->payload.cmd.prov.oob_use.oob_action,
                                               p_incoming->payload.cmd.prov.oob_use.size);
            }
            else
            {
                status = NRF_ERROR_INVALID_DATA;
            }
            serial_evt_cmd_rsp_data_prov_ctx_t response = {p_incoming->payload.cmd.prov.oob_use.context_id};
            serial_handler_common_cmd_rsp_nodata_on_error(SERIAL_OPCODE_CMD_PROV_OOB_USE, status, (uint8_t *) &response, sizeof(response));
            break;
        }
        case SERIAL_OPCODE_CMD_PROV_AUTH_DATA:
        {
            uint32_t status;
            if (p_incoming->payload.cmd.prov.oob_use.context_id < CONFIG_NUM_PROV_CONTEXTS)
            {
                status = nrf_mesh_prov_auth_data_provide(&m_prov_contexts[p_incoming->payload.cmd.prov.auth_data.context_id],
                                                         p_incoming->payload.cmd.prov.auth_data.data,
                                                         NRF_MESH_KEY_SIZE - ((sizeof(serial_cmd_prov_auth_data_t) + SERIAL_PACKET_LENGTH_OVERHEAD) - p_incoming->length));
            }
            else
            {
                status = NRF_ERROR_INVALID_DATA;
            }
            serial_evt_cmd_rsp_data_prov_ctx_t response = {p_incoming->payload.cmd.prov.auth_data.context_id};
            serial_handler_common_cmd_rsp_nodata_on_error(SERIAL_OPCODE_CMD_PROV_AUTH_DATA, status, (uint8_t *) &response, sizeof(response));
            break;
        }
        case SERIAL_OPCODE_CMD_PROV_ECDH_SECRET:
        {
            uint32_t status;
            if (p_incoming->payload.cmd.prov.oob_use.context_id < CONFIG_NUM_PROV_CONTEXTS)
            {
                status = nrf_mesh_prov_shared_secret_provide(&m_prov_contexts[p_incoming->payload.cmd.prov.ecdh_data.context_id],
                                                             p_incoming->payload.cmd.prov.ecdh_data.shared_secret);
            }
            else
            {
                status = NRF_ERROR_INVALID_DATA;
            }
            serial_evt_cmd_rsp_data_prov_ctx_t response = {p_incoming->payload.cmd.prov.ecdh_data.context_id};
            serial_handler_common_cmd_rsp_nodata_on_error(SERIAL_OPCODE_CMD_PROV_ECDH_SECRET, status, (uint8_t *) &response, sizeof(response));
            break;
        }
        case SERIAL_OPCODE_CMD_PROV_KEYPAIR_SET:
        {
            memcpy(m_private_key, p_incoming->payload.cmd.prov.keypair.private_key, NRF_MESH_ECDH_PRIVATE_KEY_SIZE);
            memcpy(m_public_key,  p_incoming->payload.cmd.prov.keypair.public_key,  NRF_MESH_ECDH_PUBLIC_KEY_SIZE);
            serial_cmd_rsp_send(p_incoming->opcode, SERIAL_STATUS_SUCCESS, NULL, 0);
            break;
        }
        case SERIAL_OPCODE_CMD_PROV_CAPABILITIES_SET:
        {
            m_provisioning_caps.num_elements       = p_incoming->payload.cmd.prov.caps.num_elements;
            m_provisioning_caps.algorithms         = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
            m_provisioning_caps.pubkey_type        = p_incoming->payload.cmd.prov.caps.public_key_type;
            m_provisioning_caps.oob_static_types   = p_incoming->payload.cmd.prov.caps.static_oob_types;
            m_provisioning_caps.oob_input_size     = p_incoming->payload.cmd.prov.caps.input_oob_size;
            m_provisioning_caps.oob_input_actions  = p_incoming->payload.cmd.prov.caps.input_oob_actions;
            m_provisioning_caps.oob_output_size    = p_incoming->payload.cmd.prov.caps.output_oob_size;
            m_provisioning_caps.oob_output_actions = p_incoming->payload.cmd.prov.caps.output_oob_actions;
            serial_cmd_rsp_send(p_incoming->opcode, SERIAL_STATUS_SUCCESS, NULL, 0);
            break;
        }
        default:
            break;
    }
}

uint32_t serial_handler_prov_context_get(uint8_t index, nrf_mesh_prov_ctx_t ** pp_prov_ctx)
{
    if (index >= CONFIG_NUM_PROV_CONTEXTS)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    *pp_prov_ctx = &m_prov_contexts[index];
    return NRF_SUCCESS;
}

uint32_t serial_handler_prov_keys_get(const uint8_t ** pp_public_key, const uint8_t ** pp_private_key)
{
    *pp_public_key = &m_public_key[0];
    *pp_private_key = &m_private_key[0];
    return NRF_SUCCESS;
}


uint32_t serial_handler_prov_oob_caps_get(const nrf_mesh_prov_oob_caps_t ** pp_caps)
{
    *pp_caps = &m_provisioning_caps;
    return NRF_SUCCESS;
}
