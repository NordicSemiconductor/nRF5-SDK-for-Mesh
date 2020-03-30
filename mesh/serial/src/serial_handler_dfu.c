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

#include "serial.h"
#include "serial_handler_dfu.h"
#include "serial_status.h"

#include "nrf_mesh.h"
#include "nrf_mesh_events.h"
#include "nrf_mesh_dfu.h"

static nrf_mesh_evt_handler_t m_evt_handler;

static void serial_handler_mesh_evt_handle(const nrf_mesh_evt_t* p_evt)
{
    serial_packet_t * p_serial_evt;
    uint32_t status;
    switch (p_evt->type)
    {
        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_firmware_outdated_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_DFU_FIRMWARE_OUTDATED;
                p_serial_evt->payload.evt.dfu.firmware_outdated.dfu_type = p_evt->params.dfu.fw_outdated.transfer.dfu_type;
                p_serial_evt->payload.evt.dfu.firmware_outdated.available_fwid     = p_evt->params.dfu.fw_outdated.transfer.id;
                p_serial_evt->payload.evt.dfu.firmware_outdated.current_fwid  = p_evt->params.dfu.fw_outdated.current;
                serial_tx(p_serial_evt);
            }
            break;

        case NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_firmware_outdated_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH;
                p_serial_evt->payload.evt.dfu.firmware_outdated.dfu_type = p_evt->params.dfu.fw_outdated.transfer.dfu_type;
                p_serial_evt->payload.evt.dfu.firmware_outdated.available_fwid     = p_evt->params.dfu.fw_outdated.transfer.id;
                p_serial_evt->payload.evt.dfu.firmware_outdated.current_fwid  = p_evt->params.dfu.fw_outdated.current;
                serial_tx(p_serial_evt);
            }
            break;

        case NRF_MESH_EVT_DFU_REQ_RELAY:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_req_relay_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_DFU_REQ_RELAY;
                p_serial_evt->payload.evt.dfu.req_relay.dfu_type  = p_evt->params.dfu.req_relay.transfer.dfu_type;
                p_serial_evt->payload.evt.dfu.req_relay.fwid      = p_evt->params.dfu.req_relay.transfer.id;
                p_serial_evt->payload.evt.dfu.req_relay.authority = p_evt->params.dfu.req_relay.authority;
                serial_tx(p_serial_evt);
            }
            break;

        case NRF_MESH_EVT_DFU_REQ_SOURCE:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_req_source_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_DFU_REQ_SOURCE;
                p_serial_evt->payload.evt.dfu.req_source.dfu_type = p_evt->params.dfu.req_source.dfu_type;
                serial_tx(p_serial_evt);
            }
            break;

        case NRF_MESH_EVT_DFU_START:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_start_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_DFU_START;
                p_serial_evt->payload.evt.dfu.start.role     = p_evt->params.dfu.start.role;
                p_serial_evt->payload.evt.dfu.start.dfu_type = p_evt->params.dfu.start.transfer.dfu_type;
                p_serial_evt->payload.evt.dfu.start.fwid     = p_evt->params.dfu.start.transfer.id;
                serial_tx(p_serial_evt);
            }
            break;

        case NRF_MESH_EVT_DFU_END:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_end_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_DFU_END;
                p_serial_evt->payload.evt.dfu.end.role       = p_evt->params.dfu.end.role;
                p_serial_evt->payload.evt.dfu.end.dfu_type   = p_evt->params.dfu.end.transfer.dfu_type;
                p_serial_evt->payload.evt.dfu.end.fwid       = p_evt->params.dfu.end.transfer.id;
                p_serial_evt->payload.evt.dfu.end.end_reason = p_evt->params.dfu.end.end_reason;
                serial_tx(p_serial_evt);
            }
            break;

        case NRF_MESH_EVT_DFU_BANK_AVAILABLE:
            status = serial_packet_buffer_get(SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_bank_t), &p_serial_evt);
            if (status == NRF_SUCCESS)
            {
                p_serial_evt->opcode = SERIAL_OPCODE_EVT_DFU_BANK_AVAILABLE;
                p_serial_evt->payload.evt.dfu.bank.dfu_type   = p_evt->params.dfu.bank.transfer.dfu_type;
                p_serial_evt->payload.evt.dfu.bank.fwid       = p_evt->params.dfu.bank.transfer.id;
                p_serial_evt->payload.evt.dfu.bank.start_addr = (uint32_t) p_evt->params.dfu.bank.p_start_addr;
                p_serial_evt->payload.evt.dfu.bank.length     = p_evt->params.dfu.bank.length;
                p_serial_evt->payload.evt.dfu.bank.is_signed  = p_evt->params.dfu.bank.is_signed;
                serial_tx(p_serial_evt);
            }
            break;

        default:
            break;
    }
}

void serial_handler_dfu_init(void)
{
    m_evt_handler.evt_cb = serial_handler_mesh_evt_handle;
    nrf_mesh_evt_handler_add(&m_evt_handler);
}

void serial_handler_dfu_rx(const serial_packet_t * p_cmd)
{
    static const uint8_t lengths[] =
    {
        SERIAL_PACKET_LENGTH_OVERHEAD, /* JUMP_TO_BOOTLOADER */
        SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_request_t), /* REQUEST */
        SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_relay_t), /* RELAY */
        SERIAL_PACKET_LENGTH_OVERHEAD, /* ABORT */
        SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_bank_info_get_t), /* BANK_INFO_GET */
        SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_bank_flash_t), /* BANK_FLASH */
        SERIAL_PACKET_LENGTH_OVERHEAD, /* STATE_GET */
    };

    if ((uint32_t)(p_cmd->opcode - SERIAL_OPCODE_CMD_RANGE_DFU_START) < sizeof(lengths) &&
        p_cmd->length != lengths[p_cmd->opcode - SERIAL_OPCODE_CMD_RANGE_DFU_START])
    {
        serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
    }
    else
    {
        switch (p_cmd->opcode)
        {
            case SERIAL_OPCODE_CMD_DFU_JUMP_TO_BOOTLOADER:
                {
                    uint32_t error_code = nrf_mesh_dfu_jump_to_bootloader();
                    /* Shouldn't return if everything worked as intended. */
                    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(error_code), NULL, 0);
                    break;
                }
            case SERIAL_OPCODE_CMD_DFU_REQUEST:
                {
                    uint32_t error_code = nrf_mesh_dfu_request(
                            (nrf_mesh_dfu_type_t) p_cmd->payload.cmd.dfu.request.dfu_type,
                            &p_cmd->payload.cmd.dfu.request.fwid,
                            (uint32_t*) p_cmd->payload.cmd.dfu.request.bank_addr);
                    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(error_code), NULL, 0);
                    break;
                }
            case SERIAL_OPCODE_CMD_DFU_RELAY:
                {
                    uint32_t error_code = nrf_mesh_dfu_relay(
                            (nrf_mesh_dfu_type_t) p_cmd->payload.cmd.dfu.relay.dfu_type,
                            &p_cmd->payload.cmd.dfu.relay.fwid);
                    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(error_code), NULL, 0);
                    break;
                }
            case SERIAL_OPCODE_CMD_DFU_ABORT:
                {
                    uint32_t error_code = nrf_mesh_dfu_abort();
                    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(error_code), NULL, 0);
                    break;
                }
            case SERIAL_OPCODE_CMD_DFU_BANK_INFO_GET:
                {
                    nrf_mesh_dfu_bank_info_t bank_info;
                    uint32_t error_code = nrf_mesh_dfu_bank_info_get(
                            (nrf_mesh_dfu_type_t) p_cmd->payload.cmd.dfu.relay.dfu_type,
                            &bank_info);
                    if (error_code == NRF_SUCCESS)
                    {
                        serial_evt_cmd_rsp_data_dfu_bank_info_t rsp;
                        rsp.dfu_type   = (uint8_t) bank_info.dfu_type;
                        rsp.fwid       = bank_info.fwid;
                        rsp.is_signed  = bank_info.is_signed;
                        rsp.start_addr = (uint32_t) bank_info.p_start_addr;
                        rsp.length     = bank_info.length;
                        serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
                    }
                    else
                    {
                        serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(error_code), NULL, 0);
                    }
                    break;
                }

            case SERIAL_OPCODE_CMD_DFU_BANK_FLASH:
                if (p_cmd->length != SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_bank_flash_t))
                {
                    serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
                }
                else
                {
                    uint32_t error_code = nrf_mesh_dfu_bank_flash((nrf_mesh_dfu_type_t) p_cmd->payload.cmd.dfu.bank_flash.dfu_type);
                    serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(error_code), NULL, 0);
                }
                break;

            case SERIAL_OPCODE_CMD_DFU_STATE_GET:
                if (p_cmd->length != SERIAL_PACKET_LENGTH_OVERHEAD)
                {
                    serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
                }
                else
                {
                    nrf_mesh_dfu_transfer_state_t state;
                    memset(&state.fwid, 0, sizeof(nrf_mesh_fwid_t)); /* Set any padding in the firmware union to 0 */
                    uint32_t error_code = nrf_mesh_dfu_state_get(&state);
                    if (error_code == NRF_SUCCESS)
                    {
                        serial_evt_cmd_rsp_data_dfu_state_t rsp;
                        rsp.role          = state.role;
                        rsp.type          = state.type;
                        rsp.fwid          = state.fwid;
                        rsp.state         = state.state;
                        rsp.data_progress = state.data_progress;
                        serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
                    }
                    else
                    {
                        serial_cmd_rsp_send(p_cmd->opcode, serial_translate_error(error_code), NULL, 0);
                    }
                }
                break;

            default:
                NRF_MESH_ASSERT(p_cmd->opcode >= SERIAL_OPCODE_CMD_RANGE_DFU_START &&
                    p_cmd->opcode <= SERIAL_OPCODE_CMD_RANGE_DFU_END);
                serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
        }
    }
}


