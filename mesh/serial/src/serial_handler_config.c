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

#include "serial_handler_config.h"

#include "nrf_mesh_configure.h"

#include "serial.h"
#include "serial_status.h"
#include "advertiser.h"

void serial_handler_config_rx(const serial_packet_t* p_cmd)
{
    switch (p_cmd->opcode)
    {
        case SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_SET:
        case SERIAL_OPCODE_CMD_CONFIG_CHANNEL_MAP_SET:
        case SERIAL_OPCODE_CMD_CONFIG_CHANNEL_MAP_GET:
        case SERIAL_OPCODE_CMD_CONFIG_TX_POWER_SET:
        case SERIAL_OPCODE_CMD_CONFIG_TX_POWER_GET:
            serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
            break;

        case SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_GET:
        {
            if (p_cmd->length != SERIAL_PACKET_LENGTH_OVERHEAD)
            {
                serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
            }
            else
            {
                ble_gap_addr_t addr;
                advertiser_address_default_get(&addr);
                serial_evt_cmd_rsp_data_adv_addr_t rsp;
                rsp.addr_type = addr.addr_type;
                memcpy(rsp.addr, addr.addr, BLE_GAP_ADDR_LEN);
                serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
            }
            break;
        }

        case SERIAL_OPCODE_CMD_CONFIG_UUID_SET:
            if (p_cmd->length != SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_config_uuid_t))
            {
                serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
            }
            else
            {
                nrf_mesh_configure_device_uuid_set(p_cmd->payload.cmd.config.uuid.uuid);
                serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, NULL, 0);
            }
            break;

        case SERIAL_OPCODE_CMD_CONFIG_UUID_GET:
            if (p_cmd->length != SERIAL_PACKET_LENGTH_OVERHEAD)
            {
                serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
            }
            else
            {
                serial_evt_cmd_rsp_data_device_uuid_t rsp;
                memcpy(rsp.device_uuid, nrf_mesh_configure_device_uuid_get(), NRF_MESH_UUID_SIZE);
                serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_SUCCESS, (uint8_t *) &rsp, sizeof(rsp));
            }
            break;

        default:
            NRF_MESH_ASSERT(p_cmd->opcode >= SERIAL_OPCODE_CMD_RANGE_CONFIG_START &&
                p_cmd->opcode <= SERIAL_OPCODE_CMD_RANGE_CONFIG_END);
            serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
            break;
    }
}

