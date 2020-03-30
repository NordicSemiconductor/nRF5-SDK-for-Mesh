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

#include "serial_handler_common.h"
#include <stdint.h>
#include "serial.h"
#include "serial_status.h"


void serial_handler_common_cmd_rsp_nodata_on_error(uint8_t opcode, uint32_t status, const uint8_t * p_data, uint16_t length)
{
    if (NRF_SUCCESS == status)
    {
        (void) serial_cmd_rsp_send(opcode, serial_translate_error(status), p_data, length);
    }
    else
    {
        (void) serial_cmd_rsp_send(opcode, serial_translate_error(status), NULL, 0);
    }
}

void serial_handler_common_rx(const serial_packet_t* p_cmd, const serial_handler_common_opcode_to_fp_map_t * p_cmd_handlers, uint32_t no_handlers)
{
    for (uint32_t i = 0; i < no_handlers; i++)
    {
        if (p_cmd->opcode == p_cmd_handlers[i].opcode)
        {
            if (p_cmd->length < SERIAL_PACKET_LENGTH_OVERHEAD + p_cmd_handlers[i].payload_minlen ||
                p_cmd->length > SERIAL_PACKET_LENGTH_OVERHEAD + p_cmd_handlers[i].payload_minlen + p_cmd_handlers[i].payload_optional_extra_bytes)
            {
                (void) serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);
            }
            else
            {
                p_cmd_handlers[i].callback(p_cmd);
            }
            /* Early return prevents us from reaching the cmd-unknown call
             * after the for-loop */
            return;
        }
    }
    (void) serial_cmd_rsp_send(p_cmd->opcode, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
}
