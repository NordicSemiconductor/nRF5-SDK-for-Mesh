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

#include <unity.h>
#include <cmock.h>

#include "serial_handler_openmesh.h"
#include "serial_packet.h"
#include "serial_status.h"

#include "nrf_mesh_dfu_mock.h"
#include "serial_mock.h"

#define EXPECT_ACK(_opcode, _error_code)                                \
    do {                                                                \
        serial_translate_error_ExpectAndReturn(_error_code, 0xAA);      \
        serial_cmd_rsp_send_Expect(_opcode, 0xAA, NULL, 0); \
    } while (0)


void setUp(void)
{
    serial_mock_Init();
}

void tearDown(void)
{
    serial_mock_Verify();
    serial_mock_Destroy();
}


/******** Tests ********/

void test_openmesh_rx(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_OPENMESH_DFU_DATA;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + 43;
    memset(cmd.payload.cmd.openmesh.dfu_data.dfu_packet, 0xAB, sizeof(nrf_mesh_dfu_packet_t));
    nrf_mesh_dfu_rx_ExpectAndReturn(cmd.payload.cmd.openmesh.dfu_data.dfu_packet, 43, NULL, NRF_SUCCESS);
    EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
    serial_handler_openmesh_rx(&cmd);
    nrf_mesh_dfu_rx_ExpectAndReturn(cmd.payload.cmd.openmesh.dfu_data.dfu_packet, 43, NULL, 0x895);
    EXPECT_ACK(cmd.opcode, 0x895);
    serial_handler_openmesh_rx(&cmd);
}

