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

#include <unity.h>
#include <cmock.h>

#include "serial_handler_config.h"

#include "serial_status.h"

#include "advertiser_mock.h"
#include "nrf_mesh_configure_mock.h"
#include "serial_mock.h"

#define EXPECT_ACK(_opcode, _error_code)                                \
    do {                                                                \
        serial_translate_error_ExpectAndReturn(_error_code, 0xAA);      \
        serial_cmd_rsp_send_Expect(_opcode, 0xAA, NULL, 0); \
    } while (0)

#define EXPECT_NACK(_opcode, _serial_status)                            \
    do {                                                                \
        serial_translate_error_IgnoreAndReturn(_serial_status);         \
        serial_cmd_rsp_send_Expect(_opcode, _serial_status, NULL, 0); \
    } while (0)

#define EXPECT_ACK_WITH_PAYLOAD(_opcode, _data, _len)                   \
    do { \
        serial_cmd_rsp_send_ExpectWithArray(_opcode, 0, (uint8_t *) (_data), _len, _len); \
    } while (0)


/*****************************************************************************
 * Test initialization and finalization
 *****************************************************************************/

void setUp(void)
{
    advertiser_mock_Init();
    nrf_mesh_configure_mock_Init();
    serial_mock_Init();
}

void tearDown(void)
{
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    nrf_mesh_configure_mock_Verify();
    nrf_mesh_configure_mock_Destroy();
    serial_mock_Verify();
    serial_mock_Destroy();
}

/*****************************************************************************
 * Tests
 *****************************************************************************/

void test_config_rx(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_GET;
    cmd.length = 1;
    ble_gap_addr_t addr;
    for (uint32_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
    {
        addr.addr[i] = i;
    }
    addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    advertiser_address_default_get_Expect(NULL);
    advertiser_address_default_get_IgnoreArg_p_addr();
    advertiser_address_default_get_ReturnMemThruPtr_p_addr(&addr, sizeof(addr));
    serial_evt_cmd_rsp_data_adv_addr_t adv_addr_rsp;
    adv_addr_rsp.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
    memcpy(adv_addr_rsp.addr, addr.addr, BLE_GAP_ADDR_LEN);
    EXPECT_ACK_WITH_PAYLOAD(SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_GET, &adv_addr_rsp, 7);
    serial_handler_config_rx(&cmd);

    cmd.length = 2;
    EXPECT_NACK(SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_GET, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_config_rx(&cmd);

    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_UUID_SET;
    cmd.length = 17;
    for (uint32_t i = 0; i < 16; i++)
        cmd.payload.cmd.config.uuid.uuid[i] = i;
    nrf_mesh_configure_device_uuid_set_Expect(cmd.payload.cmd.config.uuid.uuid);
    EXPECT_NACK(SERIAL_OPCODE_CMD_CONFIG_UUID_SET, SERIAL_STATUS_SUCCESS);
    serial_handler_config_rx(&cmd);
    cmd.length = 16;
    EXPECT_NACK(SERIAL_OPCODE_CMD_CONFIG_UUID_SET, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_config_rx(&cmd);
    cmd.length = 18;
    EXPECT_NACK(SERIAL_OPCODE_CMD_CONFIG_UUID_SET, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_config_rx(&cmd);

    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_UUID_GET;
    cmd.length = 1;
    nrf_mesh_configure_device_uuid_get_ExpectAndReturn(cmd.payload.cmd.config.uuid.uuid);
    EXPECT_ACK_WITH_PAYLOAD(SERIAL_OPCODE_CMD_CONFIG_UUID_GET, cmd.payload.cmd.config.uuid.uuid, 16);
    serial_handler_config_rx(&cmd);
    cmd.length = 2;
    EXPECT_NACK(SERIAL_OPCODE_CMD_CONFIG_UUID_GET, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_config_rx(&cmd);

    TEST_IGNORE_MESSAGE("Config not fully implemented.");
    /*lint -e527 Unreachable code */

    cmd.length = 8;
    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_SET;
    cmd.payload.cmd.config.adv_addr.addr_type = 1;
    for (uint32_t i = 0; i < 6; i++)
        cmd.payload.cmd.config.adv_addr.adv_addr[i] = i;

    EXPECT_NACK(SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_SET, SERIAL_STATUS_SUCCESS);
    serial_handler_config_rx(&cmd);
}


