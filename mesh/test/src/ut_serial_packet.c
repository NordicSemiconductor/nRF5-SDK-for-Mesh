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
#include <string.h>
#include "serial_cmd.h"
#include "serial_evt.h"
#include "serial_packet.h"

#define _TEST_PACKET_EQUAL(_packet, _arr) do {                               \
        uint8_t* arr = (uint8_t*) _arr;                                     \
        TEST_ASSERT_EQUAL_HEX8_ARRAY(arr, (uint8_t*) &_packet, arr[0] + 1); \
    } while (0)

#define TEST_PACKET_EQUAL(pkt, ...) _TEST_PACKET_EQUAL(pkt, ((uint8_t[]){__VA_ARGS__}))

void setUp(void)
{

}

void tearDown(void)
{

}




/******** Tests ********/
void test_cmd_device(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_DEVICE_ECHO;
    cmd.length = 6;
    memcpy(cmd.payload.cmd.device.echo.data, "hello", 5);
    TEST_PACKET_EQUAL(cmd, 0x06, 0x02, 'h', 'e', 'l', 'l', 'o');
}

void test_cmd_config(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_ADV_ADDR_SET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_config_adv_addr_t);
    cmd.payload.cmd.config.adv_addr.addr_type = 8;
    memcpy(cmd.payload.cmd.config.adv_addr.adv_addr, "advadd", 6);
    TEST_PACKET_EQUAL(cmd, 0x08, 0x40, 8, 'a', 'd', 'v', 'a', 'd', 'd');

    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_CHANNEL_MAP_SET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_config_channel_map_t);
    cmd.payload.cmd.config.channel_map.channel_map = 0x93;
    TEST_PACKET_EQUAL(cmd, 0x02, 0x42, 0x93);

    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_TX_POWER_SET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_config_tx_power_t);
    cmd.payload.cmd.config.tx_power.tx_power = SERIAL_CMD_TX_POWER_VALUE_Neg4dBm;
    TEST_PACKET_EQUAL(cmd, 0x02, 0x44, 0xFC);

    cmd.opcode = SERIAL_OPCODE_CMD_CONFIG_UUID_SET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_config_uuid_t);
    for (uint32_t i = 0; i < 16; i++)
        cmd.payload.cmd.config.uuid.uuid[i] = i;
    TEST_PACKET_EQUAL(cmd,
 0x11, 0x53, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

}

void test_cmd_mesh(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_ADD;
    cmd.length = 19;
    cmd.payload.cmd.mesh.subnet_add.net_key_index = 0x1234;
    for (uint32_t i = 0; i < 16; i++)
        cmd.payload.cmd.mesh.subnet_add.key[i] = i;
    TEST_PACKET_EQUAL(cmd, 19, 0x92, 0x34, 0x12,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_UPDATE;
    cmd.length = 19;
    cmd.payload.cmd.mesh.subnet_update.subnet_handle = 0xABCD;
    for (uint32_t i = 0; i < 16; i++)
        cmd.payload.cmd.mesh.subnet_update.key[i] = i;
    TEST_PACKET_EQUAL(cmd, 19, 0x93, 0xCD, 0xAB,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_SUBNET_DELETE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.subnet_delete.subnet_handle = 0x1234;
    TEST_PACKET_EQUAL(cmd, 3, 0x94, 0x34, 0x12);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_ADD;
    cmd.length = 21;
    cmd.payload.cmd.mesh.appkey_add.app_key_index = 0x1234;
    cmd.payload.cmd.mesh.appkey_add.subnet_handle = 0x5678;
    for (uint32_t i = 0; i < 16; i++)
        cmd.payload.cmd.mesh.appkey_add.key[i] = i;
    TEST_PACKET_EQUAL(cmd, 21, 0x97, 0x34, 0x12, 0x78, 0x56,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_UPDATE;
    cmd.length = 19;
    cmd.payload.cmd.mesh.appkey_update.appkey_handle = 0x5678;
    for (uint32_t i = 0; i < 16; i++)
        cmd.payload.cmd.mesh.appkey_update.key[i] = i;
    TEST_PACKET_EQUAL(cmd, 19, 0x98, 0x78, 0x56,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_DELETE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.appkey_delete.appkey_handle = 0x1234;
    TEST_PACKET_EQUAL(cmd, 3, 0x99, 0x34, 0x12);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_APPKEY_GET_ALL;
    cmd.length = 3;
    cmd.payload.cmd.mesh.appkey_get_all.subnet_handle = 0xABCD;
    TEST_PACKET_EQUAL(cmd, 3, 0x9A, 0xCD, 0xAB);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_DEVKEY_ADD;
    cmd.length = 21;
    cmd.payload.cmd.mesh.devkey_add.owner_addr = 0x1234;
    cmd.payload.cmd.mesh.devkey_add.subnet_handle = 0x5678;
    for (uint32_t i = 0; i < 16; i++)
        cmd.payload.cmd.mesh.devkey_add.key[i] = i;
    TEST_PACKET_EQUAL(cmd, 21, 0x9C, 0x34, 0x12, 0x78, 0x56,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_DEVKEY_DELETE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.devkey_delete.devkey_handle = 0x1234;
    TEST_PACKET_EQUAL(cmd, 3, 0x9D, 0x34, 0x12);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_subscription_add.address = 0xABCD;
    TEST_PACKET_EQUAL(cmd, 3, 0xA1, 0xCD, 0xAB);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_ADD_VIRTUAL;
    cmd.length = 17;
    for (uint32_t i = 0; i < 16; ++i)
        cmd.payload.cmd.mesh.addr_subscription_add_virtual.uuid[i] = i;
    TEST_PACKET_EQUAL(cmd, 17, 0xA2,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_SUBSCRIPTION_REMOVE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_subscription_remove.address_handle = 0x1234;
    TEST_PACKET_EQUAL(cmd, 3, 0xA3, 0x34, 0x12);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_publication_add.address = 0xabcd;
    TEST_PACKET_EQUAL(cmd, 3, 0xA4, 0xCD, 0xAB);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_ADD_VIRTUAL;
    cmd.length = 17;
    for (uint32_t i = 0; i < 16; ++i)
        cmd.payload.cmd.mesh.addr_publication_add_virtual.uuid[i] = i;
    TEST_PACKET_EQUAL(cmd, 17, 0xA5,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    cmd.opcode = SERIAL_OPCODE_CMD_MESH_ADDR_PUBLICATION_REMOVE;
    cmd.length = 3;
    cmd.payload.cmd.mesh.addr_publication_remove.address_handle = 0x1234;
    TEST_PACKET_EQUAL(cmd, 3, 0xA6, 0x34, 0x12);

    /* packet send */
    cmd.opcode = SERIAL_OPCODE_CMD_MESH_PACKET_SEND;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + SERIAL_CMD_MESH_PACKET_SEND_OVERHEAD + 8;
    cmd.payload.cmd.mesh.packet_send.appkey_handle = 0x1234;
    cmd.payload.cmd.mesh.packet_send.force_segmented = 0x67;
    cmd.payload.cmd.mesh.packet_send.src_addr = 0xabcd;
    cmd.payload.cmd.mesh.packet_send.dst_addr_handle = 0x7890;
    cmd.payload.cmd.mesh.packet_send.ttl = 0x56;
    cmd.payload.cmd.mesh.packet_send.transmic_size = 1;
    cmd.payload.cmd.mesh.packet_send.friendship_credential_flag = false;
    for (uint32_t i = 0; i < 8; i++)
        cmd.payload.cmd.mesh.packet_send.data[i] = i;
    TEST_PACKET_EQUAL(cmd, 19, 0xAB, 0x34, 0x12, 0xcd, 0xab, 0x90, 0x78, 0x56, 0x67, 0x01, 0x00,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07);
}

void test_cmd_dfu(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_DFU_REQUEST;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_request_t);
    cmd.payload.cmd.dfu.request.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    cmd.payload.cmd.dfu.request.fwid.application.company_id = 0x00000059;
    cmd.payload.cmd.dfu.request.fwid.application.app_id = 0x1234;
    cmd.payload.cmd.dfu.request.fwid.application.app_version = 0x12345678;
    cmd.payload.cmd.dfu.request.bank_addr = 0xaabbccdd;
    TEST_PACKET_EQUAL(cmd,
            0x10, 0xD1, 0x04, 0x59, 0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xdd, 0xcc, 0xbb, 0xaa);
    cmd.opcode = SERIAL_OPCODE_CMD_DFU_RELAY;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_relay_t);
    cmd.payload.cmd.dfu.relay.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    cmd.payload.cmd.dfu.relay.fwid.application.company_id = 0x00000059;
    cmd.payload.cmd.dfu.relay.fwid.application.app_id = 0x1234;
    cmd.payload.cmd.dfu.relay.fwid.application.app_version = 0x12345678;
    TEST_PACKET_EQUAL(cmd,
            0x0c, 0xD2, 0x04, 0x59, 0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
    cmd.opcode = SERIAL_OPCODE_CMD_DFU_BANK_INFO_GET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_bank_info_get_t);
    cmd.payload.cmd.dfu.relay.dfu_type = NRF_MESH_DFU_TYPE_SOFTDEVICE;
    TEST_PACKET_EQUAL(cmd,
            0x02, 0xD4, 0x01);
    cmd.opcode = SERIAL_OPCODE_CMD_DFU_BANK_FLASH;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_bank_flash_t);
    cmd.payload.cmd.dfu.relay.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    TEST_PACKET_EQUAL(cmd,
            0x02, 0xD5, 0x02);
}

void test_evt_cmd_rsp(void)
{
    serial_packet_t evt;
    evt.opcode = SERIAL_OPCODE_EVT_CMD_RSP;
    evt.payload.evt.cmd_rsp.opcode = 0x12;
    evt.payload.evt.cmd_rsp.status = 0x34;

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 2;
    evt.payload.evt.cmd_rsp.data.subnet.subnet_handle = 0x7856;
    TEST_PACKET_EQUAL(evt, 0x05, 0x84, 0x12, 0x34, 0x56, 0x78);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 32;
    for (uint32_t i = 0; i < 16; i++)
        evt.payload.evt.cmd_rsp.data.subnet_list.subnet_key_index[i] = 0xAB00 + i;
    TEST_PACKET_EQUAL(evt, 35, 0x84, 0x12, 0x34,
            0x00, 0xAB, 0x01, 0xAB, 0x02, 0xAB, 0x03, 0xAB, 0x04, 0xAB, 0x05, 0xAB, 0x06, 0xAB, 0x07, 0xAB, 0x08, 0xAB, 0x09, 0xAB, 0x0a, 0xAB, 0x0b, 0xAB, 0x0c, 0xAB, 0x0d, 0xAB, 0x0e, 0xAB, 0x0f, 0xAB);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 2;
    evt.payload.evt.cmd_rsp.data.appkey.appkey_handle = 0x7856;
    TEST_PACKET_EQUAL(evt, 0x05, 0x84, 0x12, 0x34, 0x56, 0x78);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 34;
    evt.payload.evt.cmd_rsp.data.appkey_list.subnet_handle = 0x1234;
    for (uint32_t i = 0; i < 16; i++)
        evt.payload.evt.cmd_rsp.data.appkey_list.appkey_key_index[i] = 0xAB00 + i;
    TEST_PACKET_EQUAL(evt, 37, 0x84, 0x12, 0x34, 0x34, 0x12,
            0x00, 0xAB, 0x01, 0xAB, 0x02, 0xAB, 0x03, 0xAB, 0x04, 0xAB, 0x05, 0xAB, 0x06, 0xAB, 0x07, 0xAB, 0x08, 0xAB, 0x09, 0xAB, 0x0a, 0xAB, 0x0b, 0xAB, 0x0c, 0xAB, 0x0d, 0xAB, 0x0e, 0xAB, 0x0f, 0xAB);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 2;
    evt.payload.evt.cmd_rsp.data.devkey.devkey_handle = 0x7856;
    TEST_PACKET_EQUAL(evt, 0x05, 0x84, 0x12, 0x34, 0x56, 0x78);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 2;
    evt.payload.evt.cmd_rsp.data.addr.address_handle = 0x7856;
    TEST_PACKET_EQUAL(evt, 0x05, 0x84, 0x12, 0x34, 0x56, 0x78);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 2;
    evt.payload.evt.cmd_rsp.data.list_size.list_size = 0x7856;
    TEST_PACKET_EQUAL(evt, 0x05, 0x84, 0x12, 0x34, 0x56, 0x78);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 2;
    evt.payload.evt.cmd_rsp.data.serial_version.serial_ver = 0x7856;
    TEST_PACKET_EQUAL(evt, 0x05, 0x84, 0x12, 0x34, 0x56, 0x78);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 1;
    evt.payload.evt.cmd_rsp.data.prov_ctx.context = 0x01;
    TEST_PACKET_EQUAL(evt, 0x04, 0x84, 0x12, 0x34, 0x01);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 7;
    evt.payload.evt.cmd_rsp.data.adv_addr.addr_type = 0x54;
    for (uint32_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
        evt.payload.evt.cmd_rsp.data.adv_addr.addr[i] = i;
    TEST_PACKET_EQUAL(evt, 10, 0x84, 0x12, 0x34, 0x54,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 16;
    for (uint32_t i = 0; i < NRF_MESH_UUID_SIZE; i++)
        evt.payload.evt.cmd_rsp.data.device_uuid.device_uuid[i] = i;
    TEST_PACKET_EQUAL(evt, 19, 0x84, 0x12, 0x34,
            0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 7;
    evt.payload.evt.cmd_rsp.data.beacon_params.beacon_slot = 0x12;
    evt.payload.evt.cmd_rsp.data.beacon_params.tx_power = 0x34;
    evt.payload.evt.cmd_rsp.data.beacon_params.channel_map = 0x56;
    evt.payload.evt.cmd_rsp.data.beacon_params.interval_ms = 0x89ABCDEF;
    TEST_PACKET_EQUAL(evt, 10, 0x84, 0x12, 0x34, 0x12, 0x34, 0x56, 0xEF, 0xCD, 0xAB, 0x89);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 20;
    evt.payload.evt.cmd_rsp.data.dfu_bank_info.dfu_type = 0x12;
    evt.payload.evt.cmd_rsp.data.dfu_bank_info.fwid.application.company_id = 0x12345678;
    evt.payload.evt.cmd_rsp.data.dfu_bank_info.fwid.application.app_id = 0xABCD;
    evt.payload.evt.cmd_rsp.data.dfu_bank_info.fwid.application.app_version = 0x12345678;
    evt.payload.evt.cmd_rsp.data.dfu_bank_info.is_signed = 0x90;
    evt.payload.evt.cmd_rsp.data.dfu_bank_info.start_addr = 0x12345678;
    evt.payload.evt.cmd_rsp.data.dfu_bank_info.length = 0x12345678;
    TEST_PACKET_EQUAL(evt, 23, 0x84, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xCD, 0xAB, 0x78, 0x56, 0x34, 0x12, 0x90, 0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

    evt.length = SERIAL_EVT_CMD_RSP_LEN_OVERHEAD + 14;
    evt.payload.evt.cmd_rsp.data.dfu_state.role = 0x12;
    evt.payload.evt.cmd_rsp.data.dfu_state.type = 0x34;
    evt.payload.evt.cmd_rsp.data.dfu_state.fwid.application.company_id = 0x12345678;
    evt.payload.evt.cmd_rsp.data.dfu_state.fwid.application.app_id = 0xABCD;
    evt.payload.evt.cmd_rsp.data.dfu_state.fwid.application.app_version = 0x12345678;
    evt.payload.evt.cmd_rsp.data.dfu_state.state = 0x90;
    evt.payload.evt.cmd_rsp.data.dfu_state.data_progress = 0x39;
    TEST_PACKET_EQUAL(evt, 17, 0x84, 0x12, 0x34, 0x12, 0x34, 0x78, 0x56, 0x34, 0x12, 0xCD, 0xAB, 0x78, 0x56, 0x34, 0x12, 0x90, 0x39);
}

void test_evt_device(void)
{
    serial_packet_t evt;
    evt.opcode = SERIAL_OPCODE_EVT_DEVICE_STARTED;
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_device_started_t);
    evt.payload.evt.device.started.operating_mode = SERIAL_DEVICE_OPERATING_MODE_APPLICATION;
    evt.payload.evt.device.started.hw_error = 0x22;
    evt.payload.evt.device.started.data_credit_available = 0x55;
    TEST_PACKET_EQUAL(evt, 0x04, 0x81, 0x02, 0x22, 0x55);

    evt.opcode = SERIAL_OPCODE_EVT_DEVICE_ECHO_RSP;
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + 5;
    memcpy(evt.payload.evt.device.echo.data, "hello", 5);
    TEST_PACKET_EQUAL(evt, 0x06, 0x82, 'h', 'e', 'l', 'l', 'o');
}

void test_evt_mesh(void)
{
    serial_packet_t evt;
    evt.opcode = SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_UNICAST;
    evt.length = SERIAL_EVT_MESH_MESSAGE_RECEIVED_LEN_OVERHEAD + 6;
    evt.payload.evt.mesh.message_received.src = 0x1234;
    evt.payload.evt.mesh.message_received.dst = 0x5678;
    evt.payload.evt.mesh.message_received.appkey_handle = 0xABCD;
    evt.payload.evt.mesh.message_received.subnet_handle = 0xEF01;
    evt.payload.evt.mesh.message_received.rssi = 0x12;
    evt.payload.evt.mesh.message_received.ttl = 0xf0;
    evt.payload.evt.mesh.message_received.adv_addr_type = 0x12;
    for (uint32_t i = 0; i < 6; i++)
        evt.payload.evt.mesh.message_received.adv_addr[i] = i;
    evt.payload.evt.mesh.message_received.actual_length = 0x1234;
    for (uint32_t i = 0; i < 6; i++)
        evt.payload.evt.mesh.message_received.data[i] = i + 0x10;
    TEST_PACKET_EQUAL(evt, 26, 0xD0, 0x34, 0x12, 0x78, 0x56, 0xcd, 0xab, 0x01, 0xef, 0xf0, 0x12, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x12, 0x34, 0x12, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15);
    evt.opcode = SERIAL_OPCODE_EVT_MESH_MESSAGE_RECEIVED_SUBSCRIPTION;
    TEST_PACKET_EQUAL(evt, 26, 0xD1, 0x34, 0x12, 0x78, 0x56, 0xcd, 0xab, 0x01, 0xef, 0xf0, 0x12, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x12, 0x34, 0x12, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15);

    evt.opcode = SERIAL_OPCODE_EVT_MESH_IV_UPDATE_NOTIFICATION;
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_mesh_iv_update_t);
    evt.payload.evt.mesh.iv_update.iv_index = 0x12345678;
    TEST_PACKET_EQUAL(evt, 5, 0xD3, 0x78, 0x56, 0x34, 0x12);

}

void test_evt_dfu(void)
{
    serial_packet_t evt;
    memset(&evt, 0x00, sizeof(serial_packet_t));
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_req_relay_t);
    evt.opcode = SERIAL_OPCODE_EVT_DFU_REQ_RELAY;
    evt.payload.evt.dfu.req_relay.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    evt.payload.evt.dfu.req_relay.fwid.bootloader.bl_id = 0xAB;
    evt.payload.evt.dfu.req_relay.fwid.bootloader.bl_version = 0xCD;
    evt.payload.evt.dfu.req_relay.authority = 0xAB;
    TEST_PACKET_EQUAL(evt,
            0x0d, 0xA0, 0x02, 0xAB, 0xCD, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xAB);

    memset(&evt, 0x00, sizeof(serial_packet_t));
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_req_source_t);
    evt.opcode = SERIAL_OPCODE_EVT_DFU_REQ_SOURCE;
    evt.payload.evt.dfu.req_source.dfu_type = NRF_MESH_DFU_TYPE_SOFTDEVICE;
    TEST_PACKET_EQUAL(evt,
            0x02, 0xA1, 0x01);

    memset(&evt, 0x00, sizeof(serial_packet_t));
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_start_t);
    evt.opcode = SERIAL_OPCODE_EVT_DFU_START;
    evt.payload.evt.dfu.start.role = NRF_MESH_DFU_ROLE_SOURCE;
    evt.payload.evt.dfu.start.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    evt.payload.evt.dfu.start.fwid.application.company_id = 0x00000059;
    evt.payload.evt.dfu.start.fwid.application.app_id = 0x1234;
    evt.payload.evt.dfu.start.fwid.application.app_version = 0x12345678;
    TEST_PACKET_EQUAL(evt,
            0x0d, 0xA2, 0x03, 0x04, 0x59, 0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

    memset(&evt, 0x00, sizeof(serial_packet_t));
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_end_t);
    evt.opcode = SERIAL_OPCODE_EVT_DFU_END;
    evt.payload.evt.dfu.end.role = NRF_MESH_DFU_ROLE_SOURCE;
    evt.payload.evt.dfu.end.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    evt.payload.evt.dfu.end.fwid.application.company_id = 0x00000059;
    evt.payload.evt.dfu.end.fwid.application.app_id = 0x1234;
    evt.payload.evt.dfu.end.fwid.application.app_version = 0x12345678;
    evt.payload.evt.dfu.end.end_reason = 0xAB;
    TEST_PACKET_EQUAL(evt,
            0x0e, 0xA3, 0x03, 0x04, 0x59, 0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12, 0xAB);

    memset(&evt, 0x00, sizeof(serial_packet_t));
    evt.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_firmware_outdated_t);
    evt.opcode = SERIAL_OPCODE_EVT_DFU_FIRMWARE_OUTDATED;
    evt.payload.evt.dfu.firmware_outdated.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    evt.payload.evt.dfu.firmware_outdated.available_fwid.application.company_id = 0x00000059;
    evt.payload.evt.dfu.firmware_outdated.available_fwid.application.app_id = 0x1234;
    evt.payload.evt.dfu.firmware_outdated.available_fwid.application.app_version = 0xAABBCCDD;
    evt.payload.evt.dfu.firmware_outdated.current_fwid.application.company_id = 0x00000059;
    evt.payload.evt.dfu.firmware_outdated.current_fwid.application.app_id = 0x1234;
    evt.payload.evt.dfu.firmware_outdated.current_fwid.application.app_version = 0x12345678;
    TEST_PACKET_EQUAL(evt,
            22, 0xA5, 0x04, 0x59, 0x00, 0x00, 0x00, 0x34, 0x12, 0xDD, 0xCC, 0xBB, 0xAA, 0x59, 0x00, 0x00, 0x00, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
}

