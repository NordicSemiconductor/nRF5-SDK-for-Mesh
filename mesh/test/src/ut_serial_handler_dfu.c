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

#include "serial_handler_dfu.h"
#include "serial_status.h"

#include "nrf_mesh_dfu_mock.h"
#include "nrf_mesh_events_mock.h"
#include "serial_mock.h"

#define EXPECT_PACKET()                                 \
    do {                                                \
        m_tx_cb_count++;                                \
        serial_tx_StubWithCallback(serial_tx_cb);       \
    } while (0)

#define EXPECT_ACK(_opcode, _error_code)                           \
    do {                                                           \
        serial_translate_error_ExpectAndReturn(_error_code, 0xAA); \
        serial_cmd_rsp_send_Expect(_opcode, 0xAA, NULL, 0);        \
    } while (0)

#define EXPECT_ACK_NO_TRANSLATE(_opcode, _serial_status)              \
    do {                                                              \
        serial_cmd_rsp_send_Expect(_opcode, _serial_status, NULL, 0); \
    } while (0)

#define EXPECT_ACK_WITH_PAYLOAD(_opcode, _data, _len)                   \
    do { \
        serial_cmd_rsp_send_ExpectWithArray(_opcode, 0, (uint8_t *) (_data), _len, _len); \
    } while (0)

/*****************************************************************************
 * Mock functions
 *****************************************************************************/

static uint32_t m_beacon_enable_calls_expected;
static uint32_t m_beacon_enable_calls_actual;
static uint32_t m_tx_cb_count;
static uint32_t m_tx_cb_count_actual;

static serial_packet_t m_expected_packet;
static const nrf_mesh_evt_handler_t * mp_evt_handler;

static void serial_tx_cb(const serial_packet_t* p_packet, int cmock_num_calls)
{
    m_tx_cb_count_actual = cmock_num_calls + 1;
    if (m_tx_cb_count != (uint32_t) cmock_num_calls + 1)
    {
        TEST_ASSERT_EQUAL(m_tx_cb_count, cmock_num_calls + 1);
    }
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, p_packet->length, "Sent packet has length 0");
    TEST_ASSERT_NOT_EQUAL_MESSAGE(0, m_expected_packet.length, "Reference packet has length 0");
    TEST_ASSERT_EQUAL_HEX8_ARRAY((uint8_t *) &m_expected_packet,
            (uint8_t *) p_packet,
            p_packet->length + SERIAL_PACKET_LENGTH_OVERHEAD);
}

static void nrf_mesh_evt_handler_add_cb(nrf_mesh_evt_handler_t * p_handler, int count)
{
    mp_evt_handler = p_handler;
}

static void callbacks_verify(void)
{
    TEST_ASSERT_EQUAL_MESSAGE(m_tx_cb_count, m_tx_cb_count_actual, "Serial TX called too few times.");
    TEST_ASSERT_EQUAL_MESSAGE(m_beacon_enable_calls_expected, m_beacon_enable_calls_actual, "Beacon enable called too few times.");
}

/*****************************************************************************
 * Test initialization and finalization
 *****************************************************************************/

void setUp(void)
{
    nrf_mesh_dfu_mock_Init();
    nrf_mesh_events_mock_Init();
    serial_mock_Init();

    nrf_mesh_evt_handler_add_StubWithCallback(nrf_mesh_evt_handler_add_cb);
    serial_handler_dfu_init();
}

void tearDown(void)
{
    nrf_mesh_dfu_mock_Verify();
    nrf_mesh_dfu_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    serial_mock_Verify();
    serial_mock_Destroy();
}

/*****************************************************************************
 * Tests
 *****************************************************************************/

void test_dfu_rx(void)
{
    uint32_t dummy_bank_addr;
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_DFU_JUMP_TO_BOOTLOADER;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    nrf_mesh_dfu_jump_to_bootloader_ExpectAndReturn(NRF_SUCCESS);
    EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_jump_to_bootloader_ExpectAndReturn(NRF_ERROR_INVALID_STATE);
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
    serial_handler_dfu_rx(&cmd);
    cmd.length += 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    cmd.length -= 2;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_mock_Verify();
    callbacks_verify();

    cmd.opcode = SERIAL_OPCODE_CMD_DFU_REQUEST;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_request_t);
    cmd.payload.cmd.dfu.request.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    cmd.payload.cmd.dfu.request.fwid.application.company_id = 0x12345678;
    cmd.payload.cmd.dfu.request.fwid.application.app_id = 0x1234;
    cmd.payload.cmd.dfu.request.fwid.application.app_version = 0x89ABCDEF;
    cmd.payload.cmd.dfu.request.bank_addr = (uint32_t) &dummy_bank_addr;
    nrf_mesh_dfu_request_ExpectAndReturn(NRF_MESH_DFU_TYPE_APPLICATION,
                                         &cmd.payload.cmd.dfu.request.fwid,
                                         &dummy_bank_addr, NRF_SUCCESS);
    EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_request_ExpectAndReturn(NRF_MESH_DFU_TYPE_APPLICATION,
                                         &cmd.payload.cmd.dfu.request.fwid,
                                         &dummy_bank_addr, NRF_ERROR_INVALID_STATE);
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
    serial_handler_dfu_rx(&cmd);
    cmd.length += 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    cmd.length -= 2;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_mock_Verify();
    callbacks_verify();

    cmd.opcode = SERIAL_OPCODE_CMD_DFU_RELAY;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_relay_t);
    cmd.payload.cmd.dfu.relay.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    cmd.payload.cmd.dfu.relay.fwid.application.company_id = 0x12345678;
    cmd.payload.cmd.dfu.relay.fwid.application.app_id = 0x1234;
    cmd.payload.cmd.dfu.relay.fwid.application.app_version = 0x89ABCDEF;
    nrf_mesh_dfu_relay_ExpectAndReturn(NRF_MESH_DFU_TYPE_APPLICATION,
                                       &cmd.payload.cmd.dfu.relay.fwid, NRF_SUCCESS);
    EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_relay_ExpectAndReturn(NRF_MESH_DFU_TYPE_APPLICATION,
                                       &cmd.payload.cmd.dfu.relay.fwid, NRF_ERROR_INVALID_STATE);
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
    serial_handler_dfu_rx(&cmd);
    cmd.length += 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    cmd.length -= 2;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_mock_Verify();
    callbacks_verify();

    cmd.opcode = SERIAL_OPCODE_CMD_DFU_ABORT;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    nrf_mesh_dfu_abort_ExpectAndReturn(NRF_SUCCESS);
    EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_abort_ExpectAndReturn(NRF_ERROR_INVALID_STATE);
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
    serial_handler_dfu_rx(&cmd);
    cmd.length += 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    cmd.length -= 2;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);

    cmd.opcode = SERIAL_OPCODE_CMD_DFU_BANK_INFO_GET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_bank_info_get_t);
    cmd.payload.cmd.dfu.bank_info.dfu_type = NRF_MESH_DFU_TYPE_APPLICATION;
    nrf_mesh_dfu_bank_info_get_ExpectAndReturn(NRF_MESH_DFU_TYPE_APPLICATION, NULL, NRF_SUCCESS);
    nrf_mesh_dfu_bank_info_get_IgnoreArg_p_bank_info();
    nrf_mesh_dfu_bank_info_t bank_info;
    memset(&bank_info, 0, sizeof(bank_info));
    bank_info.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER; // Different from request, just to avoid any assumptions being made in this module
    bank_info.fwid.bootloader.bl_id = 0x01;
    bank_info.fwid.bootloader.bl_version = 0x02;
    bank_info.is_signed = true;
    bank_info.p_start_addr = (uint32_t*) 0x12345678;
    bank_info.length = 0x10203040;
    nrf_mesh_dfu_bank_info_get_ReturnMemThruPtr_p_bank_info(&bank_info, sizeof(bank_info));
    serial_evt_cmd_rsp_data_dfu_bank_info_t cmd_rsp_data;
    cmd_rsp_data.dfu_type   = bank_info.dfu_type;
    cmd_rsp_data.fwid       = bank_info.fwid;
    cmd_rsp_data.is_signed  = bank_info.is_signed;
    cmd_rsp_data.start_addr = (uint32_t) bank_info.p_start_addr;
    cmd_rsp_data.length     = bank_info.length;
    EXPECT_ACK_WITH_PAYLOAD(cmd.opcode, &cmd_rsp_data, sizeof(cmd_rsp_data));
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_bank_info_get_ExpectAndReturn(NRF_MESH_DFU_TYPE_APPLICATION, NULL, NRF_ERROR_INVALID_STATE);
    nrf_mesh_dfu_bank_info_get_IgnoreArg_p_bank_info();
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
    serial_handler_dfu_rx(&cmd);
    cmd.length += 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    cmd.length -= 2;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);

    cmd.opcode = SERIAL_OPCODE_CMD_DFU_BANK_FLASH;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_dfu_bank_flash_t);
    cmd.payload.cmd.dfu.bank_flash.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    nrf_mesh_dfu_bank_flash_ExpectAndReturn(NRF_MESH_DFU_TYPE_BOOTLOADER, NRF_SUCCESS);
    EXPECT_ACK(cmd.opcode, NRF_SUCCESS);
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_bank_flash_ExpectAndReturn(NRF_MESH_DFU_TYPE_BOOTLOADER, NRF_ERROR_INVALID_STATE);
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
    serial_handler_dfu_rx(&cmd);
    cmd.length += 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    cmd.length -= 2;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);

    cmd.opcode = SERIAL_OPCODE_CMD_DFU_STATE_GET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    nrf_mesh_dfu_transfer_state_t state;
    memset(&state, 0, sizeof(nrf_mesh_dfu_transfer_state_t));
    state.role = NRF_MESH_DFU_ROLE_TARGET;
    state.type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    state.fwid.bootloader.bl_id = 0x12;
    state.fwid.bootloader.bl_version = 0x34;
    state.state = NRF_MESH_DFU_STATE_TARGET;
    state.data_progress = 67;
    serial_evt_cmd_rsp_data_dfu_state_t cmd_rsp_data_state;
    memset(&cmd_rsp_data_state, 0, sizeof(serial_evt_cmd_rsp_data_dfu_state_t));
    cmd_rsp_data_state.role                       = state.role;
    cmd_rsp_data_state.type                       = state.type;
    cmd_rsp_data_state.fwid.bootloader.bl_id      = state.fwid.bootloader.bl_id;
    cmd_rsp_data_state.fwid.bootloader.bl_version = state.fwid.bootloader.bl_version;
    cmd_rsp_data_state.state                      = state.state;
    cmd_rsp_data_state.data_progress              = state.data_progress;

    nrf_mesh_dfu_state_get_ExpectAndReturn(NULL, NRF_SUCCESS);
    nrf_mesh_dfu_state_get_IgnoreArg_p_dfu_transfer_state();
    nrf_mesh_dfu_state_get_ReturnMemThruPtr_p_dfu_transfer_state(&state, sizeof(state));
    EXPECT_ACK_WITH_PAYLOAD(cmd.opcode, &cmd_rsp_data_state, sizeof(serial_evt_cmd_rsp_data_dfu_state_t));
    serial_handler_dfu_rx(&cmd);
    nrf_mesh_dfu_state_get_ExpectAndReturn(NULL, NRF_ERROR_INVALID_STATE);
    nrf_mesh_dfu_state_get_IgnoreArg_p_dfu_transfer_state();
    nrf_mesh_dfu_state_get_ReturnMemThruPtr_p_dfu_transfer_state(&state, sizeof(state));
    EXPECT_ACK(cmd.opcode, NRF_ERROR_INVALID_STATE);
    serial_handler_dfu_rx(&cmd);
    cmd.length += 1;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
    cmd.length -= 2;
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_handler_dfu_rx(&cmd);
}

void test_event_rx(void)
{
    uint8_t packet[NRF_MESH_SERIAL_PACKET_OVERHEAD + NRF_MESH_SERIAL_PAYLOAD_MAXLEN];
    serial_packet_t * p_packet = (serial_packet_t *) packet;

    nrf_mesh_evt_t evt;
    /* DFU fw outdated */
    memset(&m_expected_packet, 0, sizeof(serial_packet_t));
    memset(&evt, 0, sizeof(nrf_mesh_evt_t));
    evt.type = NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED;
    evt.params.dfu.fw_outdated.transfer.dfu_type = NRF_MESH_DFU_TYPE_SOFTDEVICE;
    evt.params.dfu.fw_outdated.transfer.id.softdevice = 0x1234;
    evt.params.dfu.fw_outdated.current.softdevice = 0x6543;
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DFU_FIRMWARE_OUTDATED;
    m_expected_packet.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_firmware_outdated_t);
    m_expected_packet.payload.evt.dfu.firmware_outdated.available_fwid.softdevice = 0x1234;
    m_expected_packet.payload.evt.dfu.firmware_outdated.dfu_type = NRF_MESH_DFU_TYPE_SOFTDEVICE;
    m_expected_packet.payload.evt.dfu.firmware_outdated.current_fwid.softdevice = 0x6543;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    mp_evt_handler->evt_cb(&evt);
    callbacks_verify();

    /* DFU fw outdated no auth */
    evt.type = NRF_MESH_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH;
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DFU_FIRMWARE_OUTDATED_NO_AUTH;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    mp_evt_handler->evt_cb(&evt);
    callbacks_verify();

    /* DFU req relay */
    memset(&m_expected_packet, 0, sizeof(serial_packet_t));
    memset(&evt, 0, sizeof(nrf_mesh_evt_t));
    evt.type = NRF_MESH_EVT_DFU_REQ_RELAY;
    evt.params.dfu.req_relay.transfer.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    evt.params.dfu.req_relay.transfer.id.bootloader.bl_id = 0x12;
    evt.params.dfu.req_relay.transfer.id.bootloader.bl_version = 0x34;
    evt.params.dfu.req_relay.authority = 0x06;
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DFU_REQ_RELAY;
    m_expected_packet.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_req_relay_t);
    m_expected_packet.payload.evt.dfu.req_relay.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    m_expected_packet.payload.evt.dfu.req_relay.fwid.bootloader.bl_id = 0x12;
    m_expected_packet.payload.evt.dfu.req_relay.fwid.bootloader.bl_version = 0x34;
    m_expected_packet.payload.evt.dfu.req_relay.authority = 0x06;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    mp_evt_handler->evt_cb(&evt);
    callbacks_verify();

    /** DFU request source */
    memset(&m_expected_packet, 0, sizeof(serial_packet_t));
    memset(&evt, 0, sizeof(nrf_mesh_evt_t));
    evt.type = NRF_MESH_EVT_DFU_REQ_SOURCE;
    evt.params.dfu.req_source.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DFU_REQ_SOURCE;
    m_expected_packet.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_req_source_t);
    m_expected_packet.payload.evt.dfu.req_source.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    mp_evt_handler->evt_cb(&evt);
    callbacks_verify();

    /** DFU transfer starting */
    memset(&m_expected_packet, 0, sizeof(serial_packet_t));
    memset(&evt, 0, sizeof(nrf_mesh_evt_t));
    evt.type = NRF_MESH_EVT_DFU_START;
    evt.params.dfu.start.role = NRF_MESH_DFU_ROLE_TARGET;
    evt.params.dfu.start.transfer.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    evt.params.dfu.start.transfer.id.bootloader.bl_id = 0x12;
    evt.params.dfu.start.transfer.id.bootloader.bl_version = 0x34;
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DFU_START;
    m_expected_packet.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_start_t);
    m_expected_packet.payload.evt.dfu.start.role = NRF_MESH_DFU_ROLE_TARGET;
    m_expected_packet.payload.evt.dfu.start.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    m_expected_packet.payload.evt.dfu.start.fwid.bootloader.bl_id = 0x12;
    m_expected_packet.payload.evt.dfu.start.fwid.bootloader.bl_version = 0x34;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    mp_evt_handler->evt_cb(&evt);
    callbacks_verify();

    /** DFU transfer ended. */
    memset(&m_expected_packet, 0, sizeof(serial_packet_t));
    memset(&evt, 0, sizeof(nrf_mesh_evt_t));
    evt.type = NRF_MESH_EVT_DFU_END;
    evt.params.dfu.end.role = NRF_MESH_DFU_ROLE_TARGET;
    evt.params.dfu.end.transfer.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    evt.params.dfu.end.transfer.id.bootloader.bl_id = 0x12;
    evt.params.dfu.end.transfer.id.bootloader.bl_version = 0x34;
    evt.params.dfu.end.end_reason = NRF_MESH_DFU_END_ERROR_BANK_IN_BOOTLOADER_AREA;
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DFU_END;
    m_expected_packet.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_end_t);
    m_expected_packet.payload.evt.dfu.end.role = NRF_MESH_DFU_ROLE_TARGET;
    m_expected_packet.payload.evt.dfu.end.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    m_expected_packet.payload.evt.dfu.end.fwid.bootloader.bl_id = 0x12;
    m_expected_packet.payload.evt.dfu.end.fwid.bootloader.bl_version = 0x34;
    m_expected_packet.payload.evt.dfu.end.end_reason = NRF_MESH_DFU_END_ERROR_BANK_IN_BOOTLOADER_AREA;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    mp_evt_handler->evt_cb(&evt);
    callbacks_verify();

    /** DFU bank available. */
    uint32_t dummy_bank;
    memset(&m_expected_packet, 0, sizeof(serial_packet_t));
    memset(&evt, 0, sizeof(nrf_mesh_evt_t));
    evt.type = NRF_MESH_EVT_DFU_BANK_AVAILABLE;
    evt.params.dfu.bank.transfer.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    evt.params.dfu.bank.transfer.id.bootloader.bl_id = 0x12;
    evt.params.dfu.bank.transfer.id.bootloader.bl_version = 0x34;
    evt.params.dfu.bank.p_start_addr = &dummy_bank;
    evt.params.dfu.bank.length = 0x1234;
    evt.params.dfu.bank.is_signed = true;
    m_expected_packet.opcode = SERIAL_OPCODE_EVT_DFU_BANK_AVAILABLE;
    m_expected_packet.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_dfu_bank_t);
    m_expected_packet.payload.evt.dfu.bank.dfu_type = NRF_MESH_DFU_TYPE_BOOTLOADER;
    m_expected_packet.payload.evt.dfu.bank.fwid.bootloader.bl_id = 0x12;
    m_expected_packet.payload.evt.dfu.bank.fwid.bootloader.bl_version = 0x34;
    m_expected_packet.payload.evt.dfu.bank.start_addr = (uint32_t) &dummy_bank;
    m_expected_packet.payload.evt.dfu.bank.length = 0x1234;
    m_expected_packet.payload.evt.dfu.bank.is_signed = true;
    serial_packet_buffer_get_ExpectAndReturn(m_expected_packet.length, NULL,  NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    p_packet->length = m_expected_packet.length;
    EXPECT_PACKET();
    mp_evt_handler->evt_cb(&evt);
    callbacks_verify();
}

