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

#include "serial_status.h"
#include "serial.h"

#include <unity.h>
#include <cmock.h>
#include <string.h>
#include <stdint.h>

#include "nrf_error.h"

#include "nrf.h"
#include "nrf_mesh_serial.h"
#include "test_assert.h"

#include "bearer_event_mock.h"
#include "serial_bearer_mock.h"
#include "serial_handler_access_mock.h"
#include "serial_handler_app_mock.h"
#include "serial_handler_config_mock.h"
#include "serial_handler_dfu_mock.h"
#include "serial_handler_models_mock.h"
#include "serial_handler_device_mock.h"
#include "serial_handler_mesh_mock.h"
#include "serial_handler_prov_mock.h"
#include "serial_handler_openmesh_mock.h"

NRF_POWER_Type  * NRF_POWER;
static NRF_POWER_Type m_power;
static bearer_event_callback_t m_serial_process_cmd;
static uint32_t m_bearer_event_post_calls;

static uint32_t m_bearer_event_generic_post(bearer_event_callback_t callback, void* p_context, int cmock_num_calls)
{
    m_serial_process_cmd = callback;
    m_bearer_event_post_calls++;
    return NRF_SUCCESS;
}

void setUp(void)
{
    NRF_POWER = &m_power;
    serial_bearer_mock_Init();
    bearer_event_mock_Init();
    serial_handler_access_mock_Init();
    serial_handler_app_mock_Init();
    serial_handler_config_mock_Init();
    serial_handler_dfu_mock_Init();
    serial_handler_models_mock_Init();
    serial_handler_device_mock_Init();
    serial_handler_mesh_mock_Init();
    serial_handler_prov_mock_Init();
    serial_handler_openmesh_mock_Init();

    bearer_event_generic_post_StubWithCallback(m_bearer_event_generic_post);
}

void tearDown(void)
{
    serial_bearer_mock_Verify();
    serial_handler_app_mock_Verify();
    serial_handler_dfu_mock_Verify();
    serial_handler_config_mock_Verify();
    serial_handler_openmesh_mock_Verify();
    serial_handler_device_mock_Verify();
    serial_handler_mesh_mock_Verify();
    serial_handler_prov_mock_Verify();
    serial_bearer_mock_Verify();
    serial_bearer_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    serial_handler_access_mock_Verify();
    serial_handler_access_mock_Destroy();
    serial_handler_app_mock_Verify();
    serial_handler_app_mock_Destroy();
    serial_handler_config_mock_Verify();
    serial_handler_config_mock_Destroy();
    serial_handler_dfu_mock_Verify();
    serial_handler_dfu_mock_Destroy();
    serial_handler_models_mock_Verify();
    serial_handler_models_mock_Destroy();
    serial_handler_device_mock_Verify();
    serial_handler_device_mock_Destroy();
    serial_handler_mesh_mock_Verify();
    serial_handler_mesh_mock_Destroy();
    serial_handler_prov_mock_Verify();
    serial_handler_prov_mock_Destroy();
    serial_handler_openmesh_mock_Verify();
    serial_handler_openmesh_mock_Destroy();
}

void test_serial_invalid(void)
{
    TEST_ASSERT_EQUAL(NRF_MESH_SERIAL_STATE_UNINITIALIZED, serial_state_get());
    /* Cannot start serial without initializing it. */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, serial_start());
    TEST_ASSERT_EQUAL(NRF_MESH_SERIAL_STATE_UNINITIALIZED, serial_state_get());
    /* When the serial has not been started, serial_process should have no effect. */
    serial_process();
    /* serial_init can be called only once */
    serial_bearer_init_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_init());
    /* When the serial has not been started, serial_process should have no effect. */
    serial_process();
    TEST_ASSERT_EQUAL(0, m_bearer_event_post_calls);
    TEST_ASSERT_EQUAL(NRF_MESH_SERIAL_STATE_INITIALIZED, serial_state_get());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, serial_init());

    TEST_NRF_MESH_ASSERT_EXPECT(serial_cmd_rsp_send(0,0,NULL,1));
    uint8_t blah = 0;
    TEST_NRF_MESH_ASSERT_EXPECT(serial_cmd_rsp_send(0,0,&blah,0));

    serial_bearer_packet_buffer_get_ExpectAndReturn(sizeof(serial_evt_device_started_t) + NRF_MESH_SERIAL_PACKET_OVERHEAD, NULL, NRF_ERROR_INVALID_LENGTH);
    serial_bearer_packet_buffer_get_IgnoreArg_pp_packet();
    serial_handler_device_alloc_fail_report_Expect();
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_LENGTH, serial_start());
    TEST_ASSERT_EQUAL(NRF_MESH_SERIAL_STATE_INITIALIZED, serial_state_get());

    serial_packet_t serial_packet;
    serial_packet_t * p_packet = &serial_packet;
    /* packet buffer get and serial tx would fail without starting the serial */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, serial_packet_buffer_get(1, &p_packet));
    TEST_NRF_MESH_ASSERT_EXPECT(serial_tx(p_packet));

    serial_bearer_packet_buffer_get_ExpectAndReturn(sizeof(serial_evt_device_started_t) + NRF_MESH_SERIAL_PACKET_OVERHEAD, &p_packet, NRF_SUCCESS);
    serial_bearer_packet_buffer_get_IgnoreArg_pp_packet();
    serial_bearer_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    serial_bearer_tx_Expect(p_packet);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_start());
}

/* Since serial module can only be initialized and started once the following test case relies on the previous test case initializing the serial.*/
void test_serial_tx(void)
{
    serial_packet_t serial_packet;
    serial_packet_t * p_packet = &serial_packet;
    TEST_ASSERT_EQUAL(NRF_MESH_SERIAL_STATE_RUNNING, serial_state_get());
    serial_packet_t *p_serial_packet;
    serial_bearer_packet_buffer_get_ExpectAndReturn(0xFFF, &p_packet, NRF_SUCCESS);
    serial_bearer_packet_buffer_get_IgnoreArg_pp_packet();
    serial_bearer_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_packet_buffer_get(0xFFF, &p_serial_packet));
    TEST_ASSERT_EQUAL_PTR(p_packet, p_serial_packet);
    serial_bearer_tx_Expect(p_packet);
    serial_tx(p_serial_packet);

    uint8_t blah = 0xFA;
    serial_bearer_packet_buffer_get_ExpectAndReturn(1+SERIAL_EVT_CMD_RSP_LEN_OVERHEAD, &p_packet, NRF_SUCCESS);
    serial_bearer_packet_buffer_get_IgnoreArg_pp_packet();
    serial_bearer_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    serial_bearer_tx_Expect(p_packet);
    serial_cmd_rsp_send(0xF,0x10,&blah,1);
    TEST_ASSERT_EQUAL(SERIAL_OPCODE_EVT_CMD_RSP, p_packet->opcode);
    TEST_ASSERT_EQUAL(0xF, p_packet->payload.evt.cmd_rsp.opcode);
    TEST_ASSERT_EQUAL(0x10, p_packet->payload.evt.cmd_rsp.status);
    TEST_ASSERT_EQUAL(blah, *(uint8_t *) (&p_packet->payload.evt.cmd_rsp.data));
}

void test_serial_translate_error(void)
{
    struct error_mapping
    {
        uint32_t nrf_error;
        uint8_t serial_error;
    };

    #define ERROR_MAP_SIZE 23
    struct error_mapping error_map[ERROR_MAP_SIZE] =
        {
            {NRF_SUCCESS,                      SERIAL_STATUS_SUCCESS},
            {NRF_ERROR_SVC_HANDLER_MISSING,    SERIAL_STATUS_ERROR_INVALID_STATE},
            {NRF_ERROR_SOFTDEVICE_NOT_ENABLED, SERIAL_STATUS_ERROR_INVALID_STATE},
            {NRF_ERROR_INTERNAL,               SERIAL_STATUS_ERROR_INTERNAL},
            {NRF_ERROR_NO_MEM,                 SERIAL_STATUS_ERROR_REJECTED},
            {NRF_ERROR_NOT_FOUND,              SERIAL_STATUS_ERROR_REJECTED},
            {NRF_ERROR_NOT_SUPPORTED,          SERIAL_STATUS_ERROR_REJECTED},
            {NRF_ERROR_INVALID_PARAM,          SERIAL_STATUS_ERROR_INVALID_PARAMETER},
            {NRF_ERROR_INVALID_STATE,          SERIAL_STATUS_ERROR_INVALID_STATE},
            {NRF_ERROR_INVALID_LENGTH,         SERIAL_STATUS_ERROR_INVALID_LENGTH},
            {NRF_ERROR_INVALID_FLAGS,          SERIAL_STATUS_ERROR_INVALID_STATE},
            {NRF_ERROR_INVALID_DATA,           SERIAL_STATUS_ERROR_INVALID_DATA},
            {NRF_ERROR_DATA_SIZE,              SERIAL_STATUS_ERROR_INVALID_LENGTH},
            {NRF_ERROR_TIMEOUT,                SERIAL_STATUS_ERROR_TIMEOUT},
            {NRF_ERROR_NULL,                   SERIAL_STATUS_ERROR_INTERNAL},
            {NRF_ERROR_FORBIDDEN,              SERIAL_STATUS_ERROR_REJECTED},
            {NRF_ERROR_INVALID_ADDR,           SERIAL_STATUS_ERROR_INVALID_DATA},
            {NRF_ERROR_BUSY,                   SERIAL_STATUS_ERROR_BUSY},
            {BLE_ERROR_NOT_ENABLED,            SERIAL_STATUS_ERROR_REJECTED},
            {BLE_ERROR_INVALID_CONN_HANDLE,    SERIAL_STATUS_ERROR_UNKNOWN},
            {BLE_ERROR_INVALID_ATTR_HANDLE,    SERIAL_STATUS_ERROR_UNKNOWN},
            /* NRF_ERROR_STK_BASE_NUM+0x004 is  BLE_ERROR_NO_TX_PACKETS on s130, and BLE_ERROR_NO_TX_BUFFERS on s110 */
            {NRF_ERROR_STK_BASE_NUM+0x004,          SERIAL_STATUS_ERROR_UNKNOWN},
            {BLE_ERROR_INVALID_ROLE,           SERIAL_STATUS_ERROR_UNKNOWN},
        };

    for (uint32_t i = 0; i < ERROR_MAP_SIZE; ++i)
    {
        TEST_ASSERT_EQUAL(error_map[i].serial_error, serial_translate_error(error_map[i].nrf_error));
    }
}

/* Since serial module can only be initialized and started once the following test case relies on the previous test case initializing the serial.*/
void test_serial_rx(void)
{
    TEST_ASSERT_EQUAL(NRF_MESH_SERIAL_STATE_RUNNING, serial_state_get());
    /* Expect a call to the bearer_event_generic_post */
    serial_process();
    TEST_ASSERT_EQUAL(1, m_bearer_event_post_calls);
    /* No calls to the bearer_event_generic_post are expected */
    serial_process();
    TEST_ASSERT_EQUAL(1, m_bearer_event_post_calls);

    /* Call serial process cmd but return no available packets */
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /* Expect a call to the bearer_event_generic_post */
    serial_process();
    TEST_ASSERT_EQUAL(2, m_bearer_event_post_calls);

    /** Test handling of the supported opcodes */
    /* Call serial process cmd and test with valid packets of DEVICE type */
    serial_packet_t serial_packet;
    serial_packet.length = 1;
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_DEVICE_START;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_device_rx_Expect(&serial_packet);
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_DEVICE_END;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_device_rx_Expect(&serial_packet);
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /* Call serial process cmd and test with valid packets of CONFIG type */
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_CONFIG_START;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_config_rx_Expect(&serial_packet);
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_CONFIG_END;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_config_rx_Expect(&serial_packet);
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /* Call serial process cmd and test with valid packets of OPENMESH type */
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_OPENMESH_START;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_openmesh_rx_Expect(&serial_packet);
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_OPENMESH_END;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_openmesh_rx_Expect(&serial_packet);
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /* Call serial process cmd and test with valid packets of MESH type */
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_MESH_START;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_mesh_rx_Expect(&serial_packet);
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_MESH_END;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_mesh_rx_Expect(&serial_packet);
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /* Call serial process cmd and test with valid packets of PROV type */
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_PROV_START;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_prov_pkt_in_Expect(&serial_packet);
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_PROV_END;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_prov_pkt_in_Expect(&serial_packet);
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /* Call serial process cmd and test with valid packets of DFU type */
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_DFU_START;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_dfu_rx_Expect(&serial_packet);
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_DFU_END;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_dfu_rx_Expect(&serial_packet);
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /* Call serial process cmd and test with valid packets of APP type */
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_APP_START;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_app_rx_Expect(&serial_packet);
    serial_packet.opcode = SERIAL_OPCODE_CMD_RANGE_APP_END;
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    serial_handler_app_rx_Expect(&serial_packet);
    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    m_serial_process_cmd(NULL);

    /** Test the reception of invalid packets */
    serial_packet_t serial_packet2;
    serial_packet_t * p_packet = &serial_packet2;
    /* No opcodes between SERIAL_OPCODE_CMD_RANGE_MESH_END and SERIAL_OPCODE_CMD_RANGE_DFU_START are supported*/
    for (uint32_t i = SERIAL_OPCODE_CMD_RANGE_MESH_END+1; i < SERIAL_OPCODE_CMD_RANGE_DFU_START; ++i)
    {
        serial_packet.opcode = i;
        serial_bearer_packet_buffer_get_ExpectAndReturn(SERIAL_EVT_CMD_RSP_LEN_OVERHEAD, &p_packet, NRF_SUCCESS);
        serial_bearer_packet_buffer_get_IgnoreArg_pp_packet();
        serial_bearer_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
        serial_bearer_tx_Expect(p_packet);
        serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
        serial_bearer_rx_get_IgnoreArg_p_packet();
        serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    }

    /* No opcodes after SERIAL_OPCODE_CMD_RANGE_DFU_END are supported*/
    for (uint32_t i = SERIAL_OPCODE_CMD_RANGE_DFU_END+1; i <= UINT8_MAX; ++i)
    {
        serial_packet.opcode = i;
        serial_bearer_packet_buffer_get_ExpectAndReturn(SERIAL_EVT_CMD_RSP_LEN_OVERHEAD, &p_packet, NRF_SUCCESS);
        serial_bearer_packet_buffer_get_IgnoreArg_pp_packet();
        serial_bearer_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
        serial_bearer_tx_Expect(p_packet);
        serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
        serial_bearer_rx_get_IgnoreArg_p_packet();
        serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);
    }
    serial_packet.opcode = 0xFF;
    /* sending invalid data but not being able to get buffer will not result in a tx*/
    serial_bearer_packet_buffer_get_ExpectAndReturn(SERIAL_EVT_CMD_RSP_LEN_OVERHEAD, &p_packet, NRF_ERROR_NO_MEM);
    serial_bearer_packet_buffer_get_IgnoreArg_pp_packet();
    serial_bearer_packet_buffer_get_ReturnThruPtr_pp_packet(&p_packet);
    serial_bearer_rx_get_ExpectAndReturn(&serial_packet, true);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_bearer_rx_get_ReturnThruPtr_p_packet(&serial_packet);

    serial_bearer_rx_get_ExpectAndReturn(NULL, false);
    serial_bearer_rx_get_IgnoreArg_p_packet();
    serial_handler_device_alloc_fail_report_Expect();
    m_serial_process_cmd(NULL);


}
