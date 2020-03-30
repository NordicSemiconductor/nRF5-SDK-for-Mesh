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

#include "serial_pb_remote_client.h"
#include "serial_status.h"
#include "serial_mock.h"
#include "access_config_mock.h"
#include "pb_remote_client_mock.h"
#include "serial_handler_models_mock.h"

#define NO_PROVISIONING_UNICORNS 2
#define MODEL_HANDLE_1 20
#define MODEL_HANDLE_2 30
#define MODEL_HANDLE_3 40
#define EXPECTED_ACCESS_PUBLISH_TTL 6

static serial_handler_models_model_init_cb_t    m_model_initialize;
static serial_handler_models_model_command_cb_t m_model_command;
static uint32_t m_serial_register_cb_count;
static uint32_t m_pbr_client_init_return_val;
static serial_cmd_prov_keypair_t m_my_key_pair;
static nrf_mesh_prov_ctx_t m_my_prov_unicorns[NO_PROVISIONING_UNICORNS];
static uint32_t m_next_available_handle;
struct
{
    pb_remote_client_t * p_client;
    uint16_t element_index;
    nrf_mesh_prov_ctx_t * p_prov_ctx;
    pb_remote_client_event_cb_t event_cb;
}m_last_init_call;

/* Reset for serial_pb_remote_client is not exposed via the header, since it's for unit testing only */
void serial_pb_remote_client_reset(void);

static uint32_t m_serial_model_register_cb(const serial_handler_models_info_t * p_model_info, int cmock_num_calls)
{
    m_model_initialize = p_model_info->model_initialize;
    m_model_command = p_model_info->model_command;
    m_serial_register_cb_count++;
    return NRF_SUCCESS;
}

static uint32_t m_pbr_client_init_cb(pb_remote_client_t *p_client, uint16_t element_index, nrf_mesh_prov_ctx_t * p_prov_ctx, pb_remote_client_event_cb_t event_cb, int cmock_num_calls)
{
    if (m_pbr_client_init_return_val == NRF_SUCCESS)
    {
        p_client->model_handle = m_next_available_handle;
        p_client->state = PB_REMOTE_CLIENT_STATE_IDLE;

        m_last_init_call.p_client = p_client;
        m_last_init_call.element_index = element_index;
        m_last_init_call.p_prov_ctx = p_prov_ctx;
        m_last_init_call.event_cb = event_cb;
    }

    return m_pbr_client_init_return_val;
}

void setUp(void)
{
    serial_mock_Init();
    access_config_mock_Init();
    pb_remote_client_mock_Init();
    serial_handler_models_mock_Init();
    m_serial_register_cb_count = 0;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    serial_handler_models_register_StubWithCallback(m_serial_model_register_cb);
    pb_remote_client_init_StubWithCallback(m_pbr_client_init_cb);
}

void tearDown(void)
{
    serial_mock_Verify();
    access_config_mock_Verify();
    pb_remote_client_mock_Verify();
    serial_handler_models_mock_Verify();
    serial_pb_remote_client_reset();
    serial_mock_Verify();
    serial_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    pb_remote_client_mock_Verify();
    pb_remote_client_mock_Destroy();
    serial_handler_models_mock_Verify();
    serial_handler_models_mock_Destroy();
}


/*****************************************************************************
* Tests
*****************************************************************************/
/* This test sets SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT to 2 */

void test_serial_pbr_client_invalid(void)
{
    serial_cmd_model_specific_command_t  cmd;
    serial_cmd_model_specific_init_t init_pbr;
    serial_evt_cmd_rsp_data_model_cmd_t cmd_rsp;

    TEST_NRF_MESH_ASSERT_EXPECT(serial_pb_remote_client_init(NULL, 1, &m_my_key_pair));
    TEST_NRF_MESH_ASSERT_EXPECT(serial_pb_remote_client_init(m_my_prov_unicorns, 0, &m_my_key_pair));
    TEST_NRF_MESH_ASSERT_EXPECT(serial_pb_remote_client_init(m_my_prov_unicorns, 1, &m_my_key_pair));
    TEST_NRF_MESH_ASSERT_EXPECT(serial_pb_remote_client_init(m_my_prov_unicorns, NO_PROVISIONING_UNICORNS, NULL));

    serial_pb_remote_client_init(m_my_prov_unicorns, NO_PROVISIONING_UNICORNS, &m_my_key_pair);
    TEST_ASSERT_EQUAL(1, m_serial_register_cb_count);

    /* Call serial_pbr_client_command without initializing a pbr_client instance */
    cmd.model_cmd_info.model_handle = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, m_model_command(&cmd, &cmd_rsp));
    TEST_ASSERT_EQUAL(0, cmd_rsp.data_len);

    /* Initialize a pbr client instance*/
    init_pbr.model_init_info.element_index = 10;
    m_next_available_handle = MODEL_HANDLE_1;
    access_model_handle_t model_handle;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_initialize(&init_pbr, &model_handle));
    TEST_ASSERT_EQUAL(init_pbr.model_init_info.element_index, m_last_init_call.element_index);
    TEST_ASSERT_EQUAL_PTR(&m_my_prov_unicorns[0], m_last_init_call.p_prov_ctx);

    /* Fail to initialize a second pbr client instance  due to pb_remote_client_init */
    init_pbr.model_init_info.element_index = 11;
    m_next_available_handle = MODEL_HANDLE_2;
    m_pbr_client_init_return_val = NRF_ERROR_INVALID_PARAM;
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_PARAM, m_model_initialize(&init_pbr, &model_handle));

    /* Second instance should stil be available since the previous call failed:
     * Initialize a second pbr client instance */
    init_pbr.model_init_info.element_index = 11;
    m_next_available_handle = MODEL_HANDLE_2;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_initialize(&init_pbr, &model_handle));
    TEST_ASSERT_EQUAL(init_pbr.model_init_info.element_index, m_last_init_call.element_index);
    TEST_ASSERT_EQUAL_PTR(&m_my_prov_unicorns[1], m_last_init_call.p_prov_ctx);

    /* Cannot initialize another instance since we have set  SERIAL_PB_REMOTE_CLIENT_INSTANCE_COUNT to 2 */
    init_pbr.model_init_info.element_index = 12;
    m_next_available_handle = MODEL_HANDLE_3;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, m_model_initialize(&init_pbr, &model_handle));
    TEST_ASSERT_NOT_EQUAL(init_pbr.model_init_info.element_index, m_last_init_call.element_index);
}

void test_serial_pbr_client_events(void)
{
    serial_cmd_model_specific_init_t init_pbr;

    serial_pb_remote_client_init(m_my_prov_unicorns, NO_PROVISIONING_UNICORNS, &m_my_key_pair);
    TEST_ASSERT_EQUAL(1, m_serial_register_cb_count);
    /* Initialize two pbr client instance*/
    init_pbr.model_init_info.element_index = 10;
    m_next_available_handle = MODEL_HANDLE_1;
    access_model_handle_t model_handle;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_initialize(&init_pbr, &model_handle));
    TEST_ASSERT_EQUAL(init_pbr.model_init_info.element_index, m_last_init_call.element_index);
    TEST_ASSERT_EQUAL_PTR(&m_my_prov_unicorns[0], m_last_init_call.p_prov_ctx);

    init_pbr.model_init_info.element_index = 11;
    m_next_available_handle = MODEL_HANDLE_2;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_initialize(&init_pbr, &model_handle));
    TEST_ASSERT_EQUAL(init_pbr.model_init_info.element_index, m_last_init_call.element_index);
    TEST_ASSERT_EQUAL_PTR(&m_my_prov_unicorns[1], m_last_init_call.p_prov_ctx);


    /* Send PB_REMOTE_EVENT_REMOTE_UUID event from the pbr_client */
    static const uint8_t m_dummy_uuid[NRF_MESH_UUID_SIZE] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
    serial_packet_t s_packet;
    serial_packet_t * p_s_packet = &s_packet;
    pb_remote_event_t pbr_event;
    pbr_event.type = PB_REMOTE_EVENT_REMOTE_UUID;
    pbr_event.remote_uuid.device_id =0xAF;
    pbr_event.remote_uuid.p_uuid = m_dummy_uuid;

    uint32_t serial_packet_len = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_model_specific_header_t) + sizeof(serial_pbr_client_remote_uuid_event_t);
    serial_packet_buffer_get_ExpectAndReturn(serial_packet_len, NULL, NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_s_packet);
    serial_tx_Expect(p_s_packet);
    m_last_init_call.event_cb(&pbr_event);
    TEST_ASSERT_EQUAL(SERIAL_OPCODE_EVT_MODEL_SPECIFIC, s_packet.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, s_packet.payload.evt.model.model_evt_info.model_id.company_id);
    TEST_ASSERT_EQUAL(PB_REMOTE_CLIENT_MODEL_ID, s_packet.payload.evt.model.model_evt_info.model_id.model_id);
    TEST_ASSERT_EQUAL(pbr_event.type , s_packet.payload.evt.model.model_evt_info.evt_type);
    TEST_ASSERT_EQUAL(pbr_event.remote_uuid.device_id , s_packet.payload.evt.model.data[0]);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(pbr_event.remote_uuid.p_uuid, &s_packet.payload.evt.model.data[1], NRF_MESH_UUID_SIZE);

    /* Send a different event from the pbr_client */
    pbr_event.type = PB_REMOTE_EVENT_TX_FAILED;
    serial_packet_len = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_evt_model_specific_header_t);
    serial_packet_buffer_get_ExpectAndReturn(serial_packet_len, NULL, NRF_SUCCESS);
    serial_packet_buffer_get_IgnoreArg_pp_packet();
    serial_packet_buffer_get_ReturnThruPtr_pp_packet(&p_s_packet);
    serial_tx_Expect(p_s_packet);
    m_last_init_call.event_cb(&pbr_event);
    TEST_ASSERT_EQUAL(SERIAL_OPCODE_EVT_MODEL_SPECIFIC, s_packet.opcode);
    TEST_ASSERT_EQUAL(ACCESS_COMPANY_ID_NONE, s_packet.payload.evt.model.model_evt_info.model_id.company_id);
    TEST_ASSERT_EQUAL(PB_REMOTE_CLIENT_MODEL_ID, s_packet.payload.evt.model.model_evt_info.model_id.model_id);
    TEST_ASSERT_EQUAL(pbr_event.type , s_packet.payload.evt.model.model_evt_info.evt_type);
}

void test_serial_pbr_client_commands(void)
{
    pb_remote_client_t * p_client_model_handle1;
    pb_remote_client_t * p_client_model_handle2;
    serial_cmd_model_specific_command_t  cmd;
    serial_cmd_model_specific_init_t init_pbr;
    serial_evt_cmd_rsp_data_model_cmd_t cmd_rsp;

    serial_pb_remote_client_init(m_my_prov_unicorns, NO_PROVISIONING_UNICORNS, &m_my_key_pair);
    TEST_ASSERT_EQUAL(1, m_serial_register_cb_count);
    /* Initialize two pbr client instance*/
    init_pbr.model_init_info.element_index = 10;
    m_next_available_handle = MODEL_HANDLE_1;
    access_model_handle_t model_handle;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_initialize(&init_pbr, &model_handle));
    TEST_ASSERT_EQUAL(init_pbr.model_init_info.element_index, m_last_init_call.element_index);
    TEST_ASSERT_EQUAL_PTR(&m_my_prov_unicorns[0], m_last_init_call.p_prov_ctx);
    p_client_model_handle1 = m_last_init_call.p_client;

    init_pbr.model_init_info.element_index = 11;
    m_next_available_handle = MODEL_HANDLE_2;
    m_pbr_client_init_return_val = NRF_SUCCESS;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_initialize(&init_pbr, &model_handle));
    TEST_ASSERT_EQUAL(init_pbr.model_init_info.element_index, m_last_init_call.element_index);
    TEST_ASSERT_EQUAL_PTR(&m_my_prov_unicorns[1], m_last_init_call.p_prov_ctx);
    p_client_model_handle2 = m_last_init_call.p_client;
    /* Call serial_pbr_client_command with SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_START on a valid instance */
    cmd.model_cmd_info.model_handle = MODEL_HANDLE_1;
    serial_pbr_client_command_data_t * p_pbr_command = (serial_pbr_client_command_data_t *)cmd.data;
    p_pbr_command->cmd_type = SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_START;
    pb_remote_client_remote_scan_start_ExpectAndReturn(p_client_model_handle1, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_command(&cmd, &cmd_rsp));
    TEST_ASSERT_EQUAL(0, cmd_rsp.data_len);

    /* Call serial_pbr_client_command with SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_CANCEL on a valid instance */
    cmd.model_cmd_info.model_handle = MODEL_HANDLE_2;
    p_pbr_command->cmd_type = SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_SCAN_CANCEL;
    pb_remote_client_remote_scan_cancel_ExpectAndReturn(p_client_model_handle2, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_command(&cmd, &cmd_rsp));
    TEST_ASSERT_EQUAL(0, cmd_rsp.data_len);

     /* Call serial_pbr_client_command with SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_PROVISION on a valid instance */
    cmd.model_cmd_info.model_handle = MODEL_HANDLE_1;
    p_pbr_command->cmd_type = SERIAL_PB_REMOTE_CLIENT_CMD_TYPE_PROVISION;
    /* Give some random values to the provisioning data */
    memset(&p_pbr_command->provisioning_data.prov_data, 0xBA, sizeof(nrf_mesh_prov_provisioning_data_t));
    p_pbr_command->provisioning_data.unprov_id = 2;
    memset(&p_pbr_command->provisioning_data.capabilities, 0xCE, sizeof(nrf_mesh_prov_oob_caps_t));
    pb_remote_client_remote_provision_ExpectWithArrayAndReturn(p_client_model_handle1, 1, &p_pbr_command->provisioning_data.prov_data, 1,
            m_my_key_pair.public_key, sizeof(m_my_key_pair.public_key), m_my_key_pair.private_key, sizeof(m_my_key_pair.private_key),
            &p_pbr_command->provisioning_data.capabilities, 1, p_pbr_command->provisioning_data.unprov_id, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_model_command(&cmd, &cmd_rsp));
    TEST_ASSERT_EQUAL(0, cmd_rsp.data_len);

    /* Call serial_pbr_client_command with an unrecognized command on a valid instance */
    cmd.model_cmd_info.model_handle = MODEL_HANDLE_2;
    p_pbr_command->cmd_type = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_SUPPORTED, m_model_command(&cmd, &cmd_rsp));
    TEST_ASSERT_EQUAL(0, cmd_rsp.data_len);

    /* Call serial_pbr_client_command with non-existent pbr_client instance */
    cmd.model_cmd_info.model_handle = 0;
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, m_model_command(&cmd, &cmd_rsp));
    TEST_ASSERT_EQUAL(0, cmd_rsp.data_len);
}
