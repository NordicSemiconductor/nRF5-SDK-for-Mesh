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

#include "serial_handler_models.h"
#include "nrf_mesh_config_serial.h"
#include "serial_status.h"
#include "test_assert.h"

#include "serial_mock.h"
#include "access_config_mock.h"

#define RX_PACK_INVALID_PACK_LENGTH(CMD, MIN, MAX)  do \
                                                    { \
                                                        CMD.length = (MIN)-1;   \
                                                        serial_cmd_rsp_send_Expect(CMD.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);   \
                                                        serial_handler_models_rx(&cmd); \
                                                        CMD.length = MAX < NRF_MESH_SERIAL_PAYLOAD_MAXLEN ? (MAX)+1 : 0; \
                                                        serial_cmd_rsp_send_Expect(CMD.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);   \
                                                        serial_handler_models_rx(&cmd); \
                                                    } while (0)

static uint32_t m_init1_calls_count;
static uint32_t m_cmd1_calls_count;
static uint32_t m_init2_calls_count;
static uint32_t m_cmd2_calls_count;
static serial_evt_cmd_rsp_data_model_cmd_t m_command1_cmd_response;
static serial_evt_cmd_rsp_data_model_cmd_t m_command2_cmd_response;
static const access_model_handle_t m_model1_handle = 0xD05E;
static const access_model_handle_t m_model2_handle = 0xD0FF;

/* Reset for serial_handler_models is not exposed via the header, since it's for unit testing only */
void serial_handler_models_reset(void);

void setUp(void)
{
    serial_mock_Init();
    access_config_mock_Init();
    m_init1_calls_count = 0;
    m_cmd1_calls_count  = 0;
    m_init2_calls_count = 0;
    m_cmd2_calls_count  = 0;

    m_command1_cmd_response.data_len = sizeof(serial_evt_cmd_rsp_data_model_cmd_t) - 1;
    m_command2_cmd_response.data_len = sizeof(serial_evt_cmd_rsp_data_model_cmd_t) - 1;

    for (uint32_t i = 1; i < sizeof(serial_evt_cmd_rsp_data_model_cmd_t); ++i)
    {
        m_command1_cmd_response.data[i-1] = i;
        m_command2_cmd_response.data[i-1] = sizeof(serial_evt_cmd_rsp_data_model_cmd_t) - i;
    }
    /** RESET **/
    serial_handler_models_reset();
}

void tearDown(void)
{
    serial_mock_Verify();
    access_config_mock_Verify();
    serial_mock_Verify();
    serial_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
}

static uint32_t init_my_first_model(const serial_cmd_model_specific_init_t * p_init_params, access_model_handle_t * p_model_handle)
{
    m_init1_calls_count++;
    *p_model_handle = m_model1_handle;
    return NRF_SUCCESS;
}

static uint32_t command_my_first_model(const  serial_cmd_model_specific_command_t * p_command_params, serial_evt_cmd_rsp_data_model_cmd_t * p_cmd_rsp)
{
    m_cmd1_calls_count++;
    memcpy(p_cmd_rsp, &m_command1_cmd_response, sizeof(serial_evt_cmd_rsp_data_model_cmd_t));
    return NRF_SUCCESS;
}

static uint32_t init_my_second_model(const serial_cmd_model_specific_init_t * p_init_params, access_model_handle_t * p_model_handle)
{
    m_init2_calls_count++;
    *p_model_handle = m_model2_handle;
    return NRF_SUCCESS;
}

static uint32_t command_my_second_model(const  serial_cmd_model_specific_command_t * p_command_params, serial_evt_cmd_rsp_data_model_cmd_t * p_cmd_rsp)
{
    m_cmd2_calls_count++;
    memcpy(p_cmd_rsp, &m_command2_cmd_response, sizeof(serial_evt_cmd_rsp_data_model_cmd_t));
    return NRF_SUCCESS;
}

/*****************************************************************************
* Tests
*****************************************************************************/
void test_models_invalid(void)
{
    serial_handler_models_info_t model_info;
    model_info.model_id.company_id = 0;
    model_info.model_id.model_id = 1;
    model_info.model_initialize = init_my_first_model;
    model_info.model_command = NULL;
    /**  INVALID  **/
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, serial_handler_models_register(NULL));
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, serial_handler_models_register(&model_info));
    model_info.model_initialize = NULL;
    model_info.model_command = command_my_first_model;
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, serial_handler_models_register(&model_info));
    model_info.model_initialize = init_my_first_model;
    model_info.model_command = command_my_first_model;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_handler_models_register(&model_info));
    /* No guards against registering the same model twice, perhaps there should be? */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_handler_models_register(&model_info));
    /* ACCESS_MODEL_COUNT is set to 2 by this test */
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, serial_handler_models_register(&model_info));

    /** RESET **/
    serial_handler_models_reset();
    /** RX **/
    serial_packet_t cmd;
    /* < SERIAL_OPCODE_CMD_RANGE_MODEL_SPECIFIC_START should cause in unknown cmd response */
    for (uint32_t i = 0; i < SERIAL_OPCODE_CMD_RANGE_MODEL_SPECIFIC_START; ++i)
    {
        cmd.opcode = i;
        serial_cmd_rsp_send_Expect(i, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
        serial_handler_models_rx(&cmd);
    }
    /* > SERIAL_OPCODE_CMD_RANGE_MODEL_SPECIFIC_END should cause an ASSERT */
    for (uint32_t i = SERIAL_OPCODE_CMD_RANGE_MODEL_SPECIFIC_END+1; i <= UINT8_MAX; ++i)
    {
        cmd.opcode = i;
        TEST_NRF_MESH_ASSERT_EXPECT(serial_handler_models_rx(&cmd));
    }
}

void test_models_rx(void)
{
    access_model_id_t model1 = {.company_id = 0, .model_id = 0};
    access_model_id_t model2 = {.company_id = 3, .model_id = 2};
    serial_handler_models_info_t model_info;
    model_info.model_id.company_id = model1.company_id;
    model_info.model_id.model_id = model1.model_id;
    model_info.model_initialize = init_my_first_model;
    model_info.model_command = command_my_first_model;
    serial_packet_t cmd = {0};
    /** RX **/
    /* Testing models_get when no models have been registered */
    cmd.opcode = SERIAL_OPCODE_CMD_MODEL_SPECIFIC_MODELS_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD, SERIAL_PACKET_LENGTH_OVERHEAD);
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_MODELS_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(uint16_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_models_rx(&cmd);

    /** REGISTER **/
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_handler_models_register(&model_info));
    model_info.model_id.company_id = model2.company_id;
    model_info.model_id.model_id = model2.model_id;
    model_info.model_initialize = init_my_second_model;
    model_info.model_command = command_my_second_model;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, serial_handler_models_register(&model_info));

    /* Testing models_get when 2 models have been registered */
    serial_evt_cmd_rsp_data_models_get_t models;
    models.count = 2;
    models.model_ids[0].company_id = model1.company_id;
    models.model_ids[0].model_id = model1.model_id;
    models.model_ids[1].company_id = model2.company_id;
    models.model_ids[1].model_id = model2.model_id;
    cmd.opcode = SERIAL_OPCODE_CMD_MODEL_SPECIFIC_MODELS_GET;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD;
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_MODELS_GET, SERIAL_STATUS_SUCCESS, (uint8_t *) &models,
        sizeof(models.count) + models.count * sizeof(access_model_id_t),
        sizeof(models.count) + models.count * sizeof(access_model_id_t));
    serial_handler_models_rx(&cmd);

    /** INIT **/
    cmd.opcode = SERIAL_OPCODE_CMD_MODEL_SPECIFIC_INIT;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_model_specific_init_header_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_model_specific_init_t));
    /* Attempt with a non-existent model_id */
    cmd.payload.cmd.access.model_init.model_init_info.model_id.company_id = 0xAAAA; /* Some none existing value */
    cmd.payload.cmd.access.model_init.model_init_info.model_id.model_id = 0;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD +sizeof(serial_cmd_model_specific_init_t);
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NOT_FOUND, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_INIT, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(0, m_init1_calls_count);
    TEST_ASSERT_EQUAL(0, m_init2_calls_count);

    /* Attempt with the first model */
    cmd.payload.cmd.access.model_init.model_init_info.model_id.company_id = model1.company_id;
    cmd.payload.cmd.access.model_init.model_init_info.model_id.model_id = model1.model_id;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_model_specific_init_t);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_INIT,
        SERIAL_STATUS_SUCCESS, (uint8_t *)&m_model1_handle, sizeof(m_model1_handle), sizeof(serial_evt_cmd_rsp_data_model_init_t));
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(1, m_init1_calls_count);
    TEST_ASSERT_EQUAL(0, m_init2_calls_count);

    /* Attempt with the second model */
    cmd.payload.cmd.access.model_init.model_init_info.model_id.company_id = model2.company_id;
    cmd.payload.cmd.access.model_init.model_init_info.model_id.model_id = model2.model_id;
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_model_specific_init_t);
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_INIT,
        SERIAL_STATUS_SUCCESS, (uint8_t *)&m_model2_handle, sizeof(m_model2_handle), sizeof(serial_evt_cmd_rsp_data_model_init_t));
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(1, m_init1_calls_count);
    TEST_ASSERT_EQUAL(1, m_init2_calls_count);

    /** COMMAND **/
    cmd.opcode = SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND;
    cmd.payload.cmd.access.model_cmd.model_cmd_info.model_handle = 0;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_model_specific_command_header_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_model_specific_command_t));

    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD +sizeof(serial_cmd_model_specific_command_t);
    /* Attempt with a non-existent model_id */
    access_model_id_get_ExpectAndReturn(0, NULL, NRF_ERROR_NOT_FOUND);
    access_model_id_get_IgnoreArg_p_model_id();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NOT_FOUND, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(0, m_cmd1_calls_count);
    TEST_ASSERT_EQUAL(0, m_cmd2_calls_count);

    /* Attempt with the first model */
    access_model_id_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_id_get_ReturnThruPtr_p_model_id(&model1);
    access_model_id_get_IgnoreArg_p_model_id();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND, SERIAL_STATUS_SUCCESS, (uint8_t *) &m_command1_cmd_response, sizeof(m_command1_cmd_response),sizeof(m_command1_cmd_response));
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(1, m_cmd1_calls_count);
    TEST_ASSERT_EQUAL(0, m_cmd2_calls_count);

    /* Attempt 0 len response with the first model */
    m_command1_cmd_response.data_len = 0;
    access_model_id_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_id_get_ReturnThruPtr_p_model_id(&model1);
    access_model_id_get_IgnoreArg_p_model_id();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(2, m_cmd1_calls_count);
    TEST_ASSERT_EQUAL(0, m_cmd2_calls_count);

    /* Attempt with the second model */
    access_model_id_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_id_get_ReturnThruPtr_p_model_id(&model2);
    access_model_id_get_IgnoreArg_p_model_id();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_ExpectWithArray(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND, SERIAL_STATUS_SUCCESS,
        (uint8_t *) &m_command2_cmd_response, sizeof(m_command2_cmd_response),sizeof(m_command2_cmd_response));
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(2, m_cmd1_calls_count);
    TEST_ASSERT_EQUAL(1, m_cmd2_calls_count);

    /* Attempt 0 len response with the second model */
    m_command2_cmd_response.data_len = 0;
    access_model_id_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_id_get_ReturnThruPtr_p_model_id(&model2);
    access_model_id_get_IgnoreArg_p_model_id();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_MODEL_SPECIFIC_COMMAND, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_models_rx(&cmd);
    TEST_ASSERT_EQUAL(2, m_cmd1_calls_count);
    TEST_ASSERT_EQUAL(2, m_cmd2_calls_count);
}
