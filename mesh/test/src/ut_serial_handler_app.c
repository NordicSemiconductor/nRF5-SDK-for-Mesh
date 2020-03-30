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

#include "serial_handler_app.h"
#include "serial_status.h"

#include "serial_mock.h"

#define EXPECT_ACK_NO_TRANSLATE(_opcode, _serial_status)                \
    do {                                                                \
        serial_cmd_rsp_send_Expect(_opcode, _serial_status, NULL, 0); \
    } while (0)

static uint8_t *                              mp_app_data;
static uint32_t                               m_app_data_len;

static void serial_app_rx_cb(const uint8_t * p_data, uint32_t length)
{
    TEST_ASSERT_NOT_NULL(p_data);
    TEST_ASSERT_NOT_NULL(mp_app_data);
    TEST_ASSERT_EQUAL(m_app_data_len, length);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(mp_app_data, p_data, length);
}

/*****************************************************************************
 * Test initialization and finalization
 *****************************************************************************/

void setUp(void)
{
    serial_mock_Init();
}

void tearDown(void)
{
    serial_mock_Verify();
    serial_mock_Destroy();
}

/*****************************************************************************
 * Tests
 *****************************************************************************/

void test_app_rx(void)
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_APP_APPLICATION;
    cmd.length = 18;
    memset(cmd.payload.cmd.application.data, 0xAB, sizeof(cmd.payload.cmd.application.data));

    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_REJECTED);
    serial_handler_app_rx(&cmd);

    mp_app_data = cmd.payload.cmd.application.data;
    m_app_data_len = 17;
    serial_handler_app_cb_set(serial_app_rx_cb);
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_SUCCESS);
    serial_handler_app_rx(&cmd);

    mp_app_data = NULL;
    serial_handler_app_cb_set(NULL);
    EXPECT_ACK_NO_TRANSLATE(cmd.opcode, SERIAL_STATUS_ERROR_REJECTED);
    serial_handler_app_rx(&cmd);
}


