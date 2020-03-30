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

#include "serial_handler_access.h"
#include "nrf_mesh_config_serial.h"
#include "serial_status.h"
#include "test_assert.h"

#include "serial_mock.h"
#include "access_config_mock.h"
#include "access_mock.h"

#define RX_PACK_INVALID_PACK_LENGTH(CMD, MIN, MAX)  do \
                                                    { \
                                                        CMD.length = (MIN)-1;   \
                                                        serial_cmd_rsp_send_Expect(CMD.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);   \
                                                        serial_handler_access_rx(&cmd); \
                                                        CMD.length = (MAX)+1;   \
                                                        serial_cmd_rsp_send_Expect(CMD.opcode, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, 0);   \
                                                        serial_handler_access_rx(&cmd); \
                                                    } while (0)
static void test_access_model_pub_addr_set()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_SET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t));

    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t);
    access_model_publish_address_set_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_model_publish_address_set_IgnoreArg_handle();
    access_model_publish_address_set_IgnoreArg_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_SET, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_publish_address_set_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_model_publish_address_set_IgnoreArg_handle();
    access_model_publish_address_set_IgnoreArg_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_SET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_pub_addr_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t));

    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t);
    access_model_publish_address_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_publish_address_get_IgnoreArg_handle();
    access_model_publish_address_get_IgnoreArg_p_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_pub_addr_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_publish_address_get_ExpectAndReturn(0, NULL, NRF_ERROR_NO_MEM);
    access_model_publish_address_get_IgnoreArg_handle();
    access_model_publish_address_get_IgnoreArg_p_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_ADDR_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_model_pub_period_set()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_SET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_pub_period_set_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_pub_period_set_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_pub_period_set_t);
    access_model_publish_period_set_ExpectAndReturn(0, (access_publish_resolution_t) 0, 0, NRF_SUCCESS);
    access_model_publish_period_set_IgnoreArg_resolution();
    access_model_publish_period_set_IgnoreArg_handle();
    access_model_publish_period_set_IgnoreArg_step_number();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_SET, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_publish_period_set_ExpectAndReturn(0, (access_publish_resolution_t) 0, 0, NRF_ERROR_NO_MEM);
    access_model_publish_period_set_IgnoreArg_resolution();
    access_model_publish_period_set_IgnoreArg_handle();
    access_model_publish_period_set_IgnoreArg_step_number();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_SET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_pub_period_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t);
    access_model_publish_period_get_ExpectAndReturn(0, NULL, NULL, NRF_SUCCESS);
    access_model_publish_period_get_IgnoreArg_handle();
    access_model_publish_period_get_IgnoreArg_p_resolution();
    access_model_publish_period_get_IgnoreArg_p_step_number();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_pub_period_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_publish_period_get_ExpectAndReturn(0, NULL, NULL, NRF_ERROR_NO_MEM);
    access_model_publish_period_get_IgnoreArg_handle();
    access_model_publish_period_get_IgnoreArg_p_resolution();
    access_model_publish_period_get_IgnoreArg_p_step_number();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_PERIOD_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_model_subs_add()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_ADD;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t);
    access_model_subscription_add_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_model_subscription_add_IgnoreArg_handle();
    access_model_subscription_add_IgnoreArg_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_ADD, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_subscription_add_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_model_subscription_add_IgnoreArg_handle();
    access_model_subscription_add_IgnoreArg_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_ADD, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_subs_remove()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_REMOVE;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t);
    access_model_subscription_remove_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_model_subscription_remove_IgnoreArg_handle();
    access_model_subscription_remove_IgnoreArg_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_REMOVE, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_subscription_remove_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_model_subscription_remove_IgnoreArg_handle();
    access_model_subscription_remove_IgnoreArg_address_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_REMOVE, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_subs_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t);
    access_model_subscriptions_get_ExpectAndReturn(0, NULL, NULL, NRF_SUCCESS);
    access_model_subscriptions_get_IgnoreArg_handle();
    access_model_subscriptions_get_IgnoreArg_p_count();
    access_model_subscriptions_get_IgnoreArg_p_address_handles();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_subs_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_subscriptions_get_ExpectAndReturn(0, NULL, NULL, NRF_ERROR_INVALID_LENGTH);
    access_model_subscriptions_get_IgnoreArg_handle();
    access_model_subscriptions_get_IgnoreArg_p_count();
    access_model_subscriptions_get_IgnoreArg_p_address_handles();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_INVALID_LENGTH, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_GET, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, sizeof(serial_evt_cmd_rsp_data_model_subs_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_subscriptions_get_ExpectAndReturn(0, NULL, NULL, NRF_ERROR_NO_MEM);
    access_model_subscriptions_get_IgnoreArg_handle();
    access_model_subscriptions_get_IgnoreArg_p_count();
    access_model_subscriptions_get_IgnoreArg_p_address_handles();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_SUBS_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_model_app_bind()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_BIND;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t);
    access_model_application_bind_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_model_application_bind_IgnoreArg_handle();
    access_model_application_bind_IgnoreArg_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_BIND, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_application_bind_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_model_application_bind_IgnoreArg_handle();
    access_model_application_bind_IgnoreArg_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_BIND, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_app_unbind()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_UNBIND;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t);
    access_model_application_unbind_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_model_application_unbind_IgnoreArg_handle();
    access_model_application_unbind_IgnoreArg_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_UNBIND, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_application_unbind_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_model_application_unbind_IgnoreArg_handle();
    access_model_application_unbind_IgnoreArg_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_UNBIND, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_app_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t);
    access_model_applications_get_ExpectAndReturn(0, NULL, NULL, NRF_SUCCESS);
    access_model_applications_get_IgnoreArg_handle();
    access_model_applications_get_IgnoreArg_p_count();
    access_model_applications_get_IgnoreArg_p_appkey_handles();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_apps_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_applications_get_ExpectAndReturn(0, NULL, NULL, NRF_ERROR_INVALID_LENGTH);
    access_model_applications_get_IgnoreArg_handle();
    access_model_applications_get_IgnoreArg_p_count();
    access_model_applications_get_IgnoreArg_p_appkey_handles();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_INVALID_LENGTH, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, sizeof(serial_evt_cmd_rsp_data_model_apps_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_applications_get_ExpectAndReturn(0, NULL, NULL, NRF_ERROR_NO_MEM);
    access_model_applications_get_IgnoreArg_handle();
    access_model_applications_get_IgnoreArg_p_count();
    access_model_applications_get_IgnoreArg_p_appkey_handles();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_APP_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_model_pub_app_set()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_SET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_pair_t);
    access_model_publish_application_set_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_model_publish_application_set_IgnoreArg_handle();
    access_model_publish_application_set_IgnoreArg_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_SET, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_publish_application_set_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_model_publish_application_set_IgnoreArg_handle();
    access_model_publish_application_set_IgnoreArg_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_SET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_pub_app_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t);
    access_model_publish_application_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_publish_application_get_IgnoreArg_handle();
    access_model_publish_application_get_IgnoreArg_p_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_pub_app_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_publish_application_get_ExpectAndReturn(0, NULL, NRF_ERROR_NO_MEM);
    access_model_publish_application_get_IgnoreArg_handle();
    access_model_publish_application_get_IgnoreArg_p_appkey_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_APP_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_model_pub_ttl_set()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_SET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_pub_ttl_set_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_pub_ttl_set_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_pub_ttl_set_t);
    access_model_publish_ttl_set_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_model_publish_ttl_set_IgnoreArg_handle();
    access_model_publish_ttl_set_IgnoreArg_ttl();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_SET, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_model_publish_ttl_set_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_model_publish_ttl_set_IgnoreArg_handle();
    access_model_publish_ttl_set_IgnoreArg_ttl();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_SET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_model_pub_ttl_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t);
    access_model_publish_ttl_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_publish_ttl_get_IgnoreArg_handle();
    access_model_publish_ttl_get_IgnoreArg_p_ttl();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_pub_ttl_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_publish_ttl_get_ExpectAndReturn(0, NULL, NRF_ERROR_NO_MEM);
    access_model_publish_ttl_get_IgnoreArg_handle();
    access_model_publish_ttl_get_IgnoreArg_p_ttl();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_PUB_TTL_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

}
static void test_access_elem_loc_set()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_SET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_loc_set_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_loc_set_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_loc_set_t);
    access_element_location_set_ExpectAndReturn(0, 0, NRF_SUCCESS);
    access_element_location_set_IgnoreArg_element_index();
    access_element_location_set_IgnoreArg_location();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_SET, SERIAL_STATUS_SUCCESS, NULL, 0);
    serial_handler_access_rx(&cmd);

    access_element_location_set_ExpectAndReturn(0, 0, NRF_ERROR_NO_MEM);
    access_element_location_set_IgnoreArg_element_index();
    access_element_location_set_IgnoreArg_location();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_SET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_handler_access_rx(&cmd);
}

static void test_access_elem_loc_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t);
    access_element_location_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_element_location_get_IgnoreArg_element_index();
    access_element_location_get_IgnoreArg_p_location();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_elem_loc_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_element_location_get_ExpectAndReturn(0, NULL, NRF_ERROR_NO_MEM);
    access_element_location_get_IgnoreArg_element_index();
    access_element_location_get_IgnoreArg_p_location();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_LOC_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_elem_sig_model_count_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_ELEM_SIG_MODEL_COUNT_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t);
    access_element_sig_model_count_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_element_sig_model_count_get_IgnoreArg_element_index();
    access_element_sig_model_count_get_IgnoreArg_p_sig_model_count();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_SIG_MODEL_COUNT_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_elem_model_count_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_element_sig_model_count_get_ExpectAndReturn(0, NULL, NRF_ERROR_NO_MEM);
    access_element_sig_model_count_get_IgnoreArg_element_index();
    access_element_sig_model_count_get_IgnoreArg_p_sig_model_count();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_SIG_MODEL_COUNT_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_elem_vendor_model_count_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_ELEM_VENDOR_MODEL_COUNT_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t);
    access_element_vendor_model_count_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_element_vendor_model_count_get_IgnoreArg_element_index();
    access_element_vendor_model_count_get_IgnoreArg_p_vendor_model_count();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_VENDOR_MODEL_COUNT_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_elem_model_count_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_element_vendor_model_count_get_ExpectAndReturn(0, NULL, NRF_ERROR_NO_MEM);
    access_element_vendor_model_count_get_IgnoreArg_element_index();
    access_element_vendor_model_count_get_IgnoreArg_p_vendor_model_count();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_VENDOR_MODEL_COUNT_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_model_id_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_MODEL_ID_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_model_handle_t);
    access_model_id_get_ExpectAndReturn(0, NULL, NRF_SUCCESS);
    access_model_id_get_IgnoreArg_handle();
    access_model_id_get_IgnoreArg_p_model_id();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_ID_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_id_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_model_id_get_ExpectAndReturn(0, NULL, NRF_ERROR_NO_MEM);
    access_model_id_get_IgnoreArg_handle();
    access_model_id_get_IgnoreArg_p_model_id();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_MODEL_ID_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);
}

static void test_access_handle_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_HANDLE_GET;
    access_model_id_t dummy = {};
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_get_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_get_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_handle_get_t);
    access_handle_get_ExpectAndReturn(0, dummy, NULL, NRF_SUCCESS);
    access_handle_get_IgnoreArg_model_id();
    access_handle_get_IgnoreArg_element_index();
    access_handle_get_IgnoreArg_p_handle();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_HANDLE_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_model_handle_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_handle_get_ExpectAndReturn(0, dummy, NULL, NRF_ERROR_NO_MEM);
    access_handle_get_IgnoreArg_model_id();
    access_handle_get_IgnoreArg_element_index();
    access_handle_get_IgnoreArg_p_handle();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_HANDLE_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

}
static void test_access_elem_models_get()
{
    serial_packet_t cmd;
    cmd.opcode = SERIAL_OPCODE_CMD_ACCESS_ELEM_MODELS_GET;
    RX_PACK_INVALID_PACK_LENGTH(cmd, SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t), SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t));
    cmd.length = SERIAL_PACKET_LENGTH_OVERHEAD + sizeof(serial_cmd_access_element_index_t);
    access_element_models_get_ExpectAndReturn(0, NULL, NULL, NRF_SUCCESS);
    access_element_models_get_IgnoreArg_element_index();
    access_element_models_get_IgnoreArg_p_count();
    access_element_models_get_IgnoreArg_p_models();
    serial_translate_error_ExpectAndReturn(NRF_SUCCESS, SERIAL_STATUS_SUCCESS);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_MODELS_GET, SERIAL_STATUS_SUCCESS, NULL, sizeof(serial_evt_cmd_rsp_data_elem_models_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_element_models_get_ExpectAndReturn(0, NULL, NULL, NRF_ERROR_INVALID_LENGTH);
    access_element_models_get_IgnoreArg_element_index();
    access_element_models_get_IgnoreArg_p_count();
    access_element_models_get_IgnoreArg_p_models();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_INVALID_LENGTH, SERIAL_STATUS_ERROR_INVALID_LENGTH);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_MODELS_GET, SERIAL_STATUS_ERROR_INVALID_LENGTH, NULL, sizeof(serial_evt_cmd_rsp_data_elem_models_get_t));
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

    access_element_models_get_ExpectAndReturn(0, NULL, NULL, NRF_ERROR_NO_MEM);
    access_element_models_get_IgnoreArg_element_index();
    access_element_models_get_IgnoreArg_p_count();
    access_element_models_get_IgnoreArg_p_models();
    serial_translate_error_ExpectAndReturn(NRF_ERROR_NO_MEM, SERIAL_STATUS_ERROR_REJECTED);
    serial_cmd_rsp_send_Expect(SERIAL_OPCODE_CMD_ACCESS_ELEM_MODELS_GET, SERIAL_STATUS_ERROR_REJECTED, NULL, 0);
    serial_cmd_rsp_send_IgnoreArg_p_data();
    serial_handler_access_rx(&cmd);

}

void setUp(void)
{
    serial_mock_Init();
    access_config_mock_Init();
    access_mock_Init();
}

void tearDown(void)
{
    serial_mock_Verify();
    access_config_mock_Verify();
    access_mock_Verify();
    serial_mock_Verify();
    serial_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    access_mock_Verify();
    access_mock_Destroy();
}

static void (*opcode_test_fp[])() ={test_access_model_pub_addr_set, test_access_model_pub_addr_get,
    test_access_model_pub_period_set, test_access_model_pub_period_get, test_access_model_subs_add,
    test_access_model_subs_remove, test_access_model_subs_get, test_access_model_app_bind,
    test_access_model_app_unbind, test_access_model_app_get, test_access_model_pub_app_set,
    test_access_model_pub_app_get, test_access_model_pub_ttl_set, test_access_model_pub_ttl_get,
    test_access_elem_loc_set, test_access_elem_loc_get, test_access_elem_sig_model_count_get,
    test_access_elem_vendor_model_count_get, test_access_model_id_get, test_access_handle_get,
    test_access_elem_models_get};
/*****************************************************************************
* Tests
*****************************************************************************/
void test_access_rx(void)
{
    /**************  INVALID OPCODES  **************/
    serial_packet_t cmd;
    /* < SERIAL_OPCODE_CMD_RANGE_ACCESS_START should cause in unknown cmd response */
    for (uint32_t i = 0; i < SERIAL_OPCODE_CMD_RANGE_ACCESS_START; ++i)
    {
        cmd.opcode = i;
        serial_cmd_rsp_send_Expect(i, SERIAL_STATUS_ERROR_CMD_UNKNOWN, NULL, 0);
        serial_handler_access_rx(&cmd);
    }
    /* > SERIAL_OPCODE_CMD_RANGE_ACCESS_END should cause an ASSERT */
    for (uint32_t i = SERIAL_OPCODE_CMD_RANGE_ACCESS_END+1; i <= UINT8_MAX; ++i)
    {
        cmd.opcode = i;
        TEST_NRF_MESH_ASSERT_EXPECT(serial_handler_access_rx(&cmd));
    }
    /* Check that we have a test for each command opcode */
    const uint8_t sizeof_fp = sizeof(void(*)());
    TEST_ASSERT_EQUAL_MESSAGE(SERIAL_OPCODE_CMD_RANGE_ACCESS_END - SERIAL_OPCODE_CMD_RANGE_ACCESS_START + 1 , sizeof(opcode_test_fp)/sizeof_fp, "Not all the serial access opcodes are tested, or the START and END values are incorrect.");

    /* Execute every test. */
    for (uint32_t i = SERIAL_OPCODE_CMD_RANGE_ACCESS_START; i <= SERIAL_OPCODE_CMD_RANGE_ACCESS_END ; ++i)
    {
        opcode_test_fp[i-SERIAL_OPCODE_CMD_RANGE_ACCESS_START]();
    }
}
