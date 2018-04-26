/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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

#include <cmock.h>
#include <unity.h>

#include "mesh_stack.h"

#include "nrf_mesh_mock.h"
#include "device_state_manager_mock.h"
#include "access_mock.h"
#include "access_config_mock.h"
#include "net_state_mock.h"
#include "flash_manager_mock.h"
#include "config_server_mock.h"
#include "health_server_mock.h"
#include "hal_mock.h"
#include "nrf_mesh_events.h"

/********** Additional mock functions **********/

#define DEVKEY          { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 }
#define NETKEY          { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16 }
#define IV_INDEX        0xAA
#define NETKEY_INDEX    0x55
#define ADDRESS         0xAD
#define IF_UPDATE       true
#define KEY_REFRESH     true

static bool m_mesh_stack_models_init_cb_expected;

static nrf_mesh_evt_handler_cb_t m_mesh_evt_cb;

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    TEST_ASSERT_NULL(m_mesh_evt_cb);
    m_mesh_evt_cb = p_handler_params->evt_cb;
}

static void config_server_evt_cb(const config_server_evt_t * p_evt)
{
    // Do nothing
}

static void health_server_attention_cb(const health_server_t * p_server, bool attention_state)
{
    // Do nothing
}

static void mesh_stack_models_init_cb(void)
{
    TEST_ASSERT_TRUE(m_mesh_stack_models_init_cb_expected);
    m_mesh_stack_models_init_cb_expected = false;
}

static void mesh_stack_models_init_cb_Verify(void)
{
    TEST_ASSERT_FALSE(m_mesh_stack_models_init_cb_expected);
}

static void successful_init_test(bool dsm_flash_config_load_return, bool access_flash_config_load_return)
{
    health_server_selftest_t test_array[] = {{ .test_id = 1, .selftest_function = NULL }};
    bool device_provisioned;
    mesh_stack_init_params_t init_params;

    memset(&init_params, 0, sizeof(init_params));

    init_params.models.config_server_cb               = config_server_evt_cb;
    init_params.models.health_server_attention_cb     = health_server_attention_cb;
    init_params.models.p_health_server_selftest_array = test_array;
    init_params.models.health_server_num_selftests    = sizeof(test_array) / sizeof(test_array[0]);
    init_params.models.models_init_cb                 = mesh_stack_models_init_cb;

    m_mesh_stack_models_init_cb_expected = true;

    nrf_mesh_init_ExpectAndReturn(NULL, NRF_SUCCESS);
    nrf_mesh_init_IgnoreArg_p_init_params();
    access_init_Expect();
    dsm_init_Expect();
    config_server_init_ExpectAndReturn(config_server_evt_cb, NRF_SUCCESS);
    health_server_init_ExpectAndReturn(NULL, 0, DEVICE_COMPANY_ID, health_server_attention_cb, test_array, sizeof(test_array) / sizeof(test_array[0]), NRF_SUCCESS);
    health_server_init_IgnoreArg_p_server();
    dsm_flash_config_load_ExpectAndReturn(dsm_flash_config_load_return);
    access_flash_config_load_ExpectAndReturn(access_flash_config_load_return);
    dsm_local_unicast_addresses_get_Expect(NULL);
    dsm_local_unicast_addresses_get_IgnoreArg_p_address();

    dsm_local_unicast_address_t addresses = {.address_start = NRF_MESH_ADDR_UNASSIGNED, .count = 0};
    if (dsm_flash_config_load_return)
    {
        addresses.address_start = 1;
        addresses.count = 1;
    }
    dsm_local_unicast_addresses_get_ReturnThruPtr_p_address(&addresses);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_stack_init(&init_params, &device_provisioned));

    TEST_ASSERT_EQUAL(addresses.address_start != NRF_MESH_ADDR_UNASSIGNED, device_provisioned);
    mesh_stack_models_init_cb_Verify();
}


/********** Test initialization and finalization **********/

void setUp(void)
{
    nrf_mesh_mock_Init();
    device_state_manager_mock_Init();
    access_mock_Init();
    access_config_mock_Init();
    net_state_mock_Init();
    flash_manager_mock_Init();
    config_server_mock_Init();
    health_server_mock_Init();
    hal_mock_Init();
    m_mesh_stack_models_init_cb_expected = false;
    m_mesh_evt_cb = NULL;
}

void tearDown(void)
{
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    device_state_manager_mock_Verify();
    device_state_manager_mock_Destroy();
    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    config_server_mock_Verify();
    config_server_mock_Destroy();
    health_server_mock_Verify();
    health_server_mock_Destroy();
    hal_mock_Verify();
    hal_mock_Destroy();

    mesh_stack_models_init_cb_Verify();
}

/********** Test cases **********/

void test_init(void)
{
    /* Test NULL pointer argument */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, mesh_stack_init(NULL, NULL));

    /* Test normal initialization */
    successful_init_test(false, false);
    successful_init_test(true, false);
    successful_init_test(false, true);
    successful_init_test(true, true);
}

void test_start(void)
{
    /* Test failing start */
    nrf_mesh_enable_ExpectAndReturn(NRF_ERROR_INVALID_STATE);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, mesh_stack_start());

    /* Test successful start */
    nrf_mesh_enable_ExpectAndReturn(NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_stack_start());
}

void test_provisioning_data_store(void)
{
    nrf_mesh_prov_provisioning_data_t prov_data =
    {
        .netkey            = NETKEY,
        .netkey_index      = NETKEY_INDEX,
        .iv_index          = IV_INDEX,
        .address           = ADDRESS,
        .flags.iv_update   = IF_UPDATE,
        .flags.key_refresh = KEY_REFRESH
    };
    uint8_t devkey[] = DEVKEY;

    dsm_local_unicast_addresses_set_IgnoreAndReturn(NRF_SUCCESS);
    dsm_subnet_add_ExpectAndReturn(prov_data.netkey_index, prov_data.netkey, NULL, NRF_SUCCESS);
    dsm_subnet_add_IgnoreArg_p_subnet_handle();
    dsm_devkey_add_ExpectAndReturn(prov_data.address, 0, devkey, NULL, NRF_SUCCESS);
    dsm_devkey_add_IgnoreArg_subnet_handle();
    dsm_devkey_add_IgnoreArg_p_devkey_handle();
    net_state_iv_index_set_ExpectAndReturn(prov_data.iv_index, prov_data.flags.iv_update, NRF_SUCCESS);
    config_server_bind_IgnoreAndReturn(NRF_SUCCESS);
    access_flash_config_store_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_stack_provisioning_data_store(&prov_data, devkey));
}

void test_mesh_stack_config_clear(void)
{
    access_clear_Expect();
    dsm_clear_Expect();
    net_state_reset_Expect();

    mesh_stack_config_clear();
}

void test_mesh_stack_device_reset_flash_not_stable(void)
{
#if PERSISTENT_STORAGE
    flash_manager_is_stable_ExpectAndReturn(false);
    mesh_stack_device_reset();
    TEST_ASSERT_NOT_NULL(m_mesh_evt_cb);
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_FLASH_STABLE;

    hal_device_reset_Expect(0);
    m_mesh_evt_cb(&evt);
#endif
}

void test_mesh_stack_device_reset_flash_is_stable(void)
{
#if PERSISTENT_STORAGE
    flash_manager_is_stable_ExpectAndReturn(true);
    hal_device_reset_Expect(0);
    mesh_stack_device_reset();
#endif
}


void test_mesh_stack_device_reset_no_persistent_storage(void)
{
#if !PERSISTENT_STORAGE
    hal_device_reset_Expect(0);
    mesh_stack_device_reset();
#endif
}
