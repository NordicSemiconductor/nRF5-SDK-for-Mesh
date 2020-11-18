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

#include <cmock.h>
#include <unity.h>

#include "mesh_stack.h"

#include "nrf_mesh_mock.h"
#include "nrf_mesh_externs_mock.h"
#include "device_state_manager_mock.h"
#include "access_mock.h"
#include "access_config_mock.h"
#include "net_state_mock.h"
#include "replay_cache_mock.h"
#include "flash_manager_mock.h"
#include "config_server_mock.h"
#include "health_server_mock.h"
#include "mesh_config_mock.h"
#include "hal_mock.h"
#include "nrf_mesh_events.h"
#include "mesh_config_backend_glue_mock.h"
#include "mesh_opt_mock.h"
#include "scanner_mock.h"
#include "bearer_handler_mock.h"
#include "event_mock.h"
#include "mesh_adv_mock.h"
#include "timer_scheduler_mock.h"
#include "proxy_mock.h"

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
static nrf_mesh_evt_handler_t * mp_power_down_cb;
static bool m_is_power_down_triggered;
static bool m_test_load_failed;

static void event_handler_add_cb(nrf_mesh_evt_handler_t * p_handler_params, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_NOT_NULL(p_handler_params);
    TEST_ASSERT_NOT_NULL(p_handler_params->evt_cb);
    mp_power_down_cb = p_handler_params;
}

static void event_handle_cb(const nrf_mesh_evt_t * p_evt, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_NOT_NULL(p_evt);
    TEST_ASSERT_EQUAL(NRF_MESH_EVT_READY_TO_POWER_OFF, p_evt->type);
    m_is_power_down_triggered = true;
}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    TEST_ASSERT_NULL(m_mesh_evt_cb);
    m_mesh_evt_cb = p_handler_params->evt_cb;
}

void nrf_mesh_evt_handler_remove(nrf_mesh_evt_handler_t * p_handler_params)
{
    TEST_ASSERT_EQUAL(m_mesh_evt_cb, p_handler_params->evt_cb);
    m_mesh_evt_cb = NULL;
}

static void mesh_config_load_stub_cb(int count)
{
    TEST_ASSERT_NOT_NULL(m_mesh_evt_cb);

    if (m_test_load_failed)
    {
        const nrf_mesh_evt_t load_failure_event = {
            .type = NRF_MESH_EVT_CONFIG_LOAD_FAILURE,
            .params.config_load_failure = {
                .p_data = NULL, /* don't care for test */
                .id.file = MESH_OPT_FIRST_FREE_ID - 1, /*lint !e64 Type mismatch */
                .data_len = 1, /* don't care for test */
                .reason = MESH_CONFIG_LOAD_FAILURE_INVALID_DATA /* don't care for test */
            }
        };

        m_mesh_evt_cb(&load_failure_event);
    }
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

static void successful_init_test(bool dsm_load_config_success, bool access_load_config_success, bool load_failed)
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
    dsm_load_config_apply_ExpectAndReturn(dsm_load_config_success ? NRF_SUCCESS : NRF_ERROR_INVALID_DATA);
    access_load_config_apply_ExpectAndReturn(access_load_config_success ? NRF_SUCCESS : NRF_ERROR_INVALID_DATA);
    m_test_load_failed = load_failed;
    mesh_config_load_StubWithCallback(mesh_config_load_stub_cb);
#if MESH_FEATURE_GATT_PROXY_ENABLED
    if (dsm_load_config_success && access_load_config_success && !load_failed)
    {
        proxy_init_Expect();
    }
#endif
    nrf_mesh_is_device_provisioned_ExpectAndReturn(dsm_load_config_success && access_load_config_success && !load_failed);

    if (dsm_load_config_success && access_load_config_success && !load_failed)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_stack_init(&init_params, &device_provisioned));
    }
    else
    {
        mesh_opt_clear_Expect();
        dsm_clear_Expect();
        access_clear_Expect();
        net_state_reset_Expect();
        replay_cache_clear_Expect();
        TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_DATA, mesh_stack_init(&init_params, &device_provisioned));
    }

    TEST_ASSERT_EQUAL(dsm_load_config_success && access_load_config_success && !load_failed, device_provisioned);

    mesh_stack_models_init_cb_Verify();
    device_state_manager_mock_Verify();
    access_mock_Verify();
}


/********** Test initialization and finalization **********/

void setUp(void)
{
    nrf_mesh_mock_Init();
    device_state_manager_mock_Init();
    access_mock_Init();
    access_config_mock_Init();
    net_state_mock_Init();
    replay_cache_mock_Init();
    flash_manager_mock_Init();
    config_server_mock_Init();
    health_server_mock_Init();
    hal_mock_Init();
    mesh_config_mock_Init();
    mesh_config_backend_glue_mock_Init();
    nrf_mesh_externs_mock_Init();
    mesh_opt_mock_Init();
    scanner_mock_Init();
    bearer_handler_mock_Init();
    event_mock_Init();
    mesh_adv_mock_Init();
    timer_scheduler_mock_Init();
    proxy_mock_Init();
    m_mesh_stack_models_init_cb_expected = false;
    m_mesh_evt_cb = NULL;
    m_test_load_failed = false;
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
    replay_cache_mock_Verify();
    replay_cache_mock_Destroy();
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    config_server_mock_Verify();
    config_server_mock_Destroy();
    health_server_mock_Verify();
    health_server_mock_Destroy();
    hal_mock_Verify();
    hal_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
    mesh_config_backend_glue_mock_Verify();
    mesh_config_backend_glue_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
    mesh_opt_mock_Verify();
    mesh_opt_mock_Destroy();
    scanner_mock_Verify();
    scanner_mock_Destroy();
    bearer_handler_mock_Verify();
    bearer_handler_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    mesh_adv_mock_Verify();
    mesh_adv_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    proxy_mock_Verify();
    proxy_mock_Destroy();

    mesh_stack_models_init_cb_Verify();
}

/********** Test cases **********/

void test_init(void)
{
    /* Test NULL pointer argument */
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, mesh_stack_init(NULL, NULL));

    /* Test normal initialization */
    mesh_opt_mock_Verify();
    successful_init_test(false, false, false);
    mesh_opt_mock_Verify();
    successful_init_test(true, false, false);
    mesh_opt_mock_Verify();
    successful_init_test(false, true, false);
    mesh_opt_mock_Verify();
    successful_init_test(true, true, false);
    mesh_opt_mock_Verify();
    /* Test load failure */
    successful_init_test(true, true, true);
}

void test_start(void)
{
    /* Test failing start */
#if MESH_FEATURE_GATT_PROXY_ENABLED
    nrf_mesh_is_device_provisioned_ExpectAndReturn(true);
    proxy_is_enabled_ExpectAndReturn(false);
#endif

    nrf_mesh_enable_ExpectAndReturn(NRF_ERROR_INVALID_STATE);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, mesh_stack_start());

    /* Test successful start */
#if MESH_FEATURE_GATT_PROXY_ENABLED
    nrf_mesh_is_device_provisioned_ExpectAndReturn(true);
    proxy_is_enabled_ExpectAndReturn(true);
    proxy_start_ExpectAndReturn(NRF_SUCCESS);
#endif

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
    uint16_t subnet_handle = 1;

    dsm_local_unicast_addresses_set_IgnoreAndReturn(NRF_SUCCESS);
    dsm_subnet_add_ExpectAndReturn(prov_data.netkey_index, prov_data.netkey, NULL, NRF_SUCCESS);
    dsm_subnet_add_IgnoreArg_p_subnet_handle();
    dsm_subnet_add_ReturnThruPtr_p_subnet_handle(&subnet_handle);
    dsm_devkey_add_ExpectAndReturn(prov_data.address, 0, devkey, NULL, NRF_SUCCESS);
    dsm_devkey_add_IgnoreArg_subnet_handle();
    dsm_devkey_add_IgnoreArg_p_devkey_handle();
    net_state_iv_index_set_ExpectAndReturn(prov_data.iv_index, prov_data.flags.iv_update, NRF_SUCCESS);
    dsm_subnet_update_ExpectAndReturn(subnet_handle, prov_data.netkey, NRF_SUCCESS);
    dsm_subnet_update_swap_keys_ExpectAndReturn(subnet_handle, NRF_SUCCESS);
    config_server_bind_IgnoreAndReturn(NRF_SUCCESS);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_stack_provisioning_data_store(&prov_data, devkey));
}

void test_mesh_stack_config_clear(void)
{
    mesh_opt_clear_Expect();
    access_clear_Expect();
    dsm_clear_Expect();
    net_state_reset_Expect();
    replay_cache_clear_Expect();

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

    flash_manager_is_stable_ExpectAndReturn(false);
    m_mesh_evt_cb(&evt);

    flash_manager_is_stable_ExpectAndReturn(true);
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

void test_mesh_stack_persistence_flash_usage(void)
{
#if PERSISTENT_STORAGE
    /* has mesh config: */
    mesh_config_backend_flash_usage_t mesh_config_usage = {
        (const uint32_t *) 1024,
        1024
    };
    mesh_config_backend_flash_usage_get_ExpectAnyArgs();
    mesh_config_backend_flash_usage_get_ReturnThruPtr_p_usage(&mesh_config_usage);
    flash_manager_recovery_page_get_ExpectAndReturn((void *) 4096);
    const uint32_t * p_start = 0;
    uint32_t length = 0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_stack_persistence_flash_usage(&p_start, &length));
    TEST_ASSERT_EQUAL(mesh_config_usage.p_start, p_start);
    TEST_ASSERT_EQUAL(4096 - 1024 + PAGE_SIZE, length);

    /* No mesh config */
    mesh_config_usage.p_start = NULL;
    mesh_config_usage.length = 0;
    mesh_config_backend_flash_usage_get_ExpectAnyArgs();
    mesh_config_backend_flash_usage_get_ReturnThruPtr_p_usage(&mesh_config_usage);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_stack_persistence_flash_usage(&p_start, &length));
    TEST_ASSERT_EQUAL(NULL, (uint32_t) p_start);
    TEST_ASSERT_EQUAL(0, length);
#endif
}


void test_mesh_stack_power_down(void)
{
    scanner_disable_Expect();
#if MESH_FEATURE_GATT_PROXY_ENABLED
    proxy_node_id_disable_ExpectAndReturn(NRF_SUCCESS);
    proxy_disconnect_Expect();
    proxy_stop_ExpectAndReturn(NRF_SUCCESS);
    proxy_disable_Expect();
#endif
    timer_sch_stop_Expect();
    event_handler_add_StubWithCallback(event_handler_add_cb);
    bearer_handler_force_mode_enable_Expect();
    mesh_config_power_down_Expect();
    mesh_stack_power_down();

    TEST_ASSERT_NOT_NULL(mp_power_down_cb);
    /* third party event. */
    nrf_mesh_evt_t evt = {.type = NRF_MESH_EVT_PROXY_STOPPED};
    mp_power_down_cb->evt_cb(&evt);

    /* mesh config completed deal with emergency cache. */
    event_handle_StubWithCallback(event_handle_cb);
    bearer_handler_force_mode_disable_Expect();
    nrf_mesh_disable_ExpectAndReturn(NRF_SUCCESS);
    evt.type = NRF_MESH_EVT_CONFIG_STABLE;
    mp_power_down_cb->evt_cb(&evt);
    TEST_ASSERT_FALSE(m_is_power_down_triggered);

    evt.type = NRF_MESH_EVT_DISABLED;
    mp_power_down_cb->evt_cb(&evt);
    TEST_ASSERT_TRUE(m_is_power_down_triggered);
}
