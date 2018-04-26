/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#include <stdbool.h>
#include <string.h>

#include <cmock.h>
#include <unity.h>

#include "nrf_mesh_node_config.h"
#include "nrf_mesh_prov_bearer.h"

#include "access_mock.h"
#include "access_config_mock.h"
#include "flash_manager_mock.h"
#include "config_server_mock.h"
#include "device_state_manager_mock.h"
#include "health_server_mock.h"
#include "net_state_mock.h"
#include "nrf_mesh_mock.h"
#include "nrf_mesh_events_mock.h"
#include "nrf_mesh_prov_mock.h"
#include "nrf_mesh_prov_bearer_adv_mock.h"

/********** Additional mock functions **********/

#define EXPECT_COMPLETE_CALLBACK(p_data) \
    do { \
        m_complete_callback_expected = true; \
        mp_complete_callback_expected_data = p_data; \
    } while (0)

static bool m_complete_callback_expected;
static void * mp_complete_callback_expected_data;
static config_server_evt_cb_t mp_config_server_evt_callback;
static nrf_mesh_prov_evt_handler_cb_t m_event_handler;
static const nrf_mesh_prov_oob_caps_t * mp_expected_caps;
static const config_server_evt_t * mp_expected_config_server_evt;

static void complete_callback(void * p_data)
{
    TEST_ASSERT_TRUE(m_complete_callback_expected);
    m_complete_callback_expected = false;
    TEST_ASSERT_EQUAL_PTR(mp_complete_callback_expected_data, p_data);
}

static void nrf_mesh_prov_init_ExpectArg_p_caps(const nrf_mesh_prov_oob_caps_t * p_caps)
{
    mp_expected_caps = p_caps;
}

static uint32_t nrf_mesh_prov_init_stub(nrf_mesh_prov_ctx_t *            p_ctx,
                                        const uint8_t *                  p_public_key,
                                        const uint8_t *                  p_private_key,
                                        const nrf_mesh_prov_oob_caps_t * p_caps,
                                        nrf_mesh_prov_evt_handler_cb_t   event_handler,
                                        int call_count)
{
    TEST_ASSERT_NOT_NULL(p_ctx);
    TEST_ASSERT_NOT_NULL(p_public_key);
    TEST_ASSERT_NOT_NULL(p_private_key);
    if (mp_expected_caps)
    {
        TEST_ASSERT_EQUAL_MEMORY_MESSAGE(mp_expected_caps, p_caps, sizeof(nrf_mesh_prov_oob_caps_t),
                                         "The expected provisioning capabilities did not match");
    }
    else
    {
        TEST_ASSERT_NOT_NULL(p_caps);
    }
    TEST_ASSERT_NOT_NULL(event_handler);

    m_event_handler = event_handler;

    /* De-register */
    nrf_mesh_prov_init_StubWithCallback(NULL);
    return NRF_SUCCESS;
}

uint32_t sd_nvic_EnableIRQ(IRQn_Type IRQn)
{
    return NRF_SUCCESS;
}

uint32_t sd_softdevice_is_enabled(uint8_t * p_is_enabled)
{
    *p_is_enabled = 1;
    return NRF_SUCCESS;
}

void hal_device_reset(uint8_t reset_reason)
{
}

static void dummy_attention_cb(const health_server_t * p_server, bool enable)
{

}

static void config_server_evt_Verify(void)
{
    if (mp_expected_config_server_evt != NULL)
    {
        char msg[64];
        (void) sprintf(msg, "Config server event %d expected, but not handled", mp_expected_config_server_evt->type);
        TEST_FAIL_MESSAGE(msg);
    }
}

static void config_server_evt_Expect(const config_server_evt_t * p_evt)
{
    /* Verify that we're not overwriting the event here. */
    config_server_evt_Verify();
    mp_expected_config_server_evt = p_evt;
}

static void config_server_callback(const config_server_evt_t * p_evt)
{
    TEST_ASSERT_NOT_NULL(p_evt);
    if (mp_expected_config_server_evt == NULL)
    {
        char msg[64];
        (void) sprintf(msg, "Unexpected config server event: %d", p_evt->type);
        TEST_FAIL_MESSAGE(msg);
    }
    else
    {
        TEST_ASSERT_EQUAL_MEMORY(mp_expected_config_server_evt, p_evt, sizeof(config_server_evt_t));
        mp_expected_config_server_evt = NULL; /* Clear the event */
    }
}

static uint32_t config_server_init_stub(config_server_evt_cb_t evt_cb, int num_calls)
{
    TEST_ASSERT_EQUAL(0, num_calls);
    mp_config_server_evt_callback = evt_cb;
    return NRF_SUCCESS;
}

/********** Test initialization and finalization **********/

void setUp(void)
{
    access_mock_Init();
    access_config_mock_Init();
    flash_manager_mock_Init();
    config_server_mock_Init();
    device_state_manager_mock_Init();
    health_server_mock_Init();
    net_state_mock_Init();
    nrf_mesh_mock_Init();
    nrf_mesh_events_mock_Init();
    nrf_mesh_prov_mock_Init();
    nrf_mesh_prov_bearer_adv_mock_Init();

    mp_expected_caps = NULL;
    m_event_handler = NULL;
    mp_config_server_evt_callback = NULL;
}

void tearDown(void)
{
    access_mock_Verify();
    access_mock_Destroy();
    access_config_mock_Verify();
    access_config_mock_Destroy();
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
    config_server_mock_Verify();
    config_server_mock_Destroy();
    device_state_manager_mock_Verify();
    device_state_manager_mock_Destroy();
    health_server_mock_Verify();
    health_server_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    nrf_mesh_prov_mock_Verify();
    nrf_mesh_prov_mock_Destroy();
    nrf_mesh_prov_bearer_adv_mock_Verify();
    nrf_mesh_prov_bearer_adv_mock_Destroy();
    config_server_evt_Verify();
}

/********** Test cases **********/

void test_basic_configuration(void)
{
    void * test_data = (void *) 0x10ff10ff;
    nrf_mesh_node_config_params_t config_params;
    memset(&config_params, 0, sizeof(nrf_mesh_node_config_params_t));
    config_params.prov_caps.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    config_params.prov_caps.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    config_params.prov_caps.num_elements = 1;
    config_params.p_data = test_data;
    config_params.mesh_assertion_handler = m_assertion_handler;
    config_params.complete_callback = complete_callback;
    config_params.attention_cb = dummy_attention_cb;

    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, nrf_mesh_node_config(NULL));

    nrf_mesh_init_ExpectAndReturn(NULL, NRF_SUCCESS);
    nrf_mesh_init_IgnoreArg_p_init_params();

    m_event_handler = NULL;

    nrf_mesh_enable_ExpectAndReturn(NRF_SUCCESS);
    access_init_Expect();
    dsm_init_Expect();
    config_server_init_IgnoreAndReturn(NRF_SUCCESS);
    health_server_init_ExpectAndReturn(NULL, 0, DEVICE_COMPANY_ID, dummy_attention_cb, NULL, 0, NRF_SUCCESS);
    health_server_init_IgnoreArg_p_server();

    /* Not pre-configured */
    dsm_flash_config_load_ExpectAndReturn(false);

    nrf_mesh_prov_generate_keys_ExpectAndReturn(NULL, NULL, NRF_SUCCESS);
    nrf_mesh_prov_generate_keys_IgnoreArg_p_public();
    nrf_mesh_prov_generate_keys_IgnoreArg_p_private();

    nrf_mesh_prov_init_ExpectArg_p_caps(&config_params.prov_caps);
    nrf_mesh_prov_init_StubWithCallback(nrf_mesh_prov_init_stub);

    prov_bearer_t * p_bearer = (prov_bearer_t *) 0xCAFEBABE;
    nrf_mesh_prov_bearer_adv_interface_get_IgnoreAndReturn(p_bearer);
    nrf_mesh_prov_bearer_add_ExpectAndReturn(NULL, p_bearer, NRF_SUCCESS);
    nrf_mesh_prov_bearer_add_IgnoreArg_p_ctx();

    nrf_mesh_prov_listen_ExpectAndReturn(NULL, config_params.p_device_uri, config_params.oob_info_sources, NRF_MESH_PROV_BEARER_ADV, NRF_SUCCESS);
    nrf_mesh_prov_listen_IgnoreArg_p_ctx();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_node_config(&config_params));
    TEST_ASSERT_NOT_NULL(m_event_handler);

    /* Go through the events of a normal provisioning procedure: */
    nrf_mesh_prov_evt_t event;
    nrf_mesh_prov_ctx_t prov_context = {};

    /* The link gets established: */
    memset(&event, 0, sizeof(nrf_mesh_prov_evt_t));
    event.type = NRF_MESH_PROV_EVT_LINK_ESTABLISHED;
    event.params.link_established.p_context = &prov_context;
    m_event_handler(&event);

    /* Static authentication data is exchanged: */
    nrf_mesh_prov_auth_data_provide_ExpectAndReturn(&prov_context, config_params.p_static_data, NRF_MESH_KEY_SIZE, NRF_SUCCESS);
    memset(&event, 0, sizeof(nrf_mesh_prov_evt_t));
    event.type = NRF_MESH_PROV_EVT_STATIC_REQUEST;
    event.params.static_request.p_context = &prov_context;
    m_event_handler(&event);

    /* Configuration data is received: */
    dsm_handle_t mock_netkey_handle = 4, mock_devkey_handle = 8;
    const uint8_t mock_netkey[NRF_MESH_KEY_SIZE] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
    const uint8_t mock_devkey[NRF_MESH_KEY_SIZE] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };
    memset(&event, 0, sizeof(nrf_mesh_prov_evt_t));
    event.type = NRF_MESH_PROV_EVT_COMPLETE;
    event.params.complete.p_context = &prov_context;
    event.params.complete.p_netkey = mock_netkey;
    event.params.complete.p_devkey = mock_devkey;
    event.params.complete.iv_index = 0x10ffebad;
    event.params.complete.netkey_index = 0xada;
    event.params.complete.address = 0x10ff;
    event.params.complete.flags.iv_update = 1;
    event.params.complete.flags.key_refresh = 1;

    dsm_local_unicast_address_t expected_address;
    expected_address.address_start = event.params.complete.address;
    expected_address.count = ACCESS_ELEMENT_COUNT;
    dsm_local_unicast_addresses_set_ExpectWithArrayAndReturn(&expected_address, 1, NRF_SUCCESS);
    dsm_subnet_add_ExpectWithArrayAndReturn(event.params.complete.netkey_index, mock_netkey, NRF_MESH_KEY_SIZE, NULL, 0, NRF_SUCCESS);
    dsm_subnet_add_IgnoreArg_p_subnet_handle();
    dsm_subnet_add_ReturnThruPtr_p_subnet_handle(&mock_netkey_handle);
    dsm_devkey_add_ExpectWithArrayAndReturn(event.params.complete.address, mock_netkey_handle, mock_devkey, NRF_MESH_KEY_SIZE,
        NULL, 0, NRF_SUCCESS);
    dsm_devkey_add_IgnoreArg_p_devkey_handle();
    dsm_devkey_add_ReturnThruPtr_p_devkey_handle(&mock_devkey_handle);
    net_state_iv_index_set_ExpectAndReturn(event.params.complete.iv_index, event.params.complete.flags.iv_update, NRF_SUCCESS);
    if (event.params.complete.flags.key_refresh)
    {
        dsm_subnet_update_ExpectAndReturn(mock_netkey_handle, mock_netkey, NRF_SUCCESS);
        dsm_subnet_update_swap_keys_ExpectAndReturn(mock_netkey_handle, NRF_SUCCESS);
    }

    config_server_bind_ExpectAndReturn(mock_devkey_handle, NRF_SUCCESS);

    m_event_handler(&event);

    /* ...And the link is closed; node is happy and ready to make new friends in the network: */
    EXPECT_COMPLETE_CALLBACK(test_data);
    memset(&event, 0, sizeof(nrf_mesh_prov_evt_t));
    event.type = NRF_MESH_PROV_EVT_LINK_CLOSED;
    event.params.link_closed.p_context = &prov_context;
    m_event_handler(&event);
}

void test_config_in_flash(void)
{
    void * test_data = (void *) 0x10ff10ff;
    nrf_mesh_node_config_params_t config_params;
    memset(&config_params, 0, sizeof(nrf_mesh_node_config_params_t));
    config_params.prov_caps.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    config_params.prov_caps.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    config_params.prov_caps.num_elements = 1;
    config_params.p_data = test_data;
    config_params.complete_callback = complete_callback;

    nrf_mesh_init_ExpectAndReturn(NULL, NRF_SUCCESS);
    nrf_mesh_init_IgnoreArg_p_init_params();
    nrf_mesh_enable_ExpectAndReturn(NRF_SUCCESS);

    access_init_Expect();
    dsm_init_Expect();
    config_server_init_IgnoreAndReturn(NRF_SUCCESS);
    health_server_init_ExpectAndReturn(NULL, 0, DEVICE_COMPANY_ID, NULL, NULL, 0, NRF_SUCCESS);
    health_server_init_IgnoreArg_p_server();

    dsm_flash_config_load_ExpectAndReturn(true);
    access_flash_config_load_ExpectAndReturn(true);

    EXPECT_COMPLETE_CALLBACK(test_data);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_node_config(&config_params));
    TEST_ASSERT_NULL(m_event_handler);
}

void test_config_event_forward(void)
{
    /* Do basically the same setup as in test_config_in_flash(): */
    void * test_data = (void *) 0x10ff10ff;
    nrf_mesh_node_config_params_t config_params;
    memset(&config_params, 0, sizeof(nrf_mesh_node_config_params_t));
    config_params.prov_caps.algorithms = NRF_MESH_PROV_ALGORITHM_FIPS_P256EC;
    config_params.prov_caps.oob_static_types = NRF_MESH_PROV_OOB_STATIC_TYPE_SUPPORTED;
    config_params.prov_caps.num_elements = 1;
    config_params.p_data = test_data;
    config_params.complete_callback = complete_callback;

    /* Add config_server_callback */
    config_params.config_server_callback = config_server_callback;

    nrf_mesh_init_ExpectAndReturn(NULL, NRF_SUCCESS);
    nrf_mesh_init_IgnoreArg_p_init_params();
    nrf_mesh_enable_ExpectAndReturn(NRF_SUCCESS);

    access_init_Expect();
    dsm_init_Expect();
    config_server_init_StubWithCallback(config_server_init_stub);
    health_server_init_ExpectAndReturn(NULL, 0, DEVICE_COMPANY_ID, NULL, NULL, 0, NRF_SUCCESS);
    health_server_init_IgnoreArg_p_server();

    dsm_flash_config_load_ExpectAndReturn(true);
    access_flash_config_load_ExpectAndReturn(true);

    EXPECT_COMPLETE_CALLBACK(test_data);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_node_config(&config_params));
    TEST_ASSERT_NULL(m_event_handler);
    TEST_ASSERT_NOT_NULL(mp_config_server_evt_callback);

    /* *************** Actual test *************** */
    config_server_evt_t evt = {.type = CONFIG_SERVER_EVT_APPKEY_ADD,
                               .params.appkey_add.appkey_handle = 1};
    config_server_evt_Expect(&evt);
    mp_config_server_evt_callback(&evt);
    config_server_evt_Verify();
}
