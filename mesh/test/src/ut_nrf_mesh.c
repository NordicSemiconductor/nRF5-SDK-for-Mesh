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

#include "log.h"

#include "nrf_mesh.h"

#include "unity.h"
#include "cmock.h"

#define TEST_UUID { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f }

/*************** Mock Includes ***************/

#include "toolchain_mock.h"
#include "timer_scheduler_mock.h"
#include "transport_mock.h"
#include "network_mock.h"
#include "msg_cache_mock.h"
#include "enc_mock.h"
#include "rand_mock.h"
#include "nrf_mesh_dfu_mock.h"
#include "nrf_mesh_configure_mock.h"
#include "bearer_event_mock.h"
#include "beacon_mock.h"
#include "event_mock.h"
#include "prov_bearer_adv_mock.h"
#include "mesh_flash_mock.h"
#include "bearer_handler_mock.h"
#include "scanner_mock.h"
#include "timeslot_mock.h"
#include "advertiser_mock.h"
#include "mesh_mem_mock.h"
#include "core_tx_adv_mock.h"
#include "ad_listener_mock.h"
#include "heartbeat_mock.h"
#include "mesh_config_mock.h"
#include "mesh_opt_mock.h"
#include "core_tx_local_mock.h"
#include "ad_type_filter_mock.h"
#include "net_state_mock.h"

#define REGISTERED_BE_HANDLER   ((bearer_event_flag_t)0x2A)

static uint32_t m_rx_cb_expect;
static nrf_mesh_adv_packet_rx_data_t m_adv_packet_rx_data_expect;
static int m_scanner_init_callback_cnt;
static bearer_event_flag_callback_t m_scanner_packet_process_cb;
static scanner_packet_t m_test_packet;
static nrf_mesh_rx_metadata_t m_metadata;
static uint8_t m_ad_buffer[32];
static ble_ad_data_t * mp_ad_data = (ble_ad_data_t *)m_ad_buffer;
static bearer_event_flag_callback_t m_nrf_mesh_enable_cb;
static nrf_mesh_evt_handler_cb_t m_config_stable_cb;
static bool m_enabled_event_fired;

/*************** Static Helper Functions ***************/

static void ad_listener_process_cb(ble_packet_type_t adv_type,
                                   const uint8_t * p_payload,
                                   uint32_t payload_size,
                                   const nrf_mesh_rx_metadata_t * p_metadata,
                                   int num_calls)
{
    // Mesh packet AD listener extern:
    extern ad_listener_t m_mesh_packet_listener;
    TEST_ASSERT_EQUAL(AD_TYPE_MESH, m_mesh_packet_listener.ad_type);
    TEST_ASSERT_EQUAL(BLE_PACKET_TYPE_ADV_NONCONN_IND, m_mesh_packet_listener.adv_packet_type);
    TEST_ASSERT_NOT_NULL(m_mesh_packet_listener.handler);

    TEST_ASSERT_EQUAL(&m_test_packet.packet.payload, p_payload);
    TEST_ASSERT_EQUAL(m_test_packet.packet.header.length - BLE_ADV_PACKET_OVERHEAD, payload_size);
    TEST_ASSERT_EQUAL(AD_TYPE_MESH, mp_ad_data->type);
    m_mesh_packet_listener.handler(mp_ad_data->data, mp_ad_data->length - BLE_AD_DATA_OVERHEAD, p_metadata);
}

static void scanner_init_callback(bearer_event_flag_callback_t packet_process_cb, int cmock_num_calls)
{
    m_scanner_init_callback_cnt++;
    m_scanner_packet_process_cb = packet_process_cb;
}

bearer_event_flag_t bearer_event_flag_add_cb(bearer_event_flag_callback_t nrf_mesh_enable_cb, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_NOT_NULL(nrf_mesh_enable_cb);

    m_nrf_mesh_enable_cb = nrf_mesh_enable_cb;
    return REGISTERED_BE_HANDLER;
}

void event_handle_cb(const nrf_mesh_evt_t* p_evt, int num_calls)
{
    (void)num_calls;
    TEST_ASSERT_NOT_NULL(p_evt);
    TEST_ASSERT_EQUAL(NRF_MESH_EVT_ENABLED, p_evt->type);
    m_enabled_event_fired = true;
}

void event_handler_add_cb(nrf_mesh_evt_handler_t* p_handler_params, int num_calls)
{
    (void)num_calls;

    TEST_ASSERT_NOT_NULL(p_handler_params);
    TEST_ASSERT_NOT_NULL(p_handler_params->evt_cb);
    m_config_stable_cb = p_handler_params->evt_cb;
}

void event_handler_remove_cb(nrf_mesh_evt_handler_t* p_handler_params, int num_calls)
{
    (void)num_calls;

    TEST_ASSERT_NOT_NULL(p_handler_params);
    TEST_ASSERT_NOT_NULL(p_handler_params->evt_cb);
    m_config_stable_cb = NULL;
}

static void initialize_mesh(nrf_mesh_init_params_t * p_init_params)
{
    m_scanner_init_callback_cnt = 0;
    m_scanner_packet_process_cb = NULL;

#ifdef __linux__
    toolchain_init_irqs_Expect();
#endif
    nrf_mesh_configure_device_uuid_reset_Expect();
    msg_cache_init_Expect();
    timer_sch_init_Expect();
    bearer_event_init_Expect(NRF_MESH_IRQ_PRIORITY_LOWEST);
    transport_init_Expect();
    network_init_Expect(p_init_params);
    scanner_init_StubWithCallback(scanner_init_callback);
    advertiser_init_Expect();
    mesh_flash_init_Expect();
    heartbeat_init_Expect();
    mesh_mem_init_Expect();
    mesh_config_init_Expect();
    bearer_handler_init_Expect();
    core_tx_adv_init_Expect();
    mesh_opt_init_Expect();
    core_tx_local_init_Expect();
    ad_listener_init_Expect();
    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_cb);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_init(p_init_params));

    TEST_ASSERT_EQUAL_INT(1, m_scanner_init_callback_cnt);
    scanner_init_StubWithCallback(NULL);
}

static void rx_cb(const nrf_mesh_adv_packet_rx_data_t * p_rx_data)
{
    TEST_ASSERT_TRUE(m_rx_cb_expect > 0);
    m_rx_cb_expect--;
    TEST_ASSERT_EQUAL(m_adv_packet_rx_data_expect.p_metadata->source, p_rx_data->p_metadata->source);
    TEST_ASSERT_EQUAL(m_adv_packet_rx_data_expect.p_metadata->params.scanner.access_addr, p_rx_data->p_metadata->params.scanner.access_addr);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(
        m_adv_packet_rx_data_expect.p_metadata->params.scanner.adv_addr.addr,
        p_rx_data->p_metadata->params.scanner.adv_addr.addr,
        6);
    TEST_ASSERT_EQUAL(m_adv_packet_rx_data_expect.p_metadata->params.scanner.adv_addr.addr_type, p_rx_data->p_metadata->params.scanner.adv_addr.addr_type);
    TEST_ASSERT_EQUAL(m_adv_packet_rx_data_expect.p_metadata->params.scanner.channel, p_rx_data->p_metadata->params.scanner.channel);
    TEST_ASSERT_EQUAL(m_adv_packet_rx_data_expect.p_metadata->params.scanner.rssi, p_rx_data->p_metadata->params.scanner.rssi);
    TEST_ASSERT_EQUAL(m_adv_packet_rx_data_expect.p_metadata->params.scanner.timestamp, p_rx_data->p_metadata->params.scanner.timestamp);
    TEST_ASSERT_EQUAL_HEX8(m_adv_packet_rx_data_expect.adv_type, p_rx_data->adv_type);
    TEST_ASSERT_EQUAL(m_adv_packet_rx_data_expect.length, p_rx_data->length);
    TEST_ASSERT_EQUAL_PTR(m_adv_packet_rx_data_expect.p_payload, p_rx_data->p_payload);
}

/*************** Test Initialization and Finalization ***************/

void setUp(void)
{
    toolchain_mock_Init();
    timer_scheduler_mock_Init();
    transport_mock_Init();
    network_mock_Init();
    msg_cache_mock_Init();
    enc_mock_Init();
    rand_mock_Init();
    nrf_mesh_dfu_mock_Init();
    bearer_event_mock_Init();
    beacon_mock_Init();
    event_mock_Init();
    nrf_mesh_configure_mock_Init();
    prov_bearer_adv_mock_Init();
    mesh_flash_mock_Init();
    bearer_handler_mock_Init();
    scanner_mock_Init();
    timeslot_mock_Init();
    advertiser_mock_Init();
    mesh_mem_mock_Init();
    core_tx_adv_mock_Init();
    ad_listener_mock_Init();
    heartbeat_mock_Init();
    mesh_config_mock_Init();
    mesh_opt_mock_Init();
    core_tx_local_mock_Init();
    ad_type_filter_mock_Init();
    net_state_mock_Init();
    __LOG_INIT((LOG_SRC_API | LOG_SRC_TEST), LOG_LEVEL_ERROR, LOG_CALLBACK_DEFAULT);

    /*
     * The mesh stack cannot be reset after it is initialized, so this code ensures
     * it is only initialized once.
     */
    static bool initialized = false;
    nrf_mesh_init_params_t init_params = { .irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST };
    if (!initialized)
    {
        initialize_mesh(&init_params);
        initialized = true;
    }
}

void tearDown(void)
{
    toolchain_mock_Verify();
    toolchain_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    nrf_mesh_dfu_mock_Verify();
    nrf_mesh_dfu_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    beacon_mock_Verify();
    beacon_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    nrf_mesh_configure_mock_Verify();
    nrf_mesh_configure_mock_Destroy();
    prov_bearer_adv_mock_Verify();
    prov_bearer_adv_mock_Destroy();
    mesh_flash_mock_Verify();
    mesh_flash_mock_Destroy();
    bearer_handler_mock_Verify();
    bearer_handler_mock_Destroy();
    scanner_mock_Verify();
    scanner_mock_Destroy();
    timeslot_mock_Verify();
    timeslot_mock_Destroy();
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    mesh_mem_mock_Verify();
    mesh_mem_mock_Destroy();
    core_tx_adv_mock_Verify();
    core_tx_adv_mock_Destroy();
    ad_listener_mock_Verify();
    ad_listener_mock_Destroy();
    heartbeat_mock_Verify();
    heartbeat_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
    mesh_opt_mock_Verify();
    mesh_opt_mock_Destroy();
    core_tx_local_mock_Verify();
    core_tx_local_mock_Destroy();
    ad_type_filter_mock_Verify();
    ad_type_filter_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
}

/*************** Test Cases ***************/

/*
 * About 0xf00dbeef: To test that wrapper functions doesn't just return NRF_SUCCESS without
 * calling the functions they are supposed to call, the mock functions are also set to return
 * 0xf00dbeef. This makes it possible to check if the return value from the wrapped function
 * is correctly returned by the wrapper.
 */

void test_init(void)
{
    nrf_mesh_init_params_t init_params = { .irq_priority = NRF_MESH_IRQ_PRIORITY_LOWEST };

    /* Second initialization will give an error: */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, nrf_mesh_init(&init_params));
}

void test_enable_disable(void)
{
    bearer_handler_start_ExpectAndReturn(NRF_SUCCESS);
    network_enable_Expect();
    transport_enable_Expect();
    bearer_event_start_Expect();
    bearer_event_flag_set_Expect(REGISTERED_BE_HANDLER);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_enable());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, nrf_mesh_enable());

    net_state_is_seqnum_block_ready_ExpectAndReturn(true);
    scanner_enable_Expect();
    event_handle_Ignore();
    TEST_ASSERT_TRUE(m_nrf_mesh_enable_cb());

    bearer_handler_stop_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_disable());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, nrf_mesh_disable());
}

void test_packet_send(void)
{
    nrf_mesh_tx_params_t params = {};
    uint32_t reference;

    /* No checking of parameters in nrf_mesh, it's all left to transport: */
    transport_tx_ExpectAndReturn(&params, &reference, NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_packet_send(&params, &reference));
}

void test_process(void)
{
    bearer_event_handler_ExpectAndReturn(true);
    TEST_ASSERT_EQUAL(true, nrf_mesh_process());

    bearer_event_handler_ExpectAndReturn(false);
    TEST_ASSERT_EQUAL(false, nrf_mesh_process());
}

void test_scanner_packet_process_cb(void)
{
    /* No incoming packets ready: */
    scanner_rx_ExpectAndReturn(NULL);
    scanner_rx_pending_ExpectAndReturn(false);
    TEST_ASSERT_EQUAL(true, m_scanner_packet_process_cb());

    memset(m_test_packet.metadata.adv_addr.addr, 0x12, BLE_GAP_ADDR_LEN);
    m_test_packet.metadata.adv_addr.addr_type = 1;
    m_test_packet.metadata.access_addr = 0x12345678;
    m_test_packet.metadata.channel = 39;
    m_test_packet.metadata.rssi = -34;
    m_test_packet.metadata.timestamp = 0xabcdef;
    m_test_packet.metadata.adv_type = BLE_PACKET_TYPE_ADV_NONCONN_IND;

    memset(&m_metadata, 0, sizeof(m_metadata));
    m_metadata.source         = NRF_MESH_RX_SOURCE_SCANNER;
    m_metadata.params.scanner = m_test_packet.metadata;

    /* Process a mesh packet: */
    mp_ad_data->length = 2;
    mp_ad_data->type = AD_TYPE_MESH;
    mp_ad_data->data[0] = 4;
    m_test_packet.packet.header.length = BLE_ADV_PACKET_OVERHEAD + 3;

    scanner_rx_ExpectAndReturn(&m_test_packet);
    ad_listener_process_StubWithCallback(ad_listener_process_cb);
    network_packet_in_ExpectAndReturn(&mp_ad_data->data[0], 1, &m_metadata, NRF_SUCCESS);
    network_packet_in_IgnoreArg_p_rx_metadata();
    scanner_packet_release_Expect(&m_test_packet);
    scanner_rx_pending_ExpectAndReturn(false);
    TEST_ASSERT_EQUAL(true, m_scanner_packet_process_cb());

    /* Test application rx cb */
    nrf_mesh_rx_cb_set(rx_cb);

    memset(m_test_packet.packet.addr, 0x12, BLE_GAP_ADDR_LEN);
    m_test_packet.packet.header.addr_type = 1;

    m_test_packet.packet.header.length = BLE_ADV_PACKET_OVERHEAD + 5;
    m_test_packet.packet.header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    m_test_packet.packet.payload[0] = 4;
    m_test_packet.packet.payload[1] = 0x09; // full local name, should only be picked up by the rx cb.
    m_test_packet.packet.payload[2] = 'Y';
    m_test_packet.packet.payload[3] = 'e';
    m_test_packet.packet.payload[4] = 'p';

    m_rx_cb_expect = 1;
    m_adv_packet_rx_data_expect.length = 5;
    m_adv_packet_rx_data_expect.adv_type = m_test_packet.packet.header.type;
    m_adv_packet_rx_data_expect.p_payload = m_test_packet.packet.payload;
    m_adv_packet_rx_data_expect.p_metadata = &m_metadata;

    scanner_rx_ExpectAndReturn(&m_test_packet);
    ad_listener_process_Expect(m_test_packet.packet.header.type,
                               m_test_packet.packet.payload,
                               m_test_packet.packet.header.length - BLE_ADV_PACKET_OVERHEAD,
                               &m_metadata);
    ad_listener_process_Ignore();
    scanner_packet_release_Expect(&m_test_packet);
    scanner_rx_pending_ExpectAndReturn(false);
    TEST_ASSERT_EQUAL(true, m_scanner_packet_process_cb());
    TEST_ASSERT_EQUAL(0, m_rx_cb_expect);

    /* Remove the callback, shouldn't get it again. */
    nrf_mesh_rx_cb_clear();

    scanner_rx_ExpectAndReturn(&m_test_packet);
    ad_listener_process_Expect(m_test_packet.packet.header.type,
                               m_test_packet.packet.payload,
                               m_test_packet.packet.header.length - BLE_ADV_PACKET_OVERHEAD,
                               &m_metadata);
    ad_listener_process_Ignore();
    scanner_packet_release_Expect(&m_test_packet);
    scanner_rx_pending_ExpectAndReturn(true);
    TEST_ASSERT_EQUAL(false, m_scanner_packet_process_cb());
}

void test_evt_handler_add(void)
{
    nrf_mesh_evt_handler_t event_handler = {};

    event_handler_add_Expect(&event_handler);
    nrf_mesh_evt_handler_add(&event_handler);
}

void test_evt_handler_remove(void)
{
    nrf_mesh_evt_handler_t event_handler = {};

    event_handler_remove_Expect(&event_handler);
    nrf_mesh_evt_handler_remove(&event_handler);
}

void test_evt_nrf_mesh_enabled(void)
{
    TEST_ASSERT_NOT_NULL(m_nrf_mesh_enable_cb);

    bearer_handler_start_ExpectAndReturn(NRF_SUCCESS);
    bearer_event_flag_set_Expect(REGISTERED_BE_HANDLER);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, nrf_mesh_enable());

    net_state_is_seqnum_block_ready_ExpectAndReturn(false);
    event_handler_add_StubWithCallback(event_handler_add_cb);
    TEST_ASSERT_TRUE(m_nrf_mesh_enable_cb());
    TEST_ASSERT_FALSE(m_enabled_event_fired);

    TEST_ASSERT_NOT_NULL(m_config_stable_cb);
    event_handler_remove_StubWithCallback(event_handler_remove_cb);
    bearer_event_flag_set_Expect(REGISTERED_BE_HANDLER);
    const nrf_mesh_evt_t evt =
    {
        .type = NRF_MESH_EVT_CONFIG_STABLE,
    };
    m_config_stable_cb(&evt);
    net_state_is_seqnum_block_ready_ExpectAndReturn(true);
    scanner_enable_Expect();
    event_handle_StubWithCallback(event_handle_cb);
    TEST_ASSERT_TRUE(m_nrf_mesh_enable_cb());
    TEST_ASSERT_TRUE(m_enabled_event_fired);
}
