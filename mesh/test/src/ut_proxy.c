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
#include "unity.h"
#include "cmock.h"
#include "manual_mock_queue.h"

#include "proxy.h"

#include "utils.h"
#include "proxy_test_common.h"
#include "beacon.h"

#include "proxy_filter_mock.h"
#include "timer_scheduler_mock.h"
#include "network_mock.h"
#include "net_state_mock.h"
#include "net_packet_mock.h"
#include "enc_mock.h"
#include "net_beacon_mock.h"
#include "rand_mock.h"
#include "mesh_adv_mock.h"
#include "cache_mock.h"
#include "event_mock.h"
#include "mesh_gatt_mock.h"
#include "mesh_config_entry.h"
#include "mesh_opt_gatt.h"
#include "bearer_event_mock.h"

#define NET_ID {0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70}
#define ID_KEY {0x84, 0x39, 0x6c, 0x43, 0x5a, 0xc4, 0x85, 0x60, 0xb5, 0x96, 0x53, 0x85, 0x25, 0x3e, 0x21, 0x0c}

#define CONFIG_MSG_OVERHEAD (9 /* network header */ + 8 /* mic */)
#define TX_TOKEN (0x12345678)

static struct {
    bool flag_added;
    bearer_event_flag_t flag;
    bearer_event_flag_callback_t p_flag_callback;
} m_bearer_event_ctx[2];

typedef enum {
    BEARER_EVENT_CTX_IV_UPDATE = 0,
    BEARER_EVENT_CTX_KEY_REFRESH,
    BEARER_EVENT_CTX_END,
} bearer_event_ctx_t;

static uint8_t m_flag_set_counter;

extern void proxy_deinit(void);

typedef enum
{
    CHECK_SETTING,
    CHECK_REMOVING,
    STEP_COUNT
} opt_test_steps_t;

MOCK_QUEUE_DEF(event_handler_mock, nrf_mesh_evt_t, NULL);

static nrf_mesh_network_secmat_t m_rx_net_secmat;

extern const mesh_config_entry_params_t m_mesh_opt_gatt_proxy_params;
uint32_t mesh_config_entry_set(mesh_config_entry_id_t id, const void * p_entry)
{
    return m_mesh_opt_gatt_proxy_params.callbacks.setter(id, p_entry);
}

uint32_t mesh_config_entry_get(mesh_config_entry_id_t id, void * p_entry)
{
    m_mesh_opt_gatt_proxy_params.callbacks.getter(id, p_entry);
    return NRF_SUCCESS;
}

static void expect_proxy_stopped_evt(void)
{
    nrf_mesh_evt_t expected_evt;
    expected_evt.type = NRF_MESH_EVT_PROXY_STOPPED;
    event_handler_mock_Expect(&expected_evt);
}

static void event_handle_mock(const nrf_mesh_evt_t* p_evt, int cmock_num_calls)
{
    UNUSED_PARAMETER(cmock_num_calls);

    nrf_mesh_evt_t expected_evt;
    event_handler_mock_Consume(&expected_evt);

    TEST_ASSERT_EQUAL(expected_evt.type, p_evt->type);
}

static void start_proxy(void)
{
    /* Start an advertisement */
    nrf_mesh_beacon_info_t beacon_info = {.secmat.net_id = NET_ID};
    beacon_info_set(&beacon_info, 1);
    timer_sch_reschedule_ExpectAnyArgs();

    uint8_t service_data[9];
    service_data[0] = 0;
    memcpy(&service_data[1], beacon_info.secmat.net_id, 8);

    mesh_adv_data_set_Expect(0x1828, service_data, sizeof(service_data));
    mesh_adv_params_set_Expect(0, (MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS * 1000) / 625);
    mesh_adv_start_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(true));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_start());
}

static void send_packet(void)
{
    const core_tx_bearer_interface_t * p_if = core_tx_if_get();
    core_tx_bearer_t * p_bearer = core_tx_bearer_get();
    TEST_ASSERT_NOT_NULL(p_if);
    TEST_ASSERT_NOT_NULL(p_bearer);

    packet_mesh_net_packet_t net_packet;

    network_packet_metadata_t net_metadata;
    net_metadata.dst.value = 0x1234;
    net_metadata.dst.type  = NRF_MESH_ADDRESS_TYPE_UNICAST;

    core_tx_alloc_params_t params;
    params.net_packet_len = 20;
    params.role           = CORE_TX_ROLE_ORIGINATOR;
    params.token          = TX_TOKEN;
    params.p_metadata     = &net_metadata;

    proxy_filter_accept_ExpectAndReturn(NULL, 0x1234, true);
    proxy_filter_accept_IgnoreArg_p_filter();

    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, p_if->packet_alloc(p_bearer, &params));
    TEST_ASSERT_TRUE(gatt_tx_packet_is_allocated());
    p_if->packet_send(p_bearer, net_packet.pdu, params.net_packet_len);
    TEST_ASSERT_FALSE(gatt_tx_packet_is_allocated());
}

static void disable_proxy(void)
{
    bool enabled;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_get(&enabled));
    if (enabled)
    {
        /* Expect the proxy to kill the advertiser when disabling it. */
        mesh_adv_stop_Expect();
        timer_sch_abort_ExpectAnyArgs();
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(false));
    }
}

static bearer_event_flag_t bearer_event_flag_add_mock(bearer_event_flag_callback_t callback, int n)
{
    for (size_t i = 0; i < ARRAY_SIZE(m_bearer_event_ctx); i++)
    {
        if (m_bearer_event_ctx[i].p_flag_callback == NULL ||
            m_bearer_event_ctx[i].p_flag_callback == callback)
        {
            m_bearer_event_ctx[i].p_flag_callback = callback;
            return m_bearer_event_ctx[i].flag;
        }
    }

    TEST_ASSERT_TRUE(false);
    return 0;
}

static void helper_bearer_event_trigger(bearer_event_ctx_t be)
{
    TEST_ASSERT_TRUE(m_flag_set_counter > 0);
    TEST_ASSERT_NOT_NULL(m_bearer_event_ctx[be].p_flag_callback);
    TEST_ASSERT_TRUE(m_bearer_event_ctx[be].p_flag_callback());
    m_flag_set_counter--;
}

static void helper_bearer_event_flag_set_expect(bearer_event_ctx_t be)
{
    bearer_event_flag_set_Expect(m_bearer_event_ctx[be].flag);
    m_flag_set_counter++;
}

void setUp(void)
{
    proxy_filter_mock_Init();
    timer_scheduler_mock_Init();
    network_mock_Init();
    net_state_mock_Init();
    net_packet_mock_Init();
    enc_mock_Init();
    net_beacon_mock_Init();
    rand_mock_Init();
    mesh_adv_mock_Init();
    cache_mock_Init();
    mesh_gatt_mock_Init();
    event_handler_mock_Init();
    event_mock_Init();
    event_handle_StubWithCallback(event_handle_mock);
    bearer_event_mock_Init();
    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_mock);
}

void tearDown(void)
{
    proxy_filter_mock_Verify();
    proxy_filter_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    net_packet_mock_Verify();
    net_packet_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
    net_beacon_mock_Verify();
    net_beacon_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    mesh_adv_mock_Verify();
    mesh_adv_mock_Destroy();
    cache_mock_Verify();
    cache_mock_Destroy();
    mesh_gatt_mock_Verify();
    mesh_gatt_mock_Destroy();
    event_handler_mock_Verify();
    event_handler_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();

    proxy_deinit();
}

static uint8_t * net_packet_payload_get_callback(const packet_mesh_net_packet_t * p_net_packet, int count)
{
    return (uint8_t *) &p_net_packet->pdu[9];
}

static uint32_t net_packet_payload_len_get_callback(const network_packet_metadata_t * p_net_metadata,
                                                    uint32_t net_packet_len,
                                                    int count)
{
    return net_packet_len - 9 - net_packet_mic_size_get(p_net_metadata->control_packet);
}

static void config_rx(uint8_t * p_msg, uint32_t msg_len, bool pass_decryption)
{
    packet_mesh_net_packet_t incoming;
    uint8_t decrypted[80];

    net_packet_payload_get_StubWithCallback(net_packet_payload_get_callback);
    net_packet_payload_len_get_StubWithCallback(net_packet_payload_len_get_callback);

    mesh_gatt_evt_t rx_evt;
    rx_evt.type = MESH_GATT_EVT_TYPE_RX;
    rx_evt.conn_index = 0;
    rx_evt.params.rx.pdu_type = MESH_GATT_PDU_TYPE_PROXY_CONFIG;
    rx_evt.params.rx.p_data   = incoming.pdu;
    rx_evt.params.rx.length   = msg_len + CONFIG_MSG_OVERHEAD;

    network_packet_metadata_t net_metadata;
    /* Length check should prevent the decrypt: */
    if (msg_len < MESH_GATT_PROXY_PDU_MAX_SIZE - CONFIG_MSG_OVERHEAD)
    {
        /* Only the clear text part of the decrypted message is relevant */
        memcpy(&decrypted[9], p_msg, msg_len);
        net_metadata.control_packet = true;
        net_metadata.dst.type       = NRF_MESH_ADDRESS_TYPE_INVALID;
        net_metadata.dst.value      = 0;
        net_metadata.internal.iv_index = 0x12345678;
        net_metadata.internal.sequence_number = 1234;
        net_metadata.p_security_material      = &m_rx_net_secmat;
        net_metadata.src                      = 0x1234;
        net_metadata.ttl                      = 0;

        net_packet_decrypt_ExpectAndReturn(NULL,
                                        rx_evt.params.rx.length,
                                        &incoming,
                                        NULL,
                                        NET_PACKET_KIND_PROXY_CONFIG,
                                        pass_decryption ? NRF_SUCCESS : NRF_ERROR_NOT_FOUND);
        net_packet_decrypt_IgnoreArg_p_net_metadata();
        net_packet_decrypt_IgnoreArg_p_net_decrypted_packet();
        net_packet_decrypt_ReturnThruPtr_p_net_metadata(&net_metadata);
        net_packet_decrypt_ReturnThruPtr_p_net_decrypted_packet((packet_mesh_net_packet_t *) &decrypted);
        if (pass_decryption)
        {
            net_packet_payload_len_get_ExpectAndReturn(&net_metadata, rx_evt.params.rx.length, msg_len);
        }
    }

    gatt_evt_post(&rx_evt);
}

static void expect_filter_status(void)
{
    static network_packet_metadata_t net_metadata;
    net_metadata.control_packet = true;
    net_metadata.dst.type       = NRF_MESH_ADDRESS_TYPE_INVALID;
    net_metadata.dst.value      = 0;
    net_metadata.internal.iv_index = 0x12345678;
    net_metadata.internal.sequence_number = 1234;
    net_metadata.p_security_material      = &m_rx_net_secmat;
    net_metadata.src                      = 0x1201;
    net_metadata.ttl                      = 0;

    net_state_iv_index_and_seqnum_alloc_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    net_state_iv_index_and_seqnum_alloc_ReturnThruPtr_p_iv_index(&net_metadata.internal.iv_index);
    net_state_iv_index_and_seqnum_alloc_ReturnThruPtr_p_seqnum(&net_metadata.internal.sequence_number);

    net_packet_header_set_Expect((packet_mesh_net_packet_t *) gatt_tx_packet_get(NULL), &net_metadata);

    net_packet_encrypt_Expect(&net_metadata, 4, NULL, NET_PACKET_KIND_PROXY_CONFIG);
    net_packet_encrypt_IgnoreArg_p_net_packet();
}

/*****************************************************************************
* Test functions
*****************************************************************************/
void test_invalid_state()
{
    nrf_mesh_key_refresh_phase_t kr_phase = {0};

    /* Proxy not enabled. */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_start());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_stop());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_node_id_enable(NULL, kr_phase));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_node_id_disable());

    /* Proxy enabled, but not initialized. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_enable());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_start());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_stop());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_node_id_enable(NULL, kr_phase));
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_node_id_disable());

    /* Turn off the proxy as we didn't start the service. */
    proxy_disable();

    /* Proxy initialized, but not enabled. */
    cache_init_ExpectAnyArgs();
    init();
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_start());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_stop());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, proxy_node_id_disable());
}

void test_adv_net_id(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    nrf_mesh_beacon_info_t beacon_info = {.secmat.net_id = NET_ID};
    beacon_info_set(&beacon_info, 1);
    timer_sch_reschedule_ExpectAnyArgs();

    uint8_t service_data[9];
    service_data[0] = 0;
    memcpy(&service_data[1], beacon_info.secmat.net_id, 8);

    mesh_adv_data_set_Expect(0x1828, service_data, sizeof(service_data));
    mesh_adv_params_set_Expect(0, (MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS * 1000) / 625);
    mesh_adv_start_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(true));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_start());
}

void test_adv_node_id(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    nrf_mesh_beacon_info_t beacon_info = {.secmat.identity_key = ID_KEY};
    beacon_info_set(&beacon_info, 1);
    timer_sch_abort_ExpectAnyArgs();

    uint8_t random_data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t hash_data[8] = {0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8};

    rand_hw_rng_get_Expect(NULL, sizeof(random_data));
    rand_hw_rng_get_IgnoreArg_p_result();
    rand_hw_rng_get_ReturnMemThruPtr_p_result(random_data, 8);

    uint8_t input_data[NRF_MESH_KEY_SIZE] = {0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 0x12, 0x01};

    enc_aes_encrypt_ExpectWithArray(beacon_info.secmat.identity_key, 1, &input_data[0], sizeof(input_data), NULL, 0);
    enc_aes_encrypt_IgnoreArg_p_result();
    enc_aes_encrypt_ReturnArrayThruPtr_p_result(hash_data, sizeof(random_data));

    uint8_t service_data[17];
    service_data[0] = 1;
    memcpy(&service_data[1], hash_data, 8);
    memcpy(&service_data[9], random_data, 8);

    mesh_adv_data_set_Expect(0x1828, service_data, sizeof(service_data));
    mesh_adv_params_set_Expect(60*1000, (MESH_GATT_PROXY_NODE_IDENTITY_ADV_INT_MS * 1000) / 625);
    mesh_adv_start_Expect();

    nrf_mesh_key_refresh_phase_t kr_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_node_id_enable(&beacon_info, kr_phase));
}

void test_rx_config(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();
    establish_connection(0);

    /* Set filter type */
    uint8_t set_filter_type[] = {0x00, 0x00};
    expect_filter_status();
    proxy_filter_type_set_ExpectAndReturn(NULL, PROXY_FILTER_TYPE_WHITELIST, NRF_SUCCESS);
    proxy_filter_type_set_IgnoreArg_p_filter();
    config_rx(set_filter_type, sizeof(set_filter_type), true);

    set_filter_type[1] = 0x01;
    expect_filter_status();
    proxy_filter_type_set_ExpectAndReturn(NULL, PROXY_FILTER_TYPE_BLACKLIST, NRF_SUCCESS);
    proxy_filter_type_set_IgnoreArg_p_filter();
    config_rx(set_filter_type, sizeof(set_filter_type), true);

    set_filter_type[1] = 0x02;
    proxy_filter_type_set_ExpectAndReturn(NULL, PROXY_FILTER_TYPE_RFU_START, NRF_ERROR_INVALID_PARAM);
    proxy_filter_type_set_IgnoreArg_p_filter();
    config_rx(set_filter_type, sizeof(set_filter_type), true);

    // fail length check, nothing should happen:
    uint8_t long_buf[sizeof(set_filter_type) + 1] = {0};
    config_rx(long_buf, sizeof(set_filter_type) + 1, true);
    config_rx(set_filter_type, sizeof(set_filter_type) - 1, true);

    /* Add addresses to filter */
    uint8_t add_addr_to_filter[] = {0x01, 0xaa, 0x00, 0xaa, 0x01, 0xaa, 0x00, 0xff, 0xff, 0x12, 0x34};
    uint16_t expected_addrs[]    = {0xaa00, 0xaa01, 0xaa00, 0xffff, 0x1234};
    expect_filter_status();
    proxy_filter_add_ExpectWithArray(NULL, 0, expected_addrs, ARRAY_SIZE(expected_addrs), ARRAY_SIZE(expected_addrs));
    proxy_filter_add_IgnoreArg_p_filter();
    config_rx(add_addr_to_filter, sizeof(add_addr_to_filter), true);

    /* No addrs, should still accept: */
    expect_filter_status();
    proxy_filter_add_Expect(NULL, NULL, 0);
    proxy_filter_add_IgnoreArg_p_filter();
    proxy_filter_add_IgnoreArg_p_addrs();
    config_rx(add_addr_to_filter, 1, true);

    // fail length check, nothing should happen:
    config_rx(add_addr_to_filter, 0, true);
    config_rx(add_addr_to_filter, 2, true); // Would cut one addr in two, shouldn't pass
    config_rx(add_addr_to_filter, MESH_GATT_PROXY_PDU_MAX_SIZE - CONFIG_MSG_OVERHEAD + 1, true);

    /* Remove the addrs from filter: */
    uint8_t remove_addr_from_filter[] = {0x02, 0xaa, 0x00, 0xaa, 0x01, 0xaa, 0x00, 0xff, 0xff, 0x12, 0x34};
    expect_filter_status();
    proxy_filter_remove_ExpectWithArray(NULL, 0, expected_addrs, ARRAY_SIZE(expected_addrs), ARRAY_SIZE(expected_addrs));
    proxy_filter_remove_IgnoreArg_p_filter();
    config_rx(remove_addr_from_filter, sizeof(remove_addr_from_filter), true);

    /* No addrs, should still accept: */
    expect_filter_status();
    proxy_filter_remove_Expect(NULL, NULL, 0);
    proxy_filter_remove_IgnoreArg_p_filter();
    proxy_filter_remove_IgnoreArg_p_addrs();
    config_rx(remove_addr_from_filter, 1, true);

    // fail length check, nothing should happen:
    config_rx(remove_addr_from_filter, 0, true);
    config_rx(remove_addr_from_filter, 2, true); // Would cut one addr in two, shouldn't pass
    config_rx(add_addr_to_filter, MESH_GATT_PROXY_PDU_MAX_SIZE - CONFIG_MSG_OVERHEAD + 1, true);


    /* Receive filter status message, should be ignored: */
    uint8_t filter_status[] = {0x03, 0x00, 0x01};
    config_rx(filter_status, sizeof(filter_status), true);

    /* Receive bogus message, should be ignored: */
    uint8_t garbage[] = {0x04, 0x00, 0x01};
    config_rx(garbage, sizeof(garbage), true);

    // fail decryption, nothing should happen:
    config_rx(set_filter_type, sizeof(set_filter_type), false);

}

/** Test RX of messages that'll be forwarded to other modules */
void test_rx_forward(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();
    establish_connection(0);

    packet_mesh_net_packet_t incoming;

    net_packet_payload_get_StubWithCallback(net_packet_payload_get_callback);
    net_packet_payload_len_get_StubWithCallback(net_packet_payload_len_get_callback);

    mesh_gatt_evt_t rx_evt;
    rx_evt.type = MESH_GATT_EVT_TYPE_RX;
    rx_evt.conn_index = 0;
    rx_evt.params.rx.p_data   = incoming.pdu;
    rx_evt.params.rx.length   = sizeof(incoming);

    rx_evt.params.rx.pdu_type = MESH_GATT_PDU_TYPE_NETWORK_PDU;
    network_packet_in_ExpectAndReturn(&incoming.pdu[0], rx_evt.params.rx.length, NULL, NRF_SUCCESS);
    network_packet_in_IgnoreArg_p_rx_metadata();
    gatt_evt_post(&rx_evt);

    rx_evt.params.rx.pdu_type = MESH_GATT_PDU_TYPE_MESH_BEACON;
    net_beacon_packet_in_Expect(&incoming.pdu[0 + BEACON_PACKET_OVERHEAD], rx_evt.params.rx.length - BEACON_PACKET_OVERHEAD, NULL);
    net_beacon_packet_in_IgnoreArg_p_meta();
    gatt_evt_post(&rx_evt);

    /* Unexpected PDU type, should be ignored */
    rx_evt.params.rx.pdu_type = MESH_GATT_PDU_TYPE_PROV_PDU;
    gatt_evt_post(&rx_evt);
}

void test_tx_mesh(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();
    establish_connection(0);

    const core_tx_bearer_interface_t * p_if = core_tx_if_get();
    core_tx_bearer_t * p_bearer = core_tx_bearer_get();
    TEST_ASSERT_NOT_NULL(p_if);
    TEST_ASSERT_NOT_NULL(p_bearer);

    packet_mesh_net_packet_t net_packet;

    network_packet_metadata_t net_metadata;
    net_metadata.dst.value = 0x1234;
    net_metadata.dst.type  = NRF_MESH_ADDRESS_TYPE_UNICAST;

    core_tx_alloc_params_t params;
    params.net_packet_len = 20;
    params.role           = CORE_TX_ROLE_ORIGINATOR;
    params.token          = TX_TOKEN;
    params.p_metadata     = &net_metadata;

    /* Successful allocation: */
    proxy_filter_accept_ExpectAndReturn(NULL, 0x1234, true);
    proxy_filter_accept_IgnoreArg_p_filter();

    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, p_if->packet_alloc(p_bearer, &params));
    TEST_ASSERT_TRUE(gatt_tx_packet_is_allocated());
    p_if->packet_send(p_bearer, net_packet.pdu, params.net_packet_len);
    TEST_ASSERT_FALSE(gatt_tx_packet_is_allocated());

    /* Filter rejects the packet: */
    proxy_filter_accept_ExpectAndReturn(NULL, 0x1234, false);
    proxy_filter_accept_IgnoreArg_p_filter();

    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_REJECTED, p_if->packet_alloc(p_bearer, &params));
    TEST_ASSERT_FALSE(gatt_tx_packet_is_allocated());

    /* No mem: */
    proxy_filter_accept_ExpectAndReturn(NULL, 0x1234, true);
    proxy_filter_accept_IgnoreArg_p_filter();
    gatt_tx_packet_availability_set(false);

    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_NO_MEM, p_if->packet_alloc(p_bearer, &params));
    TEST_ASSERT_FALSE(gatt_tx_packet_is_allocated());
}

/**
 * Test various events that should trigger the proxy server to send beacons to the connected device:
 * 1. On TX ready (send beacon for all secmats)
 * 2. On incoming beacon with new data
 * 3. On subnet add
 */
void test_tx_beacon(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    nrf_mesh_beacon_info_t beacon_info[2];
    beacon_info_set(&beacon_info[0], 2);

    establish_connection(0);

    uint8_t beacon_packet[NET_BEACON_BUFFER_SIZE];
    memset(beacon_packet, 0xbe, sizeof(beacon_packet));

    /* 1. TX ready */
    for (uint32_t i = 0; i < ARRAY_SIZE(beacon_info); ++i)
    {
        net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
        net_state_iv_update_get_ExpectAndReturn(18);
        net_beacon_build_ExpectAndReturn(&beacon_info[i].secmat, 0x12345678, 18, false, NULL, NRF_SUCCESS);
        net_beacon_build_IgnoreArg_p_buffer();
        net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));
    }

    mesh_gatt_evt_t tx_ready_evt;
    tx_ready_evt.type = MESH_GATT_EVT_TYPE_TX_READY;
    tx_ready_evt.conn_index = 0;
    gatt_evt_post(&tx_ready_evt);

    net_state_mock_Verify();

    uint16_t len;
    TEST_ASSERT_EQUAL(ARRAY_SIZE(beacon_info), gatt_tx_packet_alloc_count());
    /* Verify that the sent packet was the beacon packet we built */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(beacon_packet, gatt_tx_packet_get(&len), sizeof(beacon_packet));
    TEST_ASSERT_EQUAL(sizeof(beacon_packet), len);

    /* 2. Incoming beacon with new data */
    nrf_mesh_rx_metadata_t rx_meta = {.source = NRF_MESH_RX_SOURCE_SCANNER};
    uint8_t beacon_auth[NET_BEACON_CMAC_SIZE];
    nrf_mesh_evt_t mesh_evt;
    mesh_evt.type                                = NRF_MESH_EVT_NET_BEACON_RECEIVED;
    mesh_evt.params.net_beacon.iv_index          = 0x12; /* Should trigger a beacon */
    mesh_evt.params.net_beacon.flags.key_refresh = false;
    mesh_evt.params.net_beacon.flags.iv_update   = NET_STATE_IV_UPDATE_NORMAL;
    mesh_evt.params.net_beacon.p_beacon_info     = &beacon_info[0];
    mesh_evt.params.net_beacon.p_beacon_secmat   = &beacon_info[0].secmat;
    mesh_evt.params.net_beacon.p_rx_metadata     = &rx_meta;
    mesh_evt.params.net_beacon.p_auth_value      = beacon_auth;

    cache_has_elem_ExpectAndReturn(NULL, beacon_auth, false);
    cache_has_elem_IgnoreArg_p_cache();

    cache_put_Expect(NULL, beacon_auth);
    cache_put_IgnoreArg_p_cache();

    net_beacon_build_ExpectAndReturn(mesh_evt.params.net_beacon.p_beacon_secmat,
                                     mesh_evt.params.net_beacon.iv_index,
                                     mesh_evt.params.net_beacon.flags.iv_update,
                                     mesh_evt.params.net_beacon.flags.key_refresh,
                                     NULL,
                                     NRF_SUCCESS);
    net_beacon_build_IgnoreArg_p_buffer();
    net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));

    mesh_evt_post(&mesh_evt);

    cache_has_elem_ExpectAndReturn(NULL, beacon_auth, false);
    cache_has_elem_IgnoreArg_p_cache();

    cache_put_Expect(NULL, beacon_auth);
    cache_put_IgnoreArg_p_cache();

    net_beacon_build_ExpectAndReturn(mesh_evt.params.net_beacon.p_beacon_secmat,
                                     mesh_evt.params.net_beacon.iv_index,
                                     mesh_evt.params.net_beacon.flags.iv_update,
                                     mesh_evt.params.net_beacon.flags.key_refresh,
                                     NULL,
                                     NRF_SUCCESS);
    net_beacon_build_IgnoreArg_p_buffer();
    net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));


    mesh_evt_post(&mesh_evt);

    // Cache reports that it already contains the beacon, shouldn't send
    cache_has_elem_ExpectAndReturn(NULL, beacon_auth, true);
    cache_has_elem_IgnoreArg_p_cache();

    mesh_evt_post(&mesh_evt);

    // fail beacon sending on update, should not overwrite state:

    cache_has_elem_ExpectAndReturn(NULL, beacon_auth, false);
    cache_has_elem_IgnoreArg_p_cache();

    gatt_tx_packet_availability_set(false);
    mesh_evt_post(&mesh_evt);
    // send the same event again, but with space in the buffer. This time, we should notify the client:
    gatt_tx_packet_availability_set(true);
    net_beacon_build_ExpectAndReturn(mesh_evt.params.net_beacon.p_beacon_secmat,
                                     mesh_evt.params.net_beacon.iv_index,
                                     mesh_evt.params.net_beacon.flags.iv_update,
                                     mesh_evt.params.net_beacon.flags.key_refresh,
                                     NULL,
                                     NRF_SUCCESS);
    net_beacon_build_IgnoreArg_p_buffer();
    net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));

    cache_has_elem_ExpectAndReturn(NULL, beacon_auth, false);
    cache_has_elem_IgnoreArg_p_cache();

    cache_put_Expect(NULL, beacon_auth);
    cache_put_IgnoreArg_p_cache();

    mesh_evt_post(&mesh_evt);

    /* 3. Subnet add: Should trigger on added subnet */
    net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
    net_state_iv_update_get_ExpectAndReturn(18);
    net_beacon_build_ExpectAndReturn(&beacon_info[0].secmat, 0x12345678, 18, false, NULL, NRF_SUCCESS);
    net_beacon_build_IgnoreArg_p_buffer();
    net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));

    proxy_subnet_added(0, &beacon_info[0].secmat.net_id[0]);
}

void test_disconnect(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();
    /* Start an advertisement */
    nrf_mesh_beacon_info_t beacon_info = {.secmat.net_id = NET_ID};
    beacon_info_set(&beacon_info, 1);
    timer_sch_reschedule_ExpectAnyArgs();

    uint8_t service_data[9];
    service_data[0] = 0;
    memcpy(&service_data[1], beacon_info.secmat.net_id, 8);

    mesh_adv_data_set_Expect(0x1828, service_data, sizeof(service_data));
    mesh_adv_params_set_Expect(0, (MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS * 1000) / 625);
    mesh_adv_start_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(true));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_start());

    /* Connect, then disconnect */
    establish_connection(0);

    // should resume advertising
    timer_sch_reschedule_ExpectAnyArgs();
    mesh_adv_data_set_Expect(0x1828, service_data, sizeof(service_data));
    mesh_adv_params_set_Expect(0, (MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS * 1000) / 625);
    mesh_adv_start_Expect();

    disconnect(0);

}

void test_enable_get_set(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    for (opt_test_steps_t step = 0; step < STEP_COUNT; step++)
    {
        bool enabled = true;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_get(&enabled));
        TEST_ASSERT_FALSE(enabled);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(false));

        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(true));

        enabled = false;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_get(&enabled));
        TEST_ASSERT_TRUE(enabled);

        /* Start advertising */
        nrf_mesh_beacon_info_t beacon_info = {.secmat.net_id = NET_ID};
        beacon_info_set(&beacon_info, 1);

        uint8_t service_data[9];
        service_data[0] = 0;
        memcpy(&service_data[1], beacon_info.secmat.net_id, 8);

        mesh_adv_data_set_Expect(0x1828, service_data, sizeof(service_data));
        mesh_adv_params_set_Expect(0, (MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS * 1000) / 625);
        mesh_adv_start_Expect();

        timer_sch_reschedule_ExpectAnyArgs();
        TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_start());

        switch (step)
        {
            case CHECK_SETTING:
            {
                /* Expect the proxy to kill the advertiser when disabling it. */
                mesh_adv_stop_Expect();
                timer_sch_abort_ExpectAnyArgs();

                TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(false));
                break;
            }
            case CHECK_REMOVING:
            {
                mesh_config_entry_id_t entry_id = MESH_OPT_GATT_PROXY_EID;
                /* Expect the proxy to kill the advertiser when deleting it. */
                mesh_adv_stop_Expect();
                timer_sch_abort_ExpectAnyArgs();
                m_mesh_opt_gatt_proxy_params.callbacks.deleter(entry_id);
                break;
            }
            default:
                TEST_FAIL_MESSAGE("Wrong test behavior");
        }

        enabled = true;
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_get(&enabled));
        TEST_ASSERT_FALSE(enabled);
    }
}

void test_stop_no_connections(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();
    /* Deinit callback to monitor calls */
    mesh_gatt_disconnect_StubWithCallback(NULL);

    start_proxy();

    /* proxy_stop() sends PROXY_STOPPED immediately */
    timer_sch_abort_ExpectAnyArgs();
    mesh_adv_stop_Expect();
    expect_proxy_stopped_evt();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_stop());
}

void test_stop_without_pending_packets(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    mesh_gatt_packet_is_pending_IgnoreAndReturn(false);
    /* Deinit callback to monitor calls */
    mesh_gatt_disconnect_StubWithCallback(NULL);

    start_proxy();

    establish_connection(0);

    TEST_ASSERT_TRUE(proxy_is_connected());

    mesh_gatt_disconnect_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_stop());

    /* Send DISCONNECTED event */
    expect_proxy_stopped_evt();
    disconnect(0);
}

void test_stop_with_pending_packets(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();
    /* Deinit callback to monitor calls */
    mesh_gatt_disconnect_StubWithCallback(NULL);

    start_proxy();

    establish_connection(0);

    TEST_ASSERT_TRUE(proxy_is_connected());

    send_packet();

    mesh_gatt_packet_is_pending_ExpectAnyArgsAndReturn(true);

    /* mesh_gatt_disconnect() should not be called */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_stop());

    /* Send TX_COMPLETE event */
    mesh_gatt_packet_is_pending_ExpectAnyArgsAndReturn(false);
    mesh_gatt_disconnect_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    mesh_gatt_evt_t tx_complete_evt;
    tx_complete_evt.type = MESH_GATT_EVT_TYPE_TX_COMPLETE;
    tx_complete_evt.conn_index = 0;
    tx_complete_evt.params.tx_complete.pdu_type = MESH_GATT_PDU_TYPE_NETWORK_PDU;
    tx_complete_evt.params.tx_complete.token = TX_TOKEN;
    gatt_evt_post(&tx_complete_evt);

    /* Send DISCONNECTED event */
    expect_proxy_stopped_evt();
    disconnect(0);
}

void test_stop_through_proxy_state(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    start_proxy();

    establish_connection(0);

    /* Shall NOT call disconnect */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(false));

    /* Send DISCONNECTED event. PROXY_STOPPED event should not be generated */
    disconnect(0);
}

void test_ivi_update(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    nrf_mesh_beacon_info_t beacon_info[2];
    beacon_info_set(&beacon_info[0], 2);

    establish_connection(0);

    uint8_t beacon_packet[NET_BEACON_BUFFER_SIZE];
    memset(beacon_packet, 0xbe, sizeof(beacon_packet));

    nrf_mesh_evt_t mesh_evt;
    mesh_evt.type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION;

    /* Send mesh event and expect bearer_event to be triggered. */
    helper_bearer_event_flag_set_expect(BEARER_EVENT_CTX_IV_UPDATE);
    mesh_evt_post(&mesh_evt);

    /* Prepare for bearer event to be triggered and trigger it. */
    for (uint32_t j = 0; j < ARRAY_SIZE(beacon_info); ++j)
    {
        net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
        net_state_iv_update_get_ExpectAndReturn(18);
        net_beacon_build_ExpectAndReturn(&beacon_info[j].secmat, 0x12345678, 18, false, NULL, NRF_SUCCESS);
        net_beacon_build_IgnoreArg_p_buffer();
        net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));
    }

    helper_bearer_event_trigger(BEARER_EVENT_CTX_IV_UPDATE);
}

void test_ivi_update_no_mem(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    nrf_mesh_beacon_info_t beacon_info[2];
    beacon_info_set(&beacon_info[0], 2);

    establish_connection(0);

    uint8_t beacon_packet[NET_BEACON_BUFFER_SIZE];
    memset(beacon_packet, 0xbe, sizeof(beacon_packet));

    nrf_mesh_evt_t mesh_evt;
    mesh_evt.type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION;

    /* Send mesh event to trigger bearer event. */
    helper_bearer_event_flag_set_expect(BEARER_EVENT_CTX_IV_UPDATE);
    mesh_evt_post(&mesh_evt);

    /* Case 1: mesh_gatt_packet_alloc() can't allocate a packet.
     * The transmission is postponed until TX_COMPLETE. */
    gatt_tx_packet_availability_set(false);
    net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
    net_state_iv_update_get_ExpectAndReturn(18);
    helper_bearer_event_trigger(BEARER_EVENT_CTX_IV_UPDATE);

    /* Case 2: TX_COMPLETE happened, but mesh_gatt_packet_alloc() still can't allocate a packet.
     * Wait for the next TX_COMPLETE. */
    net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
    net_state_iv_update_get_ExpectAndReturn(18);
    mesh_gatt_evt_t tx_complete_evt;
    tx_complete_evt.type = MESH_GATT_EVT_TYPE_TX_COMPLETE;
    tx_complete_evt.conn_index = 0;
    tx_complete_evt.params.tx_complete.pdu_type = MESH_GATT_PDU_TYPE_NETWORK_PDU;
    tx_complete_evt.params.tx_complete.token = TX_TOKEN;
    gatt_evt_post(&tx_complete_evt);

    /* Case 3: Now we can send beacons. */
    gatt_tx_packet_availability_set(true);
    for (uint32_t j = 0; j < ARRAY_SIZE(beacon_info); ++j)
    {
        net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
        net_state_iv_update_get_ExpectAndReturn(18);
        net_beacon_build_ExpectAndReturn(&beacon_info[j].secmat, 0x12345678, 18, false, NULL, NRF_SUCCESS);
        net_beacon_build_IgnoreArg_p_buffer();
        net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));
    }

    tx_complete_evt.type = MESH_GATT_EVT_TYPE_TX_COMPLETE;
    tx_complete_evt.conn_index = 0;
    tx_complete_evt.params.tx_complete.pdu_type = MESH_GATT_PDU_TYPE_NETWORK_PDU;
    tx_complete_evt.params.tx_complete.token = TX_TOKEN;
    gatt_evt_post(&tx_complete_evt);
}

void test_key_refresh(void)
{
    /* Clear the state */
    disable_proxy();

    cache_init_ExpectAnyArgs();
    init();

    nrf_mesh_beacon_info_t beacon_info[2];
    beacon_info_set(&beacon_info[0], 2);

    establish_connection(0);

    uint8_t beacon_packet[NET_BEACON_BUFFER_SIZE];
    memset(beacon_packet, 0xbe, sizeof(beacon_packet));

    uint8_t net_id[NRF_MESH_NETID_SIZE];
    nrf_mesh_evt_t mesh_evt;
    mesh_evt.type = NRF_MESH_EVT_KEY_REFRESH_NOTIFICATION;
    mesh_evt.params.key_refresh.p_network_id = net_id;
    mesh_evt.params.key_refresh.phase = NRF_MESH_KEY_REFRESH_PHASE_0;
    mesh_evt.params.key_refresh.subnet_index = 0;

    /* This should only trigger bearer event. */
    helper_bearer_event_flag_set_expect(BEARER_EVENT_CTX_KEY_REFRESH);
    mesh_evt_post(&mesh_evt);

    /* Now beacons should be sent. */
    net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
    net_state_iv_update_get_ExpectAndReturn(18);
    net_beacon_build_ExpectAndReturn(&beacon_info[0].secmat, 0x12345678, 18, false, NULL, NRF_SUCCESS);
    net_beacon_build_IgnoreArg_p_buffer();
    net_beacon_build_ReturnArrayThruPtr_p_buffer(beacon_packet, sizeof(beacon_packet));
    helper_bearer_event_trigger(BEARER_EVENT_CTX_KEY_REFRESH);
}
