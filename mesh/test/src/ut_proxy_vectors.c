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

#include "proxy.h"
#include "packet_mesh.h"
#include "net_packet.h"

#include "mesh_gatt.h"
#include "proxy_filter_mock.h"
#include "timer_scheduler_mock.h"
#include "mesh_adv_mock.h"
#include "proxy_test_common.h"
#include "net_state_mock.h"
#include "msg_cache_mock.h"
#include "rand_mock.h"
#include "advertiser_mock.h"
#include "beacon_mock.h"
#include "mesh_gatt_mock.h"
#include "mesh_config_entry.h"
#include "mesh_opt_gatt.h"
#include "bearer_event_mock.h"

#define NET_ENC_KEY_1     0x3a, 0x4f, 0xe8, 0x4a, 0x6c, 0xc2, 0xc6, 0xa7, 0x66, 0xea, 0x93, 0xf1, 0x08, 0x4d, 0x40, 0x39
#define NET_PRIVACY_KEY_1 0xf6, 0x95, 0xfc, 0xce, 0x70, 0x9c, 0xcf, 0xac, 0xe4, 0xd8, 0xb7, 0xa1, 0xe6, 0xe3, 0x9d, 0x25

#define NET_ID_2          0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70
#define ID_KEY_2          0x84, 0x39, 0x6c, 0x43, 0x5a, 0xc4, 0x85, 0x60, 0xb5, 0x96, 0x53, 0x85, 0x25, 0x3e, 0x21, 0x0c
#define RANDOM_DATA       0x34, 0xae, 0x60, 0x8f, 0xbb, 0xc1, 0xf2, 0xc6

extern void proxy_deinit(void);

static const struct
{
    nrf_mesh_network_secmat_t net_secmat;
    uint32_t iv_index;
    packet_mesh_net_packet_t enc_packet;
    uint8_t enc_packet_len;
    uint8_t out_data[33];
} m_proxy_config_msg_vector =
{
    .net_secmat =
    {
        .nid = 0x10,
        .encryption_key = {NET_ENC_KEY_1},
        .privacy_key = {NET_PRIVACY_KEY_1}
    },
    .iv_index = 0x12345678,
    .enc_packet.pdu = {0x10, 0x38, 0x6b, 0xd6, 0x0e, 0xfb, 0xbb, 0x8b, 0x8c, 0x28, 0x51, 0x2e, 0x79, 0x2d, 0x37, 0x11, 0xf4, 0xb5, 0x26},
    .enc_packet_len = 19,
    .out_data = {0x00, 0x00}
};

static const struct {
    nrf_mesh_beacon_info_t beacon_info;
    uint8_t expected_service_data[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH];
    uint8_t expected_len;
    uint16_t expected_uuid;
} m_service_data_net_id = {
    .beacon_info.secmat.net_id = {NET_ID_2},
    .expected_service_data = {0x00, NET_ID_2},
    .expected_len = 9,
    .expected_uuid = 0x1828
};

static struct {
    nrf_mesh_beacon_info_t beacon_info;

    uint16_t source_addr;
    uint8_t random_data[8];
    uint8_t expected_service_data[BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH];
    uint8_t expected_len;
    uint16_t expected_uuid;
} m_service_data_node_id = {
    .beacon_info.secmat.identity_key = {ID_KEY_2},
    .random_data = {RANDOM_DATA},
    .source_addr = 0x1201,
    .expected_service_data = {0x01, 0x00, 0x86, 0x17, 0x65, 0xae, 0xfc, 0xc5, 0x7b, RANDOM_DATA},
    .expected_len = 17,
    .expected_uuid = 0x1828
};

void setUp(void)
{
    /* Clear the state */
    bool enabled;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_get(&enabled));
    if (enabled)
    {
        /* Expect the proxy to kill the advertiser when disabling it. */
        mesh_adv_stop_Expect();
        timer_sch_abort_ExpectAnyArgs();
        TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(false));
    }
    proxy_filter_mock_Init();
    timer_scheduler_mock_Init();
    mesh_adv_mock_Init();
    net_state_mock_Init();
    msg_cache_mock_Init();
    rand_mock_Init();
    advertiser_mock_Init();
    beacon_mock_Init();
    mesh_gatt_mock_Init();
    bearer_event_mock_Init();
    bearer_event_flag_add_IgnoreAndReturn(0);

    msg_cache_entry_exists_IgnoreAndReturn(false);
}

void tearDown(void)
{
    proxy_filter_mock_Verify();
    proxy_filter_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    mesh_adv_mock_Verify();
    mesh_adv_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    msg_cache_mock_Verify();
    msg_cache_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    beacon_mock_Verify();
    beacon_mock_Destroy();
    mesh_gatt_mock_Verify();
    mesh_gatt_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();

    proxy_deinit();
}
/*****************************************************************************
* Mock functions
*****************************************************************************/
uint32_t network_packet_in(const uint8_t * p_packet, uint32_t net_packet_len, const nrf_mesh_rx_metadata_t * p_rx_metadata)
{
    return NRF_SUCCESS;
}

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

/*****************************************************************************
* Test functions
*****************************************************************************/

void test_config_msg_rx(void)
{
    init();
    establish_connection(0);

    // Expected outcome from test vector: The proxy server sets the filter to whitelist
    proxy_filter_type_set_ExpectAndReturn(NULL, PROXY_FILTER_TYPE_WHITELIST, NRF_SUCCESS);
    proxy_filter_type_set_IgnoreArg_p_filter();

    net_secmat_set(&m_proxy_config_msg_vector.net_secmat);
    net_state_rx_iv_index_get_ExpectAndReturn(m_proxy_config_msg_vector.iv_index & 0x000000001,
                                              m_proxy_config_msg_vector.iv_index);

    // expect it to respond with a config message:
    uint32_t seqnum = 1;
    uint32_t iv_index = m_proxy_config_msg_vector.iv_index;
    net_state_iv_index_and_seqnum_alloc_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    net_state_iv_index_and_seqnum_alloc_ReturnThruPtr_p_iv_index(&iv_index);
    net_state_iv_index_and_seqnum_alloc_ReturnThruPtr_p_seqnum(&seqnum);

    mesh_gatt_evt_t rx_evt;
    rx_evt.type = MESH_GATT_EVT_TYPE_RX;
    rx_evt.conn_index = 0;
    rx_evt.params.rx.p_data = m_proxy_config_msg_vector.enc_packet.pdu;
    rx_evt.params.rx.length = m_proxy_config_msg_vector.enc_packet_len;
    rx_evt.params.rx.pdu_type = MESH_GATT_PDU_TYPE_PROXY_CONFIG;
    gatt_evt_post(&rx_evt);

    /* The response message should have been a filter status, but we can only check the length as we
     * don't have sample data for this: */
    uint16_t rsp_len = 0;
    uint8_t * p_rsp = gatt_tx_packet_get(&rsp_len);
    TEST_ASSERT_NOT_NULL(p_rsp);
    TEST_ASSERT_EQUAL(PACKET_MESH_NET_PDU_OFFSET + 1 /* opcode len */ + 3 /* param len */ + 8 /* mic len */, rsp_len);

    proxy_filter_mock_Verify();
}

void test_adv_net_id(void)
{
    init();

    beacon_info_set(&m_service_data_net_id.beacon_info, 1);
    timer_sch_reschedule_ExpectAnyArgs();

    mesh_adv_data_set_ExpectWithArray(m_service_data_net_id.expected_uuid,
                                      m_service_data_net_id.expected_service_data,
                                      m_service_data_net_id.expected_len,
                                      m_service_data_net_id.expected_len);
    mesh_adv_params_set_Expect(0, (MESH_GATT_PROXY_NETWORK_ID_ADV_INT_MS * 1000) / 625);
    mesh_adv_start_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_gatt_proxy_set(true));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_start());
}

void test_adv_node_id(void)
{
    init();

    timer_sch_abort_ExpectAnyArgs();
    rand_hw_rng_get_Expect(NULL, 8);
    rand_hw_rng_get_IgnoreArg_p_result();
    rand_hw_rng_get_ReturnMemThruPtr_p_result(m_service_data_node_id.random_data, 8);
    mesh_adv_data_set_ExpectWithArray(m_service_data_node_id.expected_uuid,
                                      m_service_data_node_id.expected_service_data,
                                      m_service_data_node_id.expected_len,
                                      m_service_data_node_id.expected_len);
    mesh_adv_params_set_Expect(MESH_GATT_PROXY_NODE_IDENTITY_DURATION_MS, (MESH_GATT_PROXY_NODE_IDENTITY_ADV_INT_MS * 1000) / 625);
    mesh_adv_start_Expect();
    nrf_mesh_key_refresh_phase_t kr_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, proxy_node_id_enable(&m_service_data_node_id.beacon_info, kr_phase));
}
