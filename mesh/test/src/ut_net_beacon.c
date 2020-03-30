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

#include <stdlib.h>

#include <cmock.h>
#include <unity.h>

#include "net_beacon.h"

#include "nrf_mesh.h"
#include "nrf_mesh_externs.h"

#include "beacon_mock.h"
#include "enc_mock.h"
#include "timer_scheduler_mock.h"
#include "rand_mock.h"
#include "timer_mock.h"
#include "net_state_mock.h"
#include "event_mock.h"
#include "advertiser_mock.h"
#include "mesh_config_entry_mock.h"

#include "utils.h"
#include "test_assert.h"
#include "nrf_mesh_events.h"
#include "mesh_opt_core.h"

#define BEACON_OBSERVATION_PERIOD_S (NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS * NRF_MESH_BEACON_OBSERVATION_PERIODS)

/* Secure network beacon sample packet 1: */
#define SEC_NET_SAMPLE_DATA_1 {                                                                                                                          \
    .iv_update          = false,                                                                                                                         \
    .key_refresh        = false,                                                                                                                         \
    .net_key            = {0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18, 0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6},                              \
    .iv_index           = 0x12345678,                                                                                                                    \
    .info.secmat.net_id = {0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70},                                                                              \
    .info.secmat.key    = {0x54, 0x23, 0xd9, 0x67, 0xda, 0x63, 0x9a, 0x99, 0xcb, 0x02, 0x23, 0x1a, 0x83, 0xf7, 0xd2, 0x54},                              \
    .info.iv_update_permitted = true,                                                                                                                    \
    .auth               = {0x8e, 0xa2, 0x61, 0x58, 0x2f, 0x36, 0x4f, 0x6f, 0x3c, 0x74, 0xef, 0x80, 0x33, 0x6c, 0xa1, 0x7e},                              \
    .beacon             = {0x00, 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70, 0x12, 0x34, 0x56, 0x78, 0x8e, 0xa2, 0x61, 0x58, 0x2f, 0x36, 0x4f, 0x6f} \
}

/* Secure network beacon sample packet 2 (IV update in progress): */
#define SEC_NET_SAMPLE_DATA_2 {                                                                                                                          \
    .iv_update          = true,                                                                                                                          \
    .key_refresh        = false,                                                                                                                         \
    .net_key            = {0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18, 0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6},                              \
    .iv_index           = 0x12345679,                                                                                                                    \
    .info.secmat.net_id = {0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70},                                                                              \
    .info.secmat.key    = {0x54, 0x23, 0xd9, 0x67, 0xda, 0x63, 0x9a, 0x99, 0xcb, 0x02, 0x23, 0x1a, 0x83, 0xf7, 0xd2, 0x54},                              \
    .info.iv_update_permitted = true,                                                                                                                    \
    .auth               = {0xc2, 0xaf, 0x80, 0xad, 0x07, 0x2a, 0x13, 0x5c, 0x28, 0xcf, 0x84, 0x33, 0x69, 0x88, 0x70, 0x39},                              \
    .beacon             = {0x02, 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70, 0x12, 0x34, 0x56, 0x79, 0xc2, 0xaf, 0x80, 0xad, 0x07, 0x2a, 0x13, 0x5c} \
}

/* Secure network beacon sample packet 3 (IV update complete): */
#define SEC_NET_SAMPLE_DATA_3 {                                                                                                                          \
    .iv_update          = false,                                                                                                                         \
    .key_refresh        = false,                                                                                                                         \
    .net_key            = {0x7d, 0xd7, 0x36, 0x4c, 0xd8, 0x42, 0xad, 0x18, 0xc1, 0x7c, 0x2b, 0x82, 0x0c, 0x84, 0xc3, 0xd6},                              \
    .iv_index           = 0x12345679,                                                                                                                    \
    .info.secmat.net_id = {0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70},                                                                              \
    .info.secmat.key    = {0x54, 0x23, 0xd9, 0x67, 0xda, 0x63, 0x9a, 0x99, 0xcb, 0x02, 0x23, 0x1a, 0x83, 0xf7, 0xd2, 0x54},                              \
    .info.iv_update_permitted = true,                                                                                                                    \
    .auth               = {0xc6, 0x2f, 0x09, 0xe4, 0xc9, 0x57, 0xf5, 0x9d, 0x96, 0xf5, 0x06, 0xf6, 0x46, 0x04, 0xbf, 0xc1},                              \
    .beacon             = {0x00, 0x3e, 0xca, 0xff, 0x67, 0x2f, 0x67, 0x33, 0x70, 0x12, 0x34, 0x56, 0x79, 0xc6, 0x2f, 0x09, 0xe4, 0xc9, 0x57, 0xf5, 0x9d} \
}

typedef struct
{
    bool iv_update;
    bool key_refresh;
    uint8_t net_key[NRF_MESH_KEY_SIZE];
    uint32_t iv_index;
    nrf_mesh_beacon_info_t info;

    uint8_t auth[NRF_MESH_KEY_SIZE];
    uint8_t beacon[21]; /**< Without beacon type */
} net_beacon_sample_data_t;

static uint8_t * mp_expected_net_id;
static nrf_mesh_beacon_info_t ** mpp_infos;
static uint32_t m_info_count;
static uint32_t m_info_index;
static timer_sch_callback_t m_timer_cb;
static uint32_t m_rand_value;
static timestamp_t m_time_now;
static advertiser_t * mp_adv;
static advertiser_tx_complete_cb_t m_tx_complete_cb;

extern const mesh_config_entry_params_t m_net_beacon_enable_params;

void setUp(void)
{
    mp_expected_net_id = NULL;
    m_info_index = 0;
    m_info_count = 0;
    mpp_infos = NULL;
    m_time_now = 0;
    beacon_mock_Init();
    net_state_mock_Init();
    timer_scheduler_mock_Init();
    timer_mock_Init();
    enc_mock_Init();
    event_mock_Init();
    advertiser_mock_Init();
    mesh_config_entry_mock_Init();
}

void tearDown(void)
{
    beacon_mock_Verify();
    beacon_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
}

void nrf_mesh_beacon_info_next_get(const uint8_t * p_network_id, const nrf_mesh_beacon_info_t ** pp_beacon_info,
        nrf_mesh_key_refresh_phase_t * p_subnet_kr_phase)
{
    if (p_network_id == NULL)
    {
        TEST_ASSERT_EQUAL_PTR(NULL, mp_expected_net_id);
    }
    else
    {
        TEST_ASSERT_EQUAL_HEX8_ARRAY(mp_expected_net_id, p_network_id, NRF_MESH_NETID_SIZE);
    }

    TEST_ASSERT_NOT_NULL(mpp_infos);
    TEST_ASSERT_NOT_EQUAL(0, m_info_count);
    TEST_ASSERT_TRUE(m_info_index <= m_info_count);

    if (m_info_index >= m_info_count)
    {
        *pp_beacon_info = NULL;
    }
    else
    {
        if (m_info_index == 0)
        {
            TEST_ASSERT_EQUAL(NULL, *pp_beacon_info);
        }
        *pp_beacon_info = mpp_infos[m_info_index];
        *p_subnet_kr_phase = NRF_MESH_KEY_REFRESH_PHASE_0;
    }

    m_info_index++;
}

void timer_sch_schedule_mock_cb(timer_event_t * p_event, int count)
{
    TEST_ASSERT_NOT_NULL(p_event);
    TEST_ASSERT_NOT_NULL(p_event->cb);
    TEST_ASSERT_EQUAL(SEC_TO_US(NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS), p_event->interval);
    TEST_ASSERT_TRUE(m_time_now + SEC_TO_US(NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS) > p_event->timestamp);
    TEST_ASSERT_TRUE(m_time_now < p_event->timestamp);
    m_timer_cb = p_event->cb;
}

static void advertiser_instance_init_cb(advertiser_t * p_adv, advertiser_tx_complete_cb_t tx_cb, uint8_t * p_buffer, uint32_t buffer_size, int calls)
{
    TEST_ASSERT_NOT_NULL(p_adv);
    TEST_ASSERT_NOT_NULL(tx_cb);
    TEST_ASSERT_NOT_NULL(p_buffer);
    TEST_ASSERT_EQUAL(ADVERTISER_PACKET_BUFFER_PACKET_MAXLEN, buffer_size);

    mp_adv = p_adv;
    m_tx_complete_cb = tx_cb;
}

static void advertiser_enable_cb(advertiser_t * p_adv, int calls)
{
    TEST_ASSERT_EQUAL(p_adv, mp_adv);
}

static void setup_module(void)
{
    m_rand_value = 123456789; // larger than the beacon interval
    rand_hw_rng_get_Expect(NULL, 4);
    rand_hw_rng_get_IgnoreArg_p_result();
    rand_hw_rng_get_ReturnMemThruPtr_p_result((uint8_t*) &m_rand_value, 4);
    uint8_t dummy_salt[NRF_MESH_KEY_SIZE] = {0, 1, 2, 3, 4, 5, 6, 7, 0, 1, 2, 3, 4, 5, 6, 7};
    enc_s1_ExpectWithArray((const uint8_t*) "nkbk", 4, 4, NULL, 0);
    enc_s1_IgnoreArg_p_out();
    enc_s1_ReturnMemThruPtr_p_out(dummy_salt, NRF_MESH_KEY_SIZE);
    timer_now_ExpectAndReturn(1000);
    timer_sch_schedule_StubWithCallback(timer_sch_schedule_mock_cb);
    advertiser_instance_init_StubWithCallback(advertiser_instance_init_cb);
    advertiser_interval_set_Expect(NULL, 1000);
    advertiser_interval_set_IgnoreArg_p_adv();
    advertiser_enable_StubWithCallback(advertiser_enable_cb);

    net_beacon_init();
    net_beacon_enable();
    TEST_ASSERT_NOT_NULL(m_timer_cb);
    TEST_ASSERT_NOT_NULL(mp_adv);
}

static void tx_complete(void)
{
    timer_now_ExpectAndReturn(m_time_now);
    m_tx_complete_cb(mp_adv, 0, m_time_now);
}

static void expect_tx(const uint8_t * p_key, const uint8_t * p_beacon_data, uint8_t * p_auth, adv_packet_t * p_adv_packet)
{
    enc_aes_cmac_ExpectWithArray(p_key, NRF_MESH_KEY_SIZE, p_beacon_data, 13, 13, NULL, 0);
    enc_aes_cmac_IgnoreArg_p_result();
    enc_aes_cmac_ReturnMemThruPtr_p_result(p_auth, 8);
    beacon_create_ExpectWithArrayAndReturn(mp_adv, 1, BEACON_TYPE_SEC_NET_BCAST, p_beacon_data, 21, 21, p_adv_packet);
    advertiser_packet_send_Expect(mp_adv, p_adv_packet);
}
/*****************************************************************************
* Tests
*****************************************************************************/
void test_tx_single(void)
{
    setup_module();

    net_beacon_sample_data_t sample_datas[] = {SEC_NET_SAMPLE_DATA_1, SEC_NET_SAMPLE_DATA_2, SEC_NET_SAMPLE_DATA_3};

    /* single sample data */
    for (uint32_t i = 0; i < ARRAY_SIZE(sample_datas); i++)
    {
        nrf_mesh_beacon_tx_info_t tx_info;
        net_beacon_sample_data_t sample_data = sample_datas[i];
        memset(&tx_info, 0, sizeof(tx_info));
        sample_data.info.p_tx_info = &tx_info;

        nrf_mesh_beacon_info_t * p_info[] = {&sample_data.info};
        mpp_infos = p_info;
        m_info_count = 1;
        adv_packet_t adv_packet;

        m_info_index = 0;
        m_time_now = SEC_TO_US(10);

        /* Test packet creation */
        expect_tx(sample_data.info.secmat.key, sample_data.beacon, sample_data.auth, &adv_packet);

        /* Call the timeout */
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

        tx_complete();

        enc_mock_Verify();

        /* Run again, fail bearer_tx */
        m_info_index = 0;
        m_time_now += SEC_TO_US(10);
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        beacon_create_ExpectWithArrayAndReturn(mp_adv, 1, BEACON_TYPE_SEC_NET_BCAST, sample_data.beacon, 21, 21, NULL);

        /* Call the timeout */
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);
        tx_complete();

        enc_mock_Verify();
        beacon_mock_Verify();
    }
}

/**
 * Test that we can run through multiple subnets on tx
 */
void test_tx_multi(void)
{
    setup_module();

#define NETWORKS    5
    /* multiple sample data */
    nrf_mesh_beacon_info_t info[NETWORKS];
    uint8_t beacon_data[NETWORKS][21];
    nrf_mesh_beacon_tx_info_t tx_info[NETWORKS];
    nrf_mesh_beacon_info_t * p_info[NETWORKS];
    adv_packet_t adv_packet;
    uint8_t auth[8];

    memset(&info, 0, sizeof(info));
    memset(&tx_info, 0, sizeof(tx_info));
    memset(auth, 0xAB, sizeof(auth));

    for (uint32_t i = 0; i < NETWORKS; i++)
    {
        memset(&tx_info[i], 0, sizeof(tx_info[i]));
        info[i].p_tx_info = &tx_info[i];
        p_info[i] = &info[i];

        beacon_data[i][0] = 0x00;
        memcpy(&beacon_data[i][1], info[i].secmat.net_id, 8);
        beacon_data[i][9] = 0x12;
        beacon_data[i][10] = 0x34;
        beacon_data[i][11] = 0x56;
        beacon_data[i][12] = 0x78;
        memcpy(&beacon_data[i][13], auth, sizeof(auth));
    }
    mpp_infos = p_info;
    m_info_count = NETWORKS;

    m_info_index = 0;
    m_time_now = SEC_TO_US(10);


    /* Test packet creation */

    for (uint32_t i = 0; i < NETWORKS; i++)
    {
        net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
        net_state_iv_update_get_ExpectAndReturn(NET_STATE_IV_UPDATE_NORMAL);
        expect_tx(info[i].secmat.key, &beacon_data[i][0], auth, &adv_packet);

        /* On the first run, we call the timer. The next beacons will be sent as the previous one completed its TX */
        if (i == 0)
        {
            m_timer_cb(m_time_now, NULL);
        }
        else
        {
            tx_complete();
        }
        enc_mock_Verify();
        beacon_mock_Verify();
    }

#undef NETWORKS
}

void test_packet_in(void)
{
    setup_module();

    net_beacon_sample_data_t sample_datas[] = {SEC_NET_SAMPLE_DATA_1, SEC_NET_SAMPLE_DATA_2, SEC_NET_SAMPLE_DATA_3};
    for (uint32_t i = 0; i < ARRAY_SIZE(sample_datas); i++)
    {
        nrf_mesh_beacon_tx_info_t tx_info;
        net_beacon_sample_data_t sample_data = sample_datas[i];
        memset(&tx_info, 0, sizeof(tx_info));

        sample_data.info.p_tx_info = &tx_info;

        nrf_mesh_beacon_info_t * p_info[] = {&sample_data.info};
        mpp_infos = p_info;
        m_info_count = 1;
        mp_expected_net_id = sample_data.info.secmat.net_id;

        nrf_mesh_evt_t evt;
        evt.type                                = NRF_MESH_EVT_NET_BEACON_RECEIVED;
        evt.params.net_beacon.p_beacon_info     = &sample_data.info;
        evt.params.net_beacon.p_beacon_secmat   = &sample_data.info.secmat;
        evt.params.net_beacon.p_rx_metadata     = NULL;
        evt.params.net_beacon.iv_index          = sample_data.iv_index;
        evt.params.net_beacon.flags.iv_update   = sample_data.iv_update;
        evt.params.net_beacon.flags.key_refresh = sample_data.key_refresh;

        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        event_handle_Expect(&evt);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(1, sample_data.info.p_tx_info->rx_count);

        /* Try again, should bump the tx count */
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        event_handle_Expect(&evt);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(2, sample_data.info.p_tx_info->rx_count);

        /* Try again without permitting IV update. Should still produce an event, and should count the RX */
        sample_data.info.iv_update_permitted = false;
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        event_handle_Expect(&evt);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(3, sample_data.info.p_tx_info->rx_count);


        /* Don't roll over the rx count */
        m_info_index = 0;
        sample_data.info.iv_update_permitted = true;
        sample_data.info.p_tx_info->rx_count = 0xFFFF;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        event_handle_Expect(&evt);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(0xFFFF, sample_data.info.p_tx_info->rx_count);

        /* Invalid auth, shouldn't handle the beacon: */
        uint8_t dummy_auth[NRF_MESH_KEY_SIZE];
        memset(dummy_auth, 0xDA, NRF_MESH_KEY_SIZE);
        sample_data.info.p_tx_info->rx_count = 0;
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(dummy_auth, 16);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count);
    }
}

void test_pkt_in_multi(void)
{
    setup_module();

    /* This is stupid, but we need individual storage of the same beacon+auth
     * multiple times for lookup into our own beacons, as they'll hold
     * different tx info pointers. */
    net_beacon_sample_data_t stored_beacons[] =
    {
        SEC_NET_SAMPLE_DATA_1,
        SEC_NET_SAMPLE_DATA_1,
        SEC_NET_SAMPLE_DATA_1
    };
    /* Invalidate the second auth, making this one fail verification. */
    memset(stored_beacons[1].auth, 0x56, NRF_MESH_KEY_SIZE);

    /* Yield 3 networks on lookup with NET ID */
    nrf_mesh_beacon_info_t * p_info[] =
    {
        &stored_beacons[0].info,
        &stored_beacons[1].info,
        &stored_beacons[2].info
    };
    nrf_mesh_beacon_tx_info_t tx_infos[ARRAY_SIZE(p_info)];
    memset(tx_infos, 0, sizeof(tx_infos));
    for (uint32_t j = 0; j < ARRAY_SIZE(p_info); j++)
    {
        p_info[j]->p_tx_info = &tx_infos[j];
    }

    mpp_infos = p_info;
    m_info_count = ARRAY_SIZE(p_info);

    net_beacon_sample_data_t sample_data = SEC_NET_SAMPLE_DATA_1;

    nrf_mesh_evt_t evt[ARRAY_SIZE(p_info)];

    mp_expected_net_id = sample_data.info.secmat.net_id;

    m_info_index = 0;
    for (uint32_t j = 0; j < ARRAY_SIZE(p_info); j++)
    {
        enc_aes_cmac_ExpectWithArray(p_info[j]->secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(stored_beacons[j].auth, 16);

        /* Only the valid auths should generate an event: */
        if (j != 1)
        {
            evt[j].type                                = NRF_MESH_EVT_NET_BEACON_RECEIVED;
            evt[j].params.net_beacon.p_rx_metadata     = NULL;
            evt[j].params.net_beacon.iv_index          = sample_data.iv_index;
            evt[j].params.net_beacon.flags.iv_update   = sample_data.iv_update;
            evt[j].params.net_beacon.flags.key_refresh = sample_data.key_refresh;
            evt[j].params.net_beacon.p_beacon_info   = p_info[j];
            evt[j].params.net_beacon.p_beacon_secmat = &p_info[j]->secmat;
            event_handle_Expect(&evt[j]);
        }
    }
    net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
    TEST_ASSERT_EQUAL(1, tx_infos[0].rx_count);
    TEST_ASSERT_EQUAL(0, tx_infos[1].rx_count); /* Had invalid auth, didn't get a match */
    TEST_ASSERT_EQUAL(1, tx_infos[2].rx_count);
}

void test_beacon_state_change(void)
{
    mesh_config_entry_id_t entry_id = MESH_OPT_CORE_SEC_NWK_BCN_EID;
    bool get_state;
    bool set_state;

    TEST_ASSERT_NOT_NULL(m_net_beacon_enable_params.callbacks.getter);
    TEST_ASSERT_NOT_NULL(m_net_beacon_enable_params.callbacks.setter);
    TEST_ASSERT_NOT_NULL(m_net_beacon_enable_params.callbacks.deleter);

    setup_module();
    m_net_beacon_enable_params.callbacks.getter(entry_id, &get_state);
    TEST_ASSERT_EQUAL(true, get_state);

    /* Enable the beacon while already enabled: */
    set_state = true;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_net_beacon_enable_params.callbacks.setter(entry_id, &set_state));
    m_net_beacon_enable_params.callbacks.getter(entry_id, &get_state);
    TEST_ASSERT_EQUAL(true, get_state);

    /* Disable the beacon while running: */
    timer_sch_abort_Expect(NULL);
    timer_sch_abort_IgnoreArg_p_timer_evt();
    advertiser_disable_ExpectAnyArgs();
    set_state = false;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_net_beacon_enable_params.callbacks.setter(entry_id, &set_state));
    m_net_beacon_enable_params.callbacks.getter(entry_id, &get_state);
    TEST_ASSERT_EQUAL(false, get_state);

    /* Disable the beacon while already disabled: */
    set_state = false;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_net_beacon_enable_params.callbacks.setter(entry_id, &set_state));
    m_net_beacon_enable_params.callbacks.getter(entry_id, &get_state);
    TEST_ASSERT_EQUAL(false, get_state);

    /* Enable the beacon while disabled: */
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(NULL, 0);
    timer_sch_reschedule_IgnoreArg_p_timer_evt();
    timer_sch_reschedule_IgnoreArg_new_timestamp();
    set_state = true;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, m_net_beacon_enable_params.callbacks.setter(entry_id, &set_state));
    m_net_beacon_enable_params.callbacks.getter(entry_id, &get_state);
    TEST_ASSERT_EQUAL(true, get_state);

    /* Delete the beacon */
#if MESH_FEATURE_LPN_ENABLED
    advertiser_disable_ExpectAnyArgs();
    timer_sch_abort_ExpectAnyArgs();
    m_net_beacon_enable_params.callbacks.deleter(entry_id);
    m_net_beacon_enable_params.callbacks.getter(entry_id, &get_state);
    TEST_ASSERT_EQUAL(false, get_state);
#else
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_ExpectAnyArgs();
    m_net_beacon_enable_params.callbacks.deleter(entry_id);
    m_net_beacon_enable_params.callbacks.getter(entry_id, &get_state);
    TEST_ASSERT_EQUAL(true, get_state);
#endif

    /* Check high level interface */
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_SEC_NWK_BCN_EID, NULL, NRF_SUCCESS);
    mesh_config_entry_set_IgnoreArg_p_entry();
    net_beacon_state_set(true);

    bool retval = true;
    mesh_config_entry_get_ExpectAndReturn(MESH_OPT_CORE_SEC_NWK_BCN_EID, NULL, NRF_SUCCESS);
    mesh_config_entry_get_IgnoreArg_p_entry();
    mesh_config_entry_get_ReturnThruPtr_p_entry(&retval);
    TEST_ASSERT_EQUAL(true, net_beacon_state_get());
}

/**
 * Test that receiving beacons for a specific network slows down the retransmit rate
 */
void test_redundant_tx(void)
{
    setup_module();

    net_beacon_sample_data_t sample_datas[] = {SEC_NET_SAMPLE_DATA_1, SEC_NET_SAMPLE_DATA_2, SEC_NET_SAMPLE_DATA_3};
    for (uint32_t i = 0; i < ARRAY_SIZE(sample_datas); i++)
    {
        nrf_mesh_beacon_tx_info_t tx_info;
        net_beacon_sample_data_t sample_data = sample_datas[i];
        memset(&tx_info, 0, sizeof(tx_info));
        sample_data.info.p_tx_info = &tx_info;

        nrf_mesh_beacon_info_t * p_info[] = {&sample_data.info};
        mpp_infos = p_info;
        m_info_count = 1;
        adv_packet_t adv_packet;

        /* transmit a beacon on the first timeout. */
        m_time_now = SEC_TO_US(10);
        m_info_index = 0;

        /* Call the timeout */
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        expect_tx(sample_data.info.secmat.key, sample_data.beacon, sample_data.auth, &adv_packet);
        m_timer_cb(m_time_now, NULL);
        tx_complete();
        TEST_ASSERT_EQUAL(m_time_now, sample_data.info.p_tx_info->tx_timestamp); // should be reset on each transmit.

        /* Observe one beacon this interval, don't expect it to fire: */
        sample_data.info.p_tx_info->rx_count = 1;
        m_time_now += SEC_TO_US(10);
        m_info_index = 0;
        m_timer_cb(m_time_now, NULL);

        /* The observation period is expired. We have recalculated new interval */
        TEST_ASSERT_EQUAL(BEACON_OBSERVATION_PERIOD_S * (1 + 1) / 2, sample_data.info.p_tx_info->interval);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each observation period.

        /* Don't observe any beacons this interval, triggering a TX: */
        for (uint32_t j = 0; j < NRF_MESH_BEACON_OBSERVATION_PERIODS; ++j)
        {
            m_time_now += SEC_TO_US(10);
            m_info_index = 0;
            net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
            net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
            expect_tx(sample_data.info.secmat.key, sample_data.beacon, sample_data.auth, &adv_packet);
            m_timer_cb(m_time_now, NULL);
            tx_complete();
            TEST_ASSERT_EQUAL(m_time_now, sample_data.info.p_tx_info->tx_timestamp); // should be set on each transmit.
        }

        /* Observe several beacons this interval, should cause it to skip the next two TX's: */
        sample_data.info.p_tx_info->rx_count = 4;
        for (uint32_t j = 0; j < NRF_MESH_BEACON_OBSERVATION_PERIODS; ++j)
        {
            m_time_now += SEC_TO_US(10);
            m_info_index = 0;
            m_timer_cb(m_time_now, NULL);
        }

        /* Verify that the beacon interval is changed every the broadcast interval according to received beacons within observation period */
        sample_data.info.p_tx_info->interval = 100; // clear interval from the previous step (preset condition)
        sample_data.info.p_tx_info->rx_count = 4;
        m_time_now += SEC_TO_US(10);
        m_info_index = 0;
        m_timer_cb(m_time_now, NULL);
        TEST_ASSERT_EQUAL(4, sample_data.info.p_tx_info->rx_count); // should be reset on each observation period.
        TEST_ASSERT_EQUAL(BEACON_OBSERVATION_PERIOD_S * (4 + 1) / 2, sample_data.info.p_tx_info->interval);

        sample_data.info.p_tx_info->rx_count = 7;
        m_time_now += SEC_TO_US(10);
        m_info_index = 0;
        m_timer_cb(m_time_now, NULL);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each observation period.
        TEST_ASSERT_EQUAL(BEACON_OBSERVATION_PERIOD_S * (7 + 1) / 2, sample_data.info.p_tx_info->interval);

        /* Verify that the beacon interval is set to maximum value 600 seconds */
        sample_data.info.p_tx_info->interval = 100; // clear interval from the previous step (preset condition)
        sample_data.info.p_tx_info->rx_count = 99;
        m_time_now += SEC_TO_US(10);
        m_info_index = 0;
        m_timer_cb(m_time_now, NULL);
        TEST_ASSERT_EQUAL(99, sample_data.info.p_tx_info->rx_count); // should be reset on each observation period.
        TEST_ASSERT_EQUAL(600, sample_data.info.p_tx_info->interval);
        m_time_now += SEC_TO_US(10);
        m_info_index = 0;
        m_timer_cb(m_time_now, NULL);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each observation period.
        TEST_ASSERT_NOT_EQUAL(BEACON_OBSERVATION_PERIOD_S * (99 + 1) / 2, sample_data.info.p_tx_info->interval);
        TEST_ASSERT_EQUAL(600, sample_data.info.p_tx_info->interval);

        /* Verify that the beacon interval is set to minimum value 10 seconds */
        sample_data.info.p_tx_info->interval = 0; // clear interval from the previous step (preset condition)
        sample_data.info.p_tx_info->rx_count = 0;
        m_time_now += SEC_TO_US(10);
        m_info_index = 0;
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        expect_tx(sample_data.info.secmat.key, sample_data.beacon, sample_data.auth, &adv_packet);
        m_timer_cb(m_time_now, NULL);
        tx_complete();
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each observation period.
        TEST_ASSERT_EQUAL(10, sample_data.info.p_tx_info->interval);
    }
}

void test_beacon_build(void)
{
    setup_module();

    net_beacon_sample_data_t sample_datas[] = {SEC_NET_SAMPLE_DATA_1, SEC_NET_SAMPLE_DATA_2, SEC_NET_SAMPLE_DATA_3};

    /* single sample data */
    for (uint32_t i = 0; i < ARRAY_SIZE(sample_datas); i++)
    {
        net_beacon_sample_data_t sample_data = sample_datas[i];

        /* Test packet creation */
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);

        uint8_t buffer[NET_BEACON_BUFFER_SIZE];
        TEST_ASSERT_EQUAL(NRF_SUCCESS,
                          net_beacon_build(&sample_data.info.secmat,
                                           sample_data.iv_index,
                                           sample_data.iv_update,
                                           sample_data.key_refresh,
                                           buffer));
        TEST_ASSERT_EQUAL_HEX8(0x01, buffer[0]);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(sample_data.beacon, &buffer[1], sizeof(buffer) - 1);
    }
}
