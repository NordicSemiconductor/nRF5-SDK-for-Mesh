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

#include "nordic_common.h"
#include "test_assert.h"

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

static uint8_t * mp_expected_net_id;
static nrf_mesh_beacon_info_t ** mpp_infos;
static uint32_t m_info_count;
static uint32_t m_info_index;
static timer_sch_callback_t m_timer_cb;
static uint32_t m_rand_value;
static uint32_t m_time_now;

void setUp(void)
{
    mp_expected_net_id = NULL;
    m_info_index = 0;
    m_info_count = 0;
    mpp_infos = NULL;
    m_timer_cb = NULL;
    m_time_now = 0;
    beacon_mock_Init();
    net_state_mock_Init();
    timer_scheduler_mock_Init();
    timer_mock_Init();
    enc_mock_Init();
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
    TEST_ASSERT_EQUAL(SEC_TO_US(NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS / 2), p_event->interval);
    TEST_ASSERT_TRUE(m_time_now + SEC_TO_US(NRF_MESH_BEACON_SECURE_NET_BCAST_INTERVAL_SECONDS / 2) > p_event->timestamp);
    TEST_ASSERT_TRUE(m_time_now < p_event->timestamp);
    m_timer_cb = p_event->cb;
}

void setup_module(void)
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

    net_beacon_init();
    TEST_ASSERT_NOT_NULL(m_timer_cb);
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
        tx_info.tx_interval_seconds = 0;
        tx_info.tx_timestamp = 0;
        tx_info.rx_count = 0;
        sample_data.info.p_tx_info = &tx_info;

        nrf_mesh_beacon_info_t * p_info[] = {&sample_data.info};
        mpp_infos = p_info;
        m_info_count = 1;

        m_info_index = 0;
        m_time_now = SEC_TO_US(5);

        /* Test packet creation */
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        beacon_tx_ExpectWithArrayAndReturn(BEACON_TYPE_SEC_NET_BCAST, sample_data.beacon, 21, 21, 1, NRF_SUCCESS);

        /* Call the timeout */
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

        /* Run again - this time, we haven't waited long enough, and it won't send a beacon */
        m_info_index = 0;
        m_time_now += SEC_TO_US(5);
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

        beacon_mock_Verify();

        /* Run again, fail bearer_tx */
        m_info_index = 0;
        m_time_now += SEC_TO_US(5) + 1;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        beacon_tx_ExpectWithArrayAndReturn(BEACON_TYPE_SEC_NET_BCAST, sample_data.beacon, 21, 21, 1, NRF_ERROR_NO_MEM);

        /* Call the timeout */
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

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
    uint8_t beacon_data[13][NETWORKS];
    nrf_mesh_beacon_tx_info_t tx_info[NETWORKS];
    nrf_mesh_beacon_info_t * p_info[NETWORKS];

    memset(&info, 0, sizeof(info));
    memset(&tx_info, 0, sizeof(tx_info));

    for (uint32_t i = 0; i < NETWORKS; i++)
    {
        tx_info[i].rx_count = 0;
        tx_info[i].tx_timestamp = 0;
        tx_info[i].tx_interval_seconds = 0;
        info[i].p_tx_info = &tx_info[i];
        p_info[i] = &info[i];
        memset(&beacon_data[0][i], i, 13);
    }
    mpp_infos = p_info;
    m_info_count = NETWORKS;

    m_info_index = 0;
    m_time_now = SEC_TO_US(5);

    /* Test packet creation */
    for (uint32_t j = 0; j < NETWORKS; j++)
    {
        enc_aes_cmac_ExpectWithArray(info[j].secmat.key, NRF_MESH_KEY_SIZE, &beacon_data[0][j], 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_data();
        enc_aes_cmac_IgnoreArg_p_result();
        beacon_tx_ExpectAndReturn(BEACON_TYPE_SEC_NET_BCAST, NULL, 21, 1, NRF_SUCCESS);
        beacon_tx_IgnoreArg_p_payload();
    }

    /* Call the timeout */
    net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
    net_state_iv_update_get_ExpectAndReturn(NET_STATE_IV_UPDATE_NORMAL);
    m_timer_cb(m_time_now, NULL);
    enc_mock_Verify();
    beacon_mock_Verify();

    /* Run again - this time, we haven't waited long enough, and it won't send any beacons */
    m_info_index = 0;
    m_time_now += SEC_TO_US(5);
    net_state_beacon_iv_index_get_ExpectAndReturn(0x12345678);
    net_state_iv_update_get_ExpectAndReturn(NET_STATE_IV_UPDATE_NORMAL);
    m_timer_cb(m_time_now, NULL);

    beacon_mock_Verify();
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
        tx_info.tx_interval_seconds = 0;
        tx_info.tx_timestamp = 0;
        tx_info.rx_count = 0;
        sample_data.info.p_tx_info = &tx_info;

        nrf_mesh_beacon_info_t * p_info[] = {&sample_data.info};
        mpp_infos = p_info;
        m_info_count = 1;
        mp_expected_net_id = sample_data.info.secmat.net_id;

        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        net_state_beacon_received_Expect(sample_data.iv_index, sample_data.iv_update, sample_data.key_refresh);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(1, sample_data.info.p_tx_info->rx_count);

        /* Try again, should bump the tx count */
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        net_state_beacon_received_Expect(sample_data.iv_index, sample_data.iv_update, sample_data.key_refresh);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(2, sample_data.info.p_tx_info->rx_count);

        /* Try again without permitting IV update. Shouldn't forward to net state module, but should count the RX */
        sample_data.info.iv_update_permitted = false;
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
        TEST_ASSERT_EQUAL(3, sample_data.info.p_tx_info->rx_count);


        /* Don't roll over the rx count */
        m_info_index = 0;
        sample_data.info.iv_update_permitted = true;
        sample_data.info.p_tx_info->rx_count = 0xFFFF;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        net_state_beacon_received_Expect(sample_data.iv_index, sample_data.iv_update, sample_data.key_refresh);
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

    mp_expected_net_id = sample_data.info.secmat.net_id;

    m_info_index = 0;
    for (uint32_t j = 0; j < ARRAY_SIZE(p_info); j++)
    {
        enc_aes_cmac_ExpectWithArray(p_info[j]->secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(stored_beacons[j].auth, 16);
    }
    net_state_beacon_received_Expect(sample_data.iv_index, sample_data.iv_update, sample_data.key_refresh);
    net_state_beacon_received_Expect(sample_data.iv_index, sample_data.iv_update, sample_data.key_refresh);
    net_beacon_packet_in(sample_data.beacon, sizeof(sample_data.beacon), NULL);
    TEST_ASSERT_EQUAL(1, tx_infos[0].rx_count);
    TEST_ASSERT_EQUAL(0, tx_infos[1].rx_count); /* Had invalid auth, didn't get a match */
    TEST_ASSERT_EQUAL(1, tx_infos[2].rx_count);
}

void test_beacon_state_change(void)
{
    setup_module();
    TEST_ASSERT_EQUAL(true, net_beacon_state_get());

    /* Enable the beacon while already enabled: */
    net_beacon_state_set(true);
    TEST_ASSERT_EQUAL(true, net_beacon_state_get());

    /* Disable the beacon while running: */
    timer_sch_abort_Expect(NULL);
    timer_sch_abort_IgnoreArg_p_timer_evt();
    net_beacon_state_set(false);
    TEST_ASSERT_EQUAL(false, net_beacon_state_get());

    /* Disable the beacon while already disabled: */
    net_beacon_state_set(false);
    TEST_ASSERT_EQUAL(false, net_beacon_state_get());

    /* Enable the beacon while disabled: */
    timer_now_ExpectAndReturn(1000);
    timer_sch_reschedule_Expect(NULL, 0);
    timer_sch_reschedule_IgnoreArg_p_timer_evt();
    timer_sch_reschedule_IgnoreArg_new_timestamp();
    net_beacon_state_set(true);
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
        tx_info.tx_interval_seconds = 0;
        tx_info.tx_timestamp = 0;
        tx_info.rx_count = 0;
        sample_data.info.p_tx_info = &tx_info;

        nrf_mesh_beacon_info_t * p_info[] = {&sample_data.info};
        mpp_infos = p_info;
        m_info_count = 1;

        /* Always transmit a beacon on the first timeout. */
        m_time_now = SEC_TO_US(5);
        sample_data.info.p_tx_info->rx_count = 1; // Received 1 beacon this 5 second period, should transmit after 30 seconds.
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        beacon_tx_ExpectWithArrayAndReturn(BEACON_TYPE_SEC_NET_BCAST, sample_data.beacon, 21, 21, 1, NRF_SUCCESS);

        /* Call the timeout */
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);
        /* we saw 1 beacon in the first 5 seconds, expecting 30 second interval for the second. */
        TEST_ASSERT_EQUAL(30, sample_data.info.p_tx_info->tx_interval_seconds);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each transmit.
        TEST_ASSERT_EQUAL(m_time_now, sample_data.info.p_tx_info->tx_timestamp); // should be reset on each transmit.

        /* Nothing happens until T=5+30 */
        m_time_now = SEC_TO_US(20);
        m_info_index = 0;
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

        m_time_now = SEC_TO_US(34);
        m_info_index = 0;
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

        /* T > 35, transmit! */
        m_time_now = SEC_TO_US(35);
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        beacon_tx_ExpectWithArrayAndReturn(BEACON_TYPE_SEC_NET_BCAST, sample_data.beacon, 21, 21, 1, NRF_SUCCESS);
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);
        /* Didn't receive any beacons this period, bring interval down to 10 seconds: */
        TEST_ASSERT_EQUAL(10, sample_data.info.p_tx_info->tx_interval_seconds);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each transmit.
        TEST_ASSERT_EQUAL(m_time_now, sample_data.info.p_tx_info->tx_timestamp); // should be reset on each transmit.

        /* Nothing happens until T=35+10 */
        m_time_now = SEC_TO_US(40);
        m_info_index = 0;
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

        /* T > 45, transmit! */
        sample_data.info.p_tx_info->rx_count = 2;
        m_time_now = SEC_TO_US(45);
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        beacon_tx_ExpectWithArrayAndReturn(BEACON_TYPE_SEC_NET_BCAST, sample_data.beacon, 21, 21, 1, NRF_SUCCESS);
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);
        /* Received 2 beacons over 10 seconds, bring interval up to 30: */
        TEST_ASSERT_EQUAL(30, sample_data.info.p_tx_info->tx_interval_seconds);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each transmit.
        TEST_ASSERT_EQUAL(m_time_now, sample_data.info.p_tx_info->tx_timestamp); // should be reset on each transmit.

        /* Nothing happens until T=45+30 */
        m_time_now = SEC_TO_US(74);
        m_info_index = 0;
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);

        /* T > 75, transmit! */
        sample_data.info.p_tx_info->rx_count = 3;
        m_time_now = SEC_TO_US(75);
        m_info_index = 0;
        enc_aes_cmac_ExpectWithArray(sample_data.info.secmat.key, NRF_MESH_KEY_SIZE, sample_data.beacon, 13, 13, NULL, 0);
        enc_aes_cmac_IgnoreArg_p_result();
        enc_aes_cmac_ReturnMemThruPtr_p_result(sample_data.auth, 16);
        beacon_tx_ExpectWithArrayAndReturn(BEACON_TYPE_SEC_NET_BCAST, sample_data.beacon, 21, 21, 1, NRF_SUCCESS);
        net_state_beacon_iv_index_get_ExpectAndReturn(sample_data.iv_index);
        net_state_iv_update_get_ExpectAndReturn(sample_data.iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL);
        m_timer_cb(m_time_now, NULL);
        /* Received 3 beacons over 30 seconds, interval is 20: */
        TEST_ASSERT_EQUAL(20, sample_data.info.p_tx_info->tx_interval_seconds);
        TEST_ASSERT_EQUAL(0, sample_data.info.p_tx_info->rx_count); // should be reset on each transmit.
        TEST_ASSERT_EQUAL(m_time_now, sample_data.info.p_tx_info->tx_timestamp); // should be reset on each transmit.

    }
}
