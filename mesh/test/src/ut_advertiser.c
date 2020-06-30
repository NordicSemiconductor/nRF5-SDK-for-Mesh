/* Copyright (c) 2010 - 2020, Nordic Semiconductor ASA
 * All rights reserved.
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

#include "nrf.h"

#include "advertiser.h"
#include "utils.h"
#include "test_assert.h"

#include "radio_config_mock.h"
#include "packet_buffer_mock.h"
#include "bearer_handler_mock.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "broadcast_mock.h"
#include "rand_mock.h"
#include "bearer_event_mock.h"

#define BUF_SIZE    (512)

#define EXPECTED_ADV_BUFFER_OVERHEAD (4 /* token */ + 4 /* config+pad */ + 3 /* ble header */ + 6 /* gap addr */)

static NRF_FICR_Type m_FICR;
NRF_FICR_Type * NRF_FICR = &m_FICR;

static const adv_packet_t * mp_expected_adv_packet;
static uint32_t m_tx_timestamp_expected;
static uint32_t m_expect_tx_cb;
static uint8_t m_packet_buffer[BUF_SIZE];
static advertiser_t m_adv;
static const uint8_t m_dummy_ad_data[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
static uint32_t m_bearer_event_sequential_add_callback_cnt;
static uint32_t m_bearer_event_sequential_post_callback_cnt;

void setUp(void)
{
    radio_config_mock_Init();
    packet_buffer_mock_Init();
    bearer_handler_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
    broadcast_mock_Init();
    rand_mock_Init();
    bearer_event_mock_Init();
    m_expect_tx_cb = 0;
    mp_expected_adv_packet = NULL;
    m_bearer_event_sequential_add_callback_cnt = 0;
    m_bearer_event_sequential_post_callback_cnt = 0;
}

void tearDown(void)
{
    radio_config_mock_Verify();
    radio_config_mock_Destroy();
    packet_buffer_mock_Verify();
    packet_buffer_mock_Destroy();
    bearer_handler_mock_Verify();
    bearer_handler_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    broadcast_mock_Verify();
    broadcast_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
}


static void tx_complete_cb(advertiser_t * p_adv, nrf_mesh_tx_token_t token, timestamp_t timestamp)
{
    TEST_ASSERT_TRUE(m_expect_tx_cb > 0);
    m_expect_tx_cb--;
    TEST_ASSERT_EQUAL_PTR(&m_adv, p_adv);
    TEST_ASSERT_EQUAL_PTR(mp_expected_adv_packet->token, token);
}

static void bearer_event_sequential_add_callback(bearer_event_sequential_t * p_seq, bearer_event_callback_t callback, void * p_context, int cmock_num_calls)
{
    p_seq->callback = callback;
    p_seq->p_context = p_context;
    p_seq->event_pending = false;

    m_bearer_event_sequential_add_callback_cnt++;
}

static uint32_t bearer_event_sequential_post_callback(bearer_event_sequential_t * p_seq, int cmock_num_calls)
{
    p_seq->callback(p_seq->p_context);
    m_bearer_event_sequential_post_callback_cnt++;
    return NRF_SUCCESS;
}

static void init_advertiser(advertiser_t * p_adv)
{
    /* set FICR address values, to get a determinable advertisement address every time. */
    for (uint32_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
    {
        ((uint8_t *) NRF_FICR->DEVICEADDR)[i] = i;
    }
    NRF_FICR->DEVICEADDRTYPE = 0; /* Public address */

    memset(p_adv, 0, sizeof(advertiser_t));

    packet_buffer_init_Expect(&p_adv->buf, m_packet_buffer, BUF_SIZE);
    bearer_event_sequential_add_StubWithCallback(bearer_event_sequential_add_callback);
    advertiser_instance_init(p_adv, tx_complete_cb, m_packet_buffer, BUF_SIZE);
    bearer_event_sequential_add_StubWithCallback(NULL);
    TEST_ASSERT_EQUAL_UINT32(1, m_bearer_event_sequential_add_callback_cnt);
}

static void trigger_adv_evt(packet_buffer_packet_t * p_packet_buf, uint32_t random_offset, bool has_packet, uint32_t additional_packets, uint8_t channel_swap_index)
{
    adv_packet_t * p_adv_packet = (adv_packet_t *) p_packet_buf->packet;
    if (!has_packet)
    {
        packet_buffer_pop_ExpectAndReturn(&m_adv.buf, NULL, NRF_SUCCESS);
        packet_buffer_pop_IgnoreArg_pp_packet();
        packet_buffer_pop_ReturnThruPtr_pp_packet(&p_packet_buf);
    }

    if (m_adv.enabled && !m_adv.broadcast.active && (m_adv.tx_complete_callback != NULL))
    {
        bearer_event_sequential_pending_ExpectAndReturn(&m_adv.tx_complete_event, false);
    }

    if (p_adv_packet->config.repeats == ADVERTISER_REPEAT_INFINITE)
    {
        while (additional_packets--)
        {
            packet_buffer_packets_ready_to_pop_ExpectAndReturn(&m_adv.buf, true);

            packet_buffer_free_Expect(&m_adv.buf, p_packet_buf);

            packet_buffer_pop_ExpectAndReturn(&m_adv.buf, NULL, NRF_SUCCESS);
            packet_buffer_pop_IgnoreArg_pp_packet();
            packet_buffer_pop_ReturnThruPtr_pp_packet(&p_packet_buf);
        }
        /* we kept popping packets until there was nothing left */
        packet_buffer_packets_ready_to_pop_ExpectAndReturn(&m_adv.buf, false);
    }
    if (m_adv.config.channels.randomize_order)
    {
        for (uint32_t i = 0; i < m_adv.config.channels.count; i++)
        {
            rand_prng_get_ExpectAndReturn(NULL, channel_swap_index);
            rand_prng_get_IgnoreArg_p_prng();
        }
    }
    broadcast_send_ExpectAndReturn(&m_adv.broadcast, NRF_SUCCESS);
    rand_prng_get_ExpectAndReturn(NULL, random_offset);
    rand_prng_get_IgnoreArg_p_prng();
    m_adv.timer.state = TIMER_EVENT_STATE_IN_CALLBACK;

    m_adv.timer.cb(500, m_adv.timer.p_context);

    TEST_ASSERT_EQUAL_PTR(&p_adv_packet->packet, m_adv.broadcast.params.p_packet);
    TEST_ASSERT_EQUAL_PTR(m_adv.config.channels.channel_map, m_adv.broadcast.params.p_channels);
    TEST_ASSERT_EQUAL_PTR(m_adv.config.channels.count, m_adv.broadcast.params.channel_count);
    TEST_ASSERT_EQUAL((random_offset % ADVERTISER_INTERVAL_RANDOMIZATION_US) + m_adv.config.advertisement_interval_us, m_adv.timer.interval);
}
/*****************************************************************************
* Test functions
*****************************************************************************/

void test_init(void)
{
    rand_prng_seed_Expect(NULL);
    rand_prng_seed_IgnoreArg_p_prng();
    advertiser_init();
}

void test_instance_init(void)
{
    advertiser_t advertiser;
    init_advertiser(&advertiser);
    TEST_ASSERT_FALSE(advertiser.enabled);
    TEST_ASSERT_NULL(advertiser.broadcast.params.p_packet);
    TEST_ASSERT_EQUAL(RADIO_POWER_NRF_0DBM, advertiser.broadcast.params.radio_config.tx_power);
    TEST_ASSERT_EQUAL(37, advertiser.broadcast.params.radio_config.payload_maxlen);
    TEST_ASSERT_EQUAL(RADIO_MODE_BLE_1MBIT, advertiser.broadcast.params.radio_config.radio_mode);
    TEST_ASSERT_EQUAL(BEARER_ACCESS_ADDR_NONCONN, advertiser.broadcast.params.access_address);
    TEST_ASSERT_EQUAL_PTR(advertiser.config.channels.channel_map,
                          advertiser.broadcast.params.p_channels);
    TEST_ASSERT_EQUAL(3, advertiser.broadcast.params.channel_count);
    TEST_ASSERT_EQUAL(3, advertiser.broadcast.params.channel_count);
    TEST_ASSERT_NOT_NULL(advertiser.broadcast.params.tx_complete_cb);
    TEST_ASSERT_EQUAL_PTR(tx_complete_cb, advertiser.tx_complete_callback);
    TEST_ASSERT_EQUAL(BEARER_ADV_INT_DEFAULT_MS * 1000,
                      advertiser.config.advertisement_interval_us);
    TEST_ASSERT_EQUAL(false, advertiser.config.channels.randomize_order);
    TEST_ASSERT_EQUAL(3, advertiser.config.channels.count);
    TEST_ASSERT_NOT_NULL(advertiser.timer.cb);
    TEST_ASSERT_EQUAL_PTR(&advertiser, advertiser.timer.p_context);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(NRF_FICR->DEVICEADDR,
                                 advertiser.config.adv_addr.addr,
                                 BLE_GAP_ADDR_LEN);
    TEST_ASSERT_EQUAL(BLE_GAP_ADDR_TYPE_PUBLIC, advertiser.config.adv_addr.addr_type);
    TEST_ASSERT_EQUAL(0, advertiser.config.adv_addr.addr_id_peer);
}


void test_config(void)
{
    init_advertiser(&m_adv);

    advertiser_config_t config;
    for (uint32_t i = 0; i < BLE_GAP_ADDR_LEN; i++)
    {
        config.adv_addr.addr[i] = i;
    }
    config.adv_addr.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    config.advertisement_interval_us = 100000;
    for (uint32_t i = 0; i < BEARER_ADV_CHANNELS_MAX; i++)
    {
        config.channels.channel_map[i] = i;
    }

    config.channels.count = BEARER_ADV_CHANNELS_MAX;
    config.channels.randomize_order = false;

    advertiser_config_set(&m_adv, &config);
    TEST_ASSERT_EQUAL_MEMORY(&config.adv_addr, &m_adv.config.adv_addr, sizeof(ble_gap_addr_t));
    TEST_ASSERT_EQUAL(config.advertisement_interval_us, m_adv.config.advertisement_interval_us);
    TEST_ASSERT_EQUAL_MEMORY(&config.channels, &m_adv.config.channels, sizeof(advertiser_channels_t));

    config.channels.count = 1;
    config.channels.randomize_order = true;
    config.channels.channel_map[0] = 39;
    config.advertisement_interval_us = MS_TO_US(BEARER_ADV_INT_MAX_MS);
    advertiser_config_set(&m_adv, &config);
    TEST_ASSERT_EQUAL_MEMORY(&config.adv_addr, &m_adv.config.adv_addr, sizeof(ble_gap_addr_t));
    TEST_ASSERT_EQUAL(config.advertisement_interval_us, m_adv.config.advertisement_interval_us);
    TEST_ASSERT_EQUAL_MEMORY(&config.channels, &m_adv.config.channels, sizeof(advertiser_channels_t));

    config.advertisement_interval_us = MS_TO_US(BEARER_ADV_INT_MIN_MS);
    advertiser_config_set(&m_adv, &config);
    TEST_ASSERT_EQUAL_MEMORY(&config.adv_addr, &m_adv.config.adv_addr, sizeof(ble_gap_addr_t));
    TEST_ASSERT_EQUAL(config.advertisement_interval_us, m_adv.config.advertisement_interval_us);
    TEST_ASSERT_EQUAL_MEMORY(&config.channels, &m_adv.config.channels, sizeof(advertiser_channels_t));

    advertiser_config_t returned_config;
    advertiser_config_get(&m_adv, &returned_config);
    TEST_ASSERT_EQUAL_MEMORY(&m_adv.config, &returned_config, sizeof(advertiser_config_t));

    /* Test invalid values */
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_config_set(&m_adv, NULL));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_config_set(NULL, &config));

    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_config_get(&m_adv, NULL));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_config_get(NULL, &config));

    memset(&m_adv.config, 0, sizeof(m_adv.config));

    /* interval config */
    m_adv.timer.state = TIMER_EVENT_STATE_ADDED; /* Fake a scheduled timer to prevent reschedule */
    advertiser_interval_set(&m_adv, 100);
    TEST_ASSERT_EQUAL(100000, m_adv.config.advertisement_interval_us);
    /* corner cases, ensure no asserts: */
    advertiser_interval_set(&m_adv, BEARER_ADV_INT_MIN_MS);
    advertiser_interval_set(&m_adv, BEARER_ADV_INT_MAX_MS);
    /* schedule the timer: */
    m_adv.timer.state = TIMER_EVENT_STATE_ADDED;
    m_adv.enabled = true;
    timer_now_ExpectAndReturn(20000);
    rand_prng_get_ExpectAndReturn(NULL, 10);
    rand_prng_get_IgnoreArg_p_prng();
    timer_sch_reschedule_Expect(&m_adv.timer, MS_TO_US(BEARER_ADV_INT_MIN_MS) + 20010);
    advertiser_interval_set(&m_adv, BEARER_ADV_INT_MAX_MS);
    /* invalid params: */
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_interval_set(&m_adv, BEARER_ADV_INT_MIN_MS - 1));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_interval_set(&m_adv, BEARER_ADV_INT_MAX_MS + 1));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_interval_set(NULL, BEARER_ADV_INT_MAX_MS));

    /* channel config */
    advertiser_channels_set(&m_adv, &config.channels);
    TEST_ASSERT_EQUAL_MEMORY(&m_adv.config.channels, &config.channels, sizeof(advertiser_channels_t));
    /* corner cases, ensure no asserts: */
    config.channels.count = 1;
    config.channels.channel_map[0] = 0;
    advertiser_channels_set(&m_adv, &config.channels);
    TEST_ASSERT_EQUAL_MEMORY(&m_adv.config.channels, &config.channels, sizeof(advertiser_channels_t));
    config.channels.channel_map[0] = RADIO_NO_RF_CHANNELS - 1;
    advertiser_channels_set(&m_adv, &config.channels);
    TEST_ASSERT_EQUAL_MEMORY(&m_adv.config.channels, &config.channels, sizeof(advertiser_channels_t));
    /* invalid params: */
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_channels_set(&m_adv, NULL));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_channels_set(NULL, &config.channels));
    config.channels.channel_map[0] = RADIO_NO_RF_CHANNELS;
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_channels_set(&m_adv, &config.channels));
    config.channels.channel_map[0] = 0;
    config.channels.count = BEARER_ADV_CHANNELS_MAX + 1;
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_channels_set(&m_adv, &config.channels));

    /* addr config */
    advertiser_address_set(&m_adv, &config.adv_addr);
    TEST_ASSERT_EQUAL_MEMORY(&config.adv_addr, &m_adv.config.adv_addr, sizeof(ble_gap_addr_t));
    config.adv_addr.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC; /* Requires two highest bits of addr to be 0b11 by spec */
    advertiser_address_set(&m_adv, &config.adv_addr);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&config.adv_addr.addr, &m_adv.config.adv_addr.addr, BLE_GAP_ADDR_LEN - 1);
    TEST_ASSERT_EQUAL(0xC0 | config.adv_addr.addr[5], m_adv.config.adv_addr.addr[5]);
    TEST_ASSERT_EQUAL(1, m_adv.config.adv_addr.addr_type);
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_address_set(&m_adv, NULL));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_address_set(NULL, &config.adv_addr));

    /* TX power */
    advertiser_tx_power_set(&m_adv, RADIO_POWER_NRF_POS4DBM);
    TEST_ASSERT_EQUAL(RADIO_POWER_NRF_POS4DBM, m_adv.broadcast.params.radio_config.tx_power);
    /* invalid params: */
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_tx_power_set(NULL, RADIO_POWER_NRF_0DBM));
}

void test_packet_alloc(void)
{
    init_advertiser(&m_adv);
    static const uint16_t DUMMY_BUFFER_SIZE = 0x1234;

    packet_buffer_packet_t * p_packet_buf = (packet_buffer_packet_t *) &m_packet_buffer[0];
    p_packet_buf->size = DUMMY_BUFFER_SIZE;
    packet_buffer_reserve_ExpectAndReturn(&m_adv.buf,
                                          NULL,
                                          sizeof(m_dummy_ad_data) + EXPECTED_ADV_BUFFER_OVERHEAD,
                                          NRF_SUCCESS);
    packet_buffer_reserve_IgnoreArg_pp_packet();
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buf);

    adv_packet_t * p_packet = advertiser_packet_alloc(&m_adv, sizeof(m_dummy_ad_data));
    TEST_ASSERT_EQUAL_PTR(p_packet_buf->packet, p_packet);
    TEST_ASSERT_EQUAL(BLE_GAP_ADDR_LEN + sizeof(m_dummy_ad_data), p_packet->packet.header.length);
    TEST_ASSERT_EQUAL(BLE_PACKET_TYPE_ADV_NONCONN_IND, p_packet->packet.header.type);
    TEST_ASSERT_EQUAL(BLE_GAP_ADDR_TYPE_PUBLIC, p_packet->packet.header.addr_type);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_adv.config.adv_addr.addr,
                                 p_packet->packet.addr,
                                 BLE_GAP_ADDR_LEN);


    /* Test corner cases */
    packet_buffer_reserve_ExpectAndReturn(&m_adv.buf,
                                          NULL,
                                          0 + EXPECTED_ADV_BUFFER_OVERHEAD,
                                          NRF_SUCCESS);
    packet_buffer_reserve_IgnoreArg_pp_packet();
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buf);
    TEST_ASSERT_EQUAL_PTR(p_packet_buf->packet, advertiser_packet_alloc(&m_adv, 0));

    packet_buffer_reserve_ExpectAndReturn(&m_adv.buf,
                                          NULL,
                                          BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH + EXPECTED_ADV_BUFFER_OVERHEAD,
                                          NRF_SUCCESS);
    packet_buffer_reserve_IgnoreArg_pp_packet();
    packet_buffer_reserve_ReturnThruPtr_pp_packet(&p_packet_buf);
    TEST_ASSERT_EQUAL_PTR(p_packet_buf->packet, advertiser_packet_alloc(&m_adv, BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH));

    packet_buffer_reserve_ExpectAndReturn(&m_adv.buf,
                                          NULL,
                                          BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH + EXPECTED_ADV_BUFFER_OVERHEAD,
                                          NRF_ERROR_NO_MEM);
    packet_buffer_reserve_IgnoreArg_pp_packet();
    TEST_ASSERT_EQUAL_PTR(NULL, advertiser_packet_alloc(&m_adv, BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH));

    /* invalid params */
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_packet_alloc(NULL, sizeof(m_dummy_ad_data)));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_packet_alloc(&m_adv, BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH + 1));

}

void test_packet_send(void)
{
    static const uint16_t DUMMY_BUFFER_SIZE = 0x1234;
    init_advertiser(&m_adv);
    packet_buffer_packet_t * p_packet_buf = (packet_buffer_packet_t *) &m_packet_buffer[0];
    p_packet_buf->size = DUMMY_BUFFER_SIZE;
    adv_packet_t * p_packet = (adv_packet_t *) p_packet_buf->packet;
    memcpy(&p_packet->packet.payload[0], m_dummy_ad_data, sizeof(m_dummy_ad_data));

    /* mangle rfu fields, to ensure that they're set to 0 before being committed. */
    p_packet->packet.header._rfu1 = 1;
    p_packet->packet.header._rfu2 = 1;
    p_packet->packet.header._rfu3 = 1;

    p_packet->config.repeats = 1;
    packet_buffer_commit_Expect(&m_adv.buf,
                                p_packet_buf,
                                DUMMY_BUFFER_SIZE);
    advertiser_packet_send(&m_adv, p_packet);

    TEST_ASSERT_NOT_NULL(m_adv.timer.cb);
    TEST_ASSERT_EQUAL(0, m_adv.timer.interval);
    TEST_ASSERT_EQUAL(TIMER_EVENT_STATE_UNUSED, m_adv.timer.state);
    TEST_ASSERT_EQUAL(0, p_packet->packet.header._rfu1);
    TEST_ASSERT_EQUAL(0, p_packet->packet.header._rfu2);
    TEST_ASSERT_EQUAL(0, p_packet->packet.header._rfu3);
    /* ensure packet hasn't been altered by module: */
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_dummy_ad_data, p_packet->packet.payload, sizeof(m_dummy_ad_data));

    /* Commit packet again, but while the advertiser is enabled. Should leave duplicate detection to
     * packet_buffer module. */
    m_adv.enabled = true;
    packet_buffer_commit_Expect(&m_adv.buf,
                                p_packet_buf,
                                DUMMY_BUFFER_SIZE);
    timer_now_ExpectAndReturn(1000);
    rand_prng_get_ExpectAndReturn(NULL, 1000);
    rand_prng_get_IgnoreArg_p_prng();
    timer_sch_reschedule_Expect(&m_adv.timer, MS_TO_US(BEARER_ADV_INT_MIN_MS) + 2000);

    advertiser_packet_send(&m_adv, p_packet);
    m_adv.timer.state = TIMER_EVENT_STATE_ADDED;

    /* Commit packet again, should not reschedule the timer, as it's already scheduled. */
    packet_buffer_commit_Expect(&m_adv.buf,
                                p_packet_buf,
                                DUMMY_BUFFER_SIZE);
    advertiser_packet_send(&m_adv, p_packet);

    /* fails */
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_packet_send(NULL, p_packet));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_packet_send(&m_adv, NULL));
    p_packet->config.repeats = 0; /* illegal */
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_packet_send(&m_adv, p_packet));

    /* Check the divisor of the first schedule, by returning value bigger than the divisor. */
    p_packet->config.repeats = 1;
    m_adv.timer.state = TIMER_EVENT_STATE_UNUSED;
    packet_buffer_commit_Expect(&m_adv.buf,
                                p_packet_buf,
                                DUMMY_BUFFER_SIZE);
    timer_now_ExpectAndReturn(1000);
    rand_prng_get_ExpectAndReturn(NULL, MS_TO_US(BEARER_ADV_INT_MIN_MS) + ADVERTISER_INTERVAL_RANDOMIZATION_US + 2000);
    rand_prng_get_IgnoreArg_p_prng();
    timer_sch_reschedule_Expect(&m_adv.timer, MS_TO_US(BEARER_ADV_INT_MIN_MS) + 3000);
    advertiser_packet_send(&m_adv, p_packet);
}

void test_packet_discard(void)
{
    init_advertiser(&m_adv);
    packet_buffer_packet_t * p_packet_buf = (packet_buffer_packet_t *) &m_packet_buffer[0];
    adv_packet_t * p_packet = (adv_packet_t *) p_packet_buf->packet;

    packet_buffer_free_Expect(&m_adv.buf, p_packet_buf);
    advertiser_packet_discard(&m_adv, p_packet);
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_packet_discard(NULL, p_packet));
    TEST_NRF_MESH_ASSERT_EXPECT(advertiser_packet_discard(&m_adv, NULL));
}

void test_advertising(void)
{
    init_advertiser(&m_adv);
    packet_buffer_packet_t * p_packet_buf = (packet_buffer_packet_t *)  m_packet_buffer;
    adv_packet_t * p_adv_packet = (adv_packet_t *) p_packet_buf->packet;
    m_adv.enabled = true;
    p_adv_packet->config.repeats = 1;

    trigger_adv_evt(p_packet_buf, 11000, false, 0, 0);
    TEST_ASSERT_EQUAL(0, p_adv_packet->config.repeats);

    packet_buffer_mock_Verify();

    /* Go through same procedure with a packet that repeats several times. */
    m_adv.p_packet = NULL;
    p_adv_packet->config.repeats = 2;
    trigger_adv_evt(p_packet_buf, 6000, false, 0, 0);
    TEST_ASSERT_EQUAL(1, p_adv_packet->config.repeats);

    packet_buffer_mock_Verify();

    m_adv.p_packet = p_adv_packet;
    p_adv_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;
    trigger_adv_evt(p_packet_buf, 123456789, true, 0, 0);
    TEST_ASSERT_EQUAL(ADVERTISER_REPEAT_INFINITE, p_adv_packet->config.repeats);

    packet_buffer_mock_Verify();

    /* Have multiple infinity packets lined up. */
    trigger_adv_evt(p_packet_buf, 123456789, true, 1, 0);
    TEST_ASSERT_EQUAL(ADVERTISER_REPEAT_INFINITE, p_adv_packet->config.repeats);

    packet_buffer_mock_Verify();

    m_adv.timer.interval = 100000;
    m_adv.enabled = false; /* should just ignore callback and don't reschedule */
    m_adv.timer.cb(500, m_adv.timer.p_context);
    TEST_ASSERT_EQUAL(0, m_adv.timer.interval);

    m_adv.enabled = true;
    m_adv.broadcast.active = true; /* shouldn't do anything except reschedule */
    m_adv.timer.interval = 100000;
    rand_prng_get_ExpectAndReturn(NULL, 1000);
    rand_prng_get_IgnoreArg_p_prng();
    m_adv.timer.cb(500, m_adv.timer.p_context);
    TEST_ASSERT_EQUAL(1000 + m_adv.config.advertisement_interval_us, m_adv.timer.interval);

    /* Randomize channels */
    struct
    {
        uint8_t swap_index;
        uint8_t expected_channels[3];
    } channel_swap[] =
    {
        {0, {39, 37, 38}}, /* should swap 0<->0, 0<->1, 0<->2 */
        {1, {38, 39, 37}}, /* should swap 1<->0, 1<->1, 1<->2 */
        {2, {39, 37, 38}}, /* should swap 2<->0, 2<->1, 2<->2 */
    };
    for (uint32_t i = 0; i < ARRAY_SIZE(channel_swap); i++)
    {
        /* reset channels */
        m_adv.config.channels.channel_map[0] = 37;
        m_adv.config.channels.channel_map[1] = 38;
        m_adv.config.channels.channel_map[2] = 39;

        m_adv.broadcast.active = false;
        m_adv.config.channels.randomize_order = true;
        p_adv_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;
        trigger_adv_evt(p_packet_buf, 123456789, true, false, channel_swap[i].swap_index);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(channel_swap[i].expected_channels, m_adv.config.channels.channel_map, 3);
    }
}

void test_broadcast_callback(void)
{
    init_advertiser(&m_adv);
    packet_buffer_packet_t * p_packet_buf = (packet_buffer_packet_t *)  m_packet_buffer;
    adv_packet_t * p_adv_packet = (adv_packet_t *) p_packet_buf->packet;
    m_adv.p_packet = p_adv_packet;

    /* Do the tx callback, expect it do do nothing, as all transmissions aren't done yet. */
    p_adv_packet->config.repeats = 1;
    m_tx_timestamp_expected = 10000;
    m_bearer_event_sequential_post_callback_cnt = 0;
    bearer_event_sequential_post_StubWithCallback(bearer_event_sequential_post_callback);
    m_adv.broadcast.params.tx_complete_cb(&m_adv.broadcast.params, m_tx_timestamp_expected);
    bearer_event_sequential_post_StubWithCallback(NULL);
    TEST_ASSERT_EQUAL(0, m_bearer_event_sequential_post_callback_cnt);
    TEST_ASSERT_EQUAL(0, m_expect_tx_cb);
    /* with infinite repeats: Expect a TX event, but don't kill the packet */
    m_expect_tx_cb = 1;
    mp_expected_adv_packet = p_adv_packet;
    p_adv_packet->config.repeats = ADVERTISER_REPEAT_INFINITE;
    m_bearer_event_sequential_post_callback_cnt = 0;
    bearer_event_sequential_post_StubWithCallback(bearer_event_sequential_post_callback);
    m_adv.broadcast.params.tx_complete_cb(&m_adv.broadcast.params, m_tx_timestamp_expected);
    bearer_event_sequential_post_StubWithCallback(NULL);
    TEST_ASSERT_EQUAL(1, m_bearer_event_sequential_post_callback_cnt);
    TEST_ASSERT_EQUAL(0, m_expect_tx_cb);
    packet_buffer_mock_Verify();
    /* Do the tx callback, expect it to forward to user and go idle */
    p_adv_packet->config.repeats = 0;
    m_expect_tx_cb = 1;
    mp_expected_adv_packet = p_adv_packet;
    packet_buffer_free_Expect(&m_adv.buf, p_packet_buf);
    m_bearer_event_sequential_post_callback_cnt = 0;
    bearer_event_sequential_post_StubWithCallback(bearer_event_sequential_post_callback);
    m_adv.broadcast.params.tx_complete_cb(&m_adv.broadcast.params, m_tx_timestamp_expected);
    bearer_event_sequential_post_StubWithCallback(NULL);
    TEST_ASSERT_EQUAL(1, m_bearer_event_sequential_post_callback_cnt);
    TEST_ASSERT_EQUAL(0, m_expect_tx_cb);
    packet_buffer_mock_Verify();
}

void test_enable_disable(void)
{
    init_advertiser(&m_adv);

    /* no active packet, but queue's not empty */
    m_adv.enabled = false;
    m_adv.p_packet = NULL; // have to check whether we can pop
    packet_buffer_can_pop_ExpectAndReturn(&m_adv.buf, true);
    timer_now_ExpectAndReturn(1000);
    rand_prng_get_ExpectAndReturn(NULL, 5000);
    rand_prng_get_IgnoreArg_p_prng();
    timer_sch_reschedule_Expect(&m_adv.timer, MS_TO_US(BEARER_ADV_INT_MIN_MS) + 1000 + 5000);
    advertiser_enable(&m_adv);
    TEST_ASSERT_TRUE(m_adv.enabled);

    /* No active packets, empty queue */
    m_adv.enabled = false;
    packet_buffer_can_pop_ExpectAndReturn(&m_adv.buf, false);
    advertiser_enable(&m_adv);
    TEST_ASSERT_TRUE(m_adv.enabled);

    /* Active packet */
    m_adv.enabled = false;
    adv_packet_t active_packet;
    m_adv.p_packet = &active_packet;
    timer_now_ExpectAndReturn(1000);
    rand_prng_get_ExpectAndReturn(NULL, 5000);
    rand_prng_get_IgnoreArg_p_prng();
    timer_sch_reschedule_Expect(&m_adv.timer, MS_TO_US(BEARER_ADV_INT_MIN_MS) + 1000 + 5000);
    advertiser_enable(&m_adv);
    TEST_ASSERT_TRUE(m_adv.enabled);


    m_adv.enabled = true;
    advertiser_enable(&m_adv);
    TEST_ASSERT_TRUE(m_adv.enabled);

    timer_sch_abort_Expect(&m_adv.timer);
    advertiser_disable(&m_adv);
    TEST_ASSERT_FALSE(m_adv.enabled);
    /* Do the same thing even if it's already disabled */
    timer_sch_abort_Expect(&m_adv.timer);
    advertiser_disable(&m_adv);
    TEST_ASSERT_FALSE(m_adv.enabled);
}

void test_flush(void)
{
    init_advertiser(&m_adv);

    adv_packet_t curr_packet;
    curr_packet.config.repeats = 8;

    m_adv.enabled = true;
    m_adv.p_packet = &curr_packet;
    packet_buffer_flush_Expect(&m_adv.buf);
    advertiser_flush(&m_adv);
    TEST_ASSERT_EQUAL(0, curr_packet.config.repeats);

    curr_packet.config.repeats = ADVERTISER_REPEAT_INFINITE;
    packet_buffer_flush_Expect(&m_adv.buf);
    advertiser_flush(&m_adv);
    TEST_ASSERT_EQUAL(0, curr_packet.config.repeats);

    curr_packet.config.repeats = 0;
    packet_buffer_flush_Expect(&m_adv.buf);
    advertiser_flush(&m_adv);
    TEST_ASSERT_EQUAL(0, curr_packet.config.repeats);

    curr_packet.config.repeats = 1;
    packet_buffer_flush_Expect(&m_adv.buf);
    advertiser_flush(&m_adv);
    TEST_ASSERT_EQUAL(0, curr_packet.config.repeats);

    m_adv.p_packet = NULL; //!
    packet_buffer_flush_Expect(&m_adv.buf);
    advertiser_flush(&m_adv);

    /* should do the flush regardless of status */
    m_adv.enabled = false;
    packet_buffer_flush_Expect(&m_adv.buf);
    advertiser_flush(&m_adv);
}
