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
#include "core_tx_mock.h"
#include "bearer_event_mock.h"
#include "broadcast_mock.h"
#include "advertiser_mock.h"
#include "test_assert.h"
#include "timer_scheduler_mock.h"
#include "radio_config.h"
#include "timer_mock.h"

#include "core_tx_friend.h"

#define TX_TOKEN    (0x12345678)
#define TX_TIMESTAMP    (0x56781234)
#define NET_PACKET_LEN  29

#define EXPECTED_SCHEDULE_OVERHEAD  (RADIO_RAMPUP_TIME - TIMER_NOW_MAX_ERROR_US)

static ble_gap_addr_t m_adv_addr = {
    .addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC,
    .addr = {1, 2, 3, 4, 5, 6}
};
static uint8_t m_net_packet_buf[NET_PACKET_LEN];

static bearer_event_sequential_t * mp_tx_complete_event;
static broadcast_complete_cb_t m_broadcast_complete_cb;
static broadcast_t * mp_broadcast;
static core_tx_bearer_t * mp_bearer;
static core_tx_friend_t m_friend;

static const network_packet_metadata_t m_net_meta = {
    .control_packet = true,
    .ttl = 0,
    .src = 0x0001,
    .dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST,
    .dst.value = 0x0002,
};
static const core_tx_alloc_params_t m_alloc_params = {
    .net_packet_len = NET_PACKET_LEN,
    .p_metadata = &m_net_meta,
    .token = TX_TOKEN,
    .role = CORE_TX_ROLE_ORIGINATOR,
    .bearer_selector = CORE_TX_BEARER_TYPE_FRIEND,
};

void setUp(void)
{
    core_tx_mock_Init();
    bearer_event_mock_Init();
    broadcast_mock_Init();
    advertiser_mock_Init();
    timer_scheduler_mock_Init();
    timer_mock_Init();
    mp_tx_complete_event = NULL;
    mp_bearer = NULL;
    m_broadcast_complete_cb = NULL;
    mp_broadcast = NULL;
    memset(m_net_packet_buf, 0xab, sizeof(m_net_packet_buf));
}

void tearDown(void)
{
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    broadcast_mock_Verify();
    broadcast_mock_Destroy();
    advertiser_mock_Verify();
    advertiser_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
}

static void bearer_event_sequential_add_mock(bearer_event_sequential_t * p_seq,
                                             bearer_event_callback_t callback,
                                             void * p_context,
                                             int calls)
{
    p_seq->callback = callback;
    p_seq->p_context = p_context;
    mp_tx_complete_event = p_seq;
}

static void core_tx_bearer_add_mock(core_tx_bearer_t * p_bearer,
                                    const core_tx_bearer_interface_t * p_if,
                                    core_tx_bearer_type_t type,
                                    int calls)
{
    TEST_ASSERT_NOT_NULL(p_bearer);
    TEST_ASSERT_NOT_NULL(p_if);
    TEST_ASSERT_NOT_NULL(p_if->packet_alloc);
    TEST_ASSERT_NOT_NULL(p_if->packet_send);
    TEST_ASSERT_NOT_NULL(p_if->packet_discard);
    TEST_ASSERT_EQUAL(CORE_TX_BEARER_TYPE_FRIEND, type);
    p_bearer->p_interface = p_if;
    mp_bearer = p_bearer;
}

static uint32_t broadcast_send_mock(broadcast_t * p_broadcast, int calls)
{
    TEST_ASSERT_EQUAL(BEARER_ACCESS_ADDR_DEFAULT, p_broadcast->params.access_address);
    const uint8_t expect_channels[] = NRF_MESH_ADV_CHAN_DEFAULT;
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expect_channels, p_broadcast->params.p_channels, ARRAY_SIZE(expect_channels));
    TEST_ASSERT_EQUAL(p_broadcast->params.channel_count, ARRAY_SIZE(expect_channels));
    TEST_ASSERT_EQUAL(RADIO_POWER_NRF_0DBM, p_broadcast->params.radio_config.tx_power);
    TEST_ASSERT_EQUAL(RADIO_MODE_BLE_1MBIT, p_broadcast->params.radio_config.radio_mode);
    TEST_ASSERT_EQUAL(RADIO_CONFIG_ADV_MAX_PAYLOAD_SIZE, p_broadcast->params.radio_config.payload_maxlen);

    TEST_ASSERT_NOT_NULL(p_broadcast->params.p_packet);
    TEST_ASSERT_EQUAL(BLE_PACKET_TYPE_ADV_NONCONN_IND, p_broadcast->params.p_packet->header.type);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(m_adv_addr.addr, p_broadcast->params.p_packet->addr, BLE_GAP_ADDR_LEN);
    TEST_ASSERT_EQUAL(BLE_ADV_PACKET_OVERHEAD + sizeof(ble_ad_data_t) + NET_PACKET_LEN, p_broadcast->params.p_packet->header.length);

    const ble_ad_header_t expect_header = {
        .length = NET_PACKET_LEN + BLE_AD_DATA_OVERHEAD,
        .type = AD_TYPE_MESH,
    };
    const ble_ad_data_t * p_ad = (const ble_ad_data_t *) p_broadcast->params.p_packet->payload;

    TEST_ASSERT_EQUAL_MEMORY(&expect_header, p_ad, sizeof(expect_header));
    TEST_ASSERT_EQUAL_MEMORY(m_net_packet_buf, p_ad->data, NET_PACKET_LEN);

    m_broadcast_complete_cb = p_broadcast->params.tx_complete_cb;
    mp_broadcast = p_broadcast;

    return NRF_SUCCESS;
}

/*****************************************************************************
* Test functions
*****************************************************************************/

void test_init(void)
{
    bearer_event_sequential_add_StubWithCallback(bearer_event_sequential_add_mock);
    advertiser_address_default_get_ExpectAnyArgs();
    advertiser_address_default_get_ReturnThruPtr_p_addr(&m_adv_addr);
    core_tx_bearer_add_StubWithCallback(core_tx_bearer_add_mock);


    core_tx_friend_init(&m_friend, TX_TOKEN, RADIO_POWER_NRF_0DBM);
    TEST_ASSERT_EQUAL(TX_TOKEN, m_friend.token);
    TEST_ASSERT_NOT_NULL(m_friend.tx_complete_event.callback);
    TEST_ASSERT_NOT_NULL(m_friend.tx_timer.cb);

    TEST_ASSERT_NOT_NULL(mp_tx_complete_event);
    TEST_ASSERT_EQUAL_PTR(&m_friend.bearer, mp_bearer);
    TEST_ASSERT_FALSE(m_friend.enabled);
}

void test_enable(void)
{
    test_init();

    core_tx_friend_enable(&m_friend);
    TEST_ASSERT_EQUAL(CORE_TX_FRIEND_STATE_READY, m_friend.state);
    // second call has no effect:
    core_tx_friend_enable(&m_friend);
    TEST_ASSERT_EQUAL(CORE_TX_FRIEND_STATE_READY, m_friend.state);
    TEST_ASSERT_TRUE(m_friend.enabled);
}

void test_alloc(void)
{
    test_enable();
    core_tx_alloc_params_t alloc_params = m_alloc_params;

    // Allocate a packet on all bearers, should reject it
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_ALLOW_ALL;
    alloc_params.token = TX_TOKEN; // right TX token, shouldn't matter
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_REJECTED, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // should accept packet for friend only, if the token is right
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_FRIEND;
    alloc_params.token = TX_TOKEN + 1; // wrong
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_REJECTED, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));
    alloc_params.token = TX_TOKEN; // right, should accept
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // Can only allocate one packet at the time
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_NO_MEM, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));
}

void test_dealloc(void)
{
    test_alloc();

    // successfully discard the current packet
    mp_bearer->p_interface->packet_discard(mp_bearer);

    // Should assert if we try it again
    TEST_NRF_MESH_ASSERT_EXPECT(mp_bearer->p_interface->packet_discard(mp_bearer));
}

void test_send(void)
{
    test_alloc();

    // commit:
    mp_bearer->p_interface->packet_send(mp_bearer, m_net_packet_buf, ARRAY_SIZE(m_net_packet_buf));

    // All operations other than tx complete should fail now
    TEST_NRF_MESH_ASSERT_EXPECT(mp_bearer->p_interface->packet_send(mp_bearer, m_net_packet_buf, ARRAY_SIZE(m_net_packet_buf)));
    TEST_NRF_MESH_ASSERT_EXPECT(mp_bearer->p_interface->packet_discard(mp_bearer));
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_NO_MEM, mp_bearer->p_interface->packet_alloc(mp_bearer, &m_alloc_params));

    // Transmit:

    // successfully send the current packet
    mp_broadcast = NULL;
    broadcast_send_StubWithCallback(broadcast_send_mock);

    m_friend.tx_timer.cb(0, m_friend.tx_timer.p_context);
    TEST_ASSERT_EQUAL_PTR(&m_friend.broadcast, mp_broadcast);

    // TX complete:

    // fail:
    bearer_event_sequential_post_ExpectAndReturn(mp_tx_complete_event, NRF_ERROR_FORBIDDEN);
    TEST_NRF_MESH_ASSERT_EXPECT(m_broadcast_complete_cb(&mp_broadcast->params, TX_TIMESTAMP));
    // success:
    bearer_event_sequential_post_ExpectAndReturn(mp_tx_complete_event, NRF_SUCCESS);
    m_broadcast_complete_cb(&mp_broadcast->params, TX_TIMESTAMP);

    // call event handler:
    core_tx_complete_Expect(mp_bearer, CORE_TX_ROLE_ORIGINATOR, TX_TIMESTAMP, TX_TOKEN);
    mp_tx_complete_event->callback(mp_tx_complete_event->p_context);

    // Now, we're able to allocate again:
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, mp_bearer->p_interface->packet_alloc(mp_bearer, &m_alloc_params));
}

void test_disable(void)
{
    test_enable();

    core_tx_friend_disable(&m_friend);
    TEST_ASSERT_FALSE(m_friend.enabled);

    // while disabled, we shouldn't be able to allocate any packets (even with the right parameters):
    core_tx_alloc_params_t alloc_params = m_alloc_params;
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_FRIEND;
    alloc_params.token = TX_TOKEN;
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_REJECTED, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    timer_scheduler_mock_Verify();

    // allocate a packet, then disable. Should assert, as we're not handling this case now.
    test_alloc();
    TEST_NRF_MESH_ASSERT_EXPECT(core_tx_friend_disable(&m_friend));

    // allocate and commit a packet, should go back to ready
    test_alloc();
    mp_bearer->p_interface->packet_send(mp_bearer, m_net_packet_buf, ARRAY_SIZE(m_net_packet_buf));

    timer_sch_abort_Expect(&m_friend.tx_timer);
    core_tx_friend_disable(&m_friend);
    TEST_ASSERT_FALSE(m_friend.enabled);
    TEST_ASSERT_EQUAL(CORE_TX_FRIEND_STATE_READY, m_friend.state);

    // allocate, commit and send a packet
    test_alloc();
    mp_bearer->p_interface->packet_send(mp_bearer, m_net_packet_buf, ARRAY_SIZE(m_net_packet_buf));

    mp_broadcast = NULL;
    broadcast_send_StubWithCallback(broadcast_send_mock);

    m_friend.tx_timer.cb(0, m_friend.tx_timer.p_context);
    TEST_ASSERT_EQUAL_PTR(&m_friend.broadcast, mp_broadcast);

    core_tx_friend_disable(&m_friend);
    TEST_ASSERT_FALSE(m_friend.enabled);

    // if the tx complete processing runs now, it should trigger a TX complete, even if it has been disabled:
    core_tx_complete_Expect(mp_bearer, CORE_TX_ROLE_ORIGINATOR, TX_TIMESTAMP, TX_TOKEN);
    mp_tx_complete_event->callback(mp_tx_complete_event->p_context);

}

void test_scheduling(void)
{
    test_enable();
    timer_sch_reschedule_Expect(&m_friend.tx_timer, 100 - EXPECTED_SCHEDULE_OVERHEAD); // subtract overhead
    core_tx_friend_schedule(&m_friend, 100);

    // shouldn't police the state in this function
    m_friend.enabled = false;
    timer_sch_reschedule_Expect(&m_friend.tx_timer, 100 - EXPECTED_SCHEDULE_OVERHEAD); // subtract overhead
    core_tx_friend_schedule(&m_friend, 100);
}

/**
 * Testing stopping then starting again within the same context makes a schedule broadcast finish in
 * the second session. This shouldn't cause any trouble.
 */
void test_atomic_stop_start(void)
{
    test_enable();

    // allocate, commit and send a packet
    test_alloc();
    mp_bearer->p_interface->packet_send(mp_bearer, m_net_packet_buf, ARRAY_SIZE(m_net_packet_buf));

    mp_broadcast = NULL;
    broadcast_send_StubWithCallback(broadcast_send_mock);

    m_friend.tx_timer.cb(0, m_friend.tx_timer.p_context);
    TEST_ASSERT_EQUAL_PTR(&m_friend.broadcast, mp_broadcast);

    // Now disable and enable again. Should not be able to allocate a packet, as the broadcast is still in progress
    core_tx_friend_disable(&m_friend);
    TEST_ASSERT_FALSE(m_friend.enabled);
    core_tx_friend_enable(&m_friend);

    core_tx_alloc_params_t alloc_params = m_alloc_params;
    alloc_params.bearer_selector = CORE_TX_BEARER_TYPE_FRIEND;
    alloc_params.token = TX_TOKEN;
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_FAIL_NO_MEM, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));

    // Finish the packet.
    bearer_event_sequential_post_ExpectAndReturn(mp_tx_complete_event, NRF_SUCCESS);
    m_broadcast_complete_cb(&mp_broadcast->params, TX_TIMESTAMP);
    core_tx_complete_Expect(mp_bearer, CORE_TX_ROLE_ORIGINATOR, TX_TIMESTAMP, TX_TOKEN);
    mp_tx_complete_event->callback(mp_tx_complete_event->p_context);

    // Now a second allocation should go just fine:
    TEST_ASSERT_EQUAL(CORE_TX_ALLOC_SUCCESS, mp_bearer->p_interface->packet_alloc(mp_bearer, &alloc_params));
}
