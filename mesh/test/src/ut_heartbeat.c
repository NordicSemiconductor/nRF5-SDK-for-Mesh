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

#include "heartbeat.h"
#include "nrf_mesh_utils.h"
#include "test_assert.h"

#include "timer_scheduler_mock.h"
#include "event_mock.h"
#include "rand_mock.h"
#include "timer_mock.h"
#include "nrf_mesh_opt_mock.h"
#include "transport_mock.h"
#include "config_server_mock.h"
#include "nrf_mesh_externs_mock.h"
#include "mesh_config_entry_mock.h"

#include "mesh_config_entry.h"
#include "mesh_config_listener.h"
#include "mesh_opt_core.h"
#include "mesh_opt_gatt.h"

#define UT_ADDRESS_UNASSIGNED_SAMPLE    (0x0000)

#define UT_ADDRESS_UNICAST_SAMPLE       (0x7FFF)
#define UT_ADDRESS_UNICAST_SAMPLE2      (0x0FFF)
#define UT_ADDRESS_UNICAST_SAMPLE3      (0x00FF)

#define UT_ADDRESS_VIRTUAL_SAMPLE       (0xBFFF)

#define UT_ADDRESS_GROUP_MAX_SAMPLE     (0xFEFF)
#define UT_ADDRESS_GROUP_MIN_SAMPLE     (0xC000)

#define TIME_0   (10)
#define TIME_1   (10000)
#define TIME_2   (20000)

#define TIMESTAMP_DONT_CARE     (0xDEADBEEF)

#define TEST_LOCAL_ADDRESS_UNICAST  0x1000

#define UT_HB_MAX_EVENT_HANDLERS  10

/* These defines must have the same values as defined in heartbeat.c */
#define HEARTBEAT_PUBLISH_SUB_INTERVAL_S (1800)
#define HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S (1)

static heartbeat_subscription_state_t  m_ut_hb_sub =
{
    .src        = UT_ADDRESS_UNICAST_SAMPLE,
    .dst        = UT_ADDRESS_UNICAST_SAMPLE,
    .count      = 0x0000,
    .period     = 0x1000,
    .min_hops   = 0x11,
    .max_hops   = 0x50
};

static nrf_mesh_evt_t m_tx_complete_evt = {
    .type = NRF_MESH_EVT_TX_COMPLETE,
    .params.tx_complete.token = 0
};

static uint32_t m_timer_sch_reschedule_stub_cnt;
static uint32_t m_exp_timer_sch_reschedule_stub_cnt;

static uint32_t m_event_handler_add_stub_cnt;
static uint32_t m_exp_event_handler_add_stub_cnt;

static const transport_control_packet_t * mp_transport_control_tx_p_params;
static nrf_mesh_tx_token_t m_tcp_tx_token;
static uint32_t m_transport_control_tx_stub_cnt;
static uint32_t m_exp_transport_control_tx_stub_cnt;
static packet_mesh_trs_control_packet_t  m_generated_hb_pdu;
static uint8_t m_generated_hb_pdu_len;

static transport_control_packet_handler_t m_transport_hb_opcode_handler;

static nrf_mesh_evt_handler_t  m_event_handler;

static packet_mesh_trs_control_packet_t m_control_pkt_data;

static nrf_mesh_network_secmat_t m_net_secmat =
    {
        .nid = 0xBA,
        .encryption_key = {0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00},
        .privacy_key = {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55}
    };

static timer_event_t * mp_timer_event;
static timestamp_t m_expected_timeout;

static heartbeat_publication_state_t * mp_hb_pub;

extern mesh_config_listener_t m_heartbeat_relay_listener;
extern mesh_config_listener_t m_heartbeat_proxy_listener;

/*****************************************************************************/
/** Mock STUB functions */
#define RESCHEDULE_EXPECT(NEXT_TIMEOUT)                                 \
    do                                                                  \
    {                                                                   \
        timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_stub); \
        m_expected_timeout = NEXT_TIMEOUT;                              \
    } while (0)                                                         \

static void timer_sch_reschedule_stub(timer_event_t * p_event, timestamp_t next_timeout, int count)
{
    TEST_ASSERT_NOT_NULL(p_event);

    mp_timer_event = p_event;

    TEST_ASSERT_EQUAL(m_expected_timeout, next_timeout);

    /* De-register stub */
    timer_sch_reschedule_StubWithCallback(NULL);

    m_timer_sch_reschedule_stub_cnt++;
}

static uint32_t transport_control_tx_stub(const transport_control_packet_t * p_params, nrf_mesh_tx_token_t tx_token, int count)
{
    mp_transport_control_tx_p_params = p_params;
    m_tcp_tx_token = tx_token;
    m_transport_control_tx_stub_cnt++;

    memcpy(m_generated_hb_pdu.pdu, p_params->p_data, p_params->data_len);
    m_generated_hb_pdu_len = p_params->data_len;

    // test the transport packet's key parameters match the publication settings
    // ?? todo: fix: test_pub_params = heartbeat_publication_get();
    TEST_ASSERT_EQUAL(mp_hb_pub->ttl, p_params->ttl);
    TEST_ASSERT_TRUE((p_params->dst.type == NRF_MESH_ADDRESS_TYPE_UNICAST) || (p_params->dst.type == NRF_MESH_ADDRESS_TYPE_GROUP));
    TEST_ASSERT_EQUAL(mp_hb_pub->dst, p_params->dst.value);

    return NRF_SUCCESS;
}

static uint32_t transport_control_packet_consumer_add_stub(const transport_control_packet_handler_t * p_handlers, uint32_t handler_count, int count)
{
    m_transport_hb_opcode_handler = *p_handlers;
    return NRF_SUCCESS;
}

/** Just copy the given data into local nrf_mesh_evt_handler_t variable */
static void event_handler_add_stub(nrf_mesh_evt_handler_t * p_handler_params, int count)
{
    m_event_handler.evt_cb = p_handler_params->evt_cb;
    m_event_handler_add_stub_cnt++;
}

/** Trigger the callback from local structure */
static void event_handle_stub(const nrf_mesh_evt_t * p_evt)
{
    m_event_handler.evt_cb(p_evt);
}

static uint32_t ut_mock_config_server_hb_pub_params_get_ret_success(heartbeat_publication_information_t * p_pub_info)
 {
    // set valid security material and publication params
    p_pub_info->p_net_secmat    = &m_net_secmat;
    p_pub_info->local_address = TEST_LOCAL_ADDRESS_UNICAST;

    return NRF_SUCCESS;
}

static uint32_t ut_mock_config_server_hb_pub_params_get_ret_not_found(heartbeat_publication_information_t * p_pub_info)
{
    return NRF_ERROR_NOT_FOUND;
}

/*****************************************************************************/
/** Helper functions */

static void helper_ut_check_timer_sch_reschedule_called(void)
{
    m_exp_timer_sch_reschedule_stub_cnt++;
    TEST_ASSERT_EQUAL(m_exp_timer_sch_reschedule_stub_cnt, m_timer_sch_reschedule_stub_cnt);
}

static void helper_ut_check_event_handler_add_called(void)
{
    m_exp_event_handler_add_stub_cnt++;
    TEST_ASSERT_EQUAL(m_exp_event_handler_add_stub_cnt, m_event_handler_add_stub_cnt);
}



static transport_control_packet_t helper_create_hb_control_packet(
                                  uint16_t src,
                                  uint16_t dst,
                                  uint8_t  rx_pkt_ttl,
                                  uint8_t  hb_pdu_initttl_oct0,
                                  uint16_t hb_pdu_feature_oct12
                                  )
{
    transport_control_packet_t control_packet;
    control_packet.opcode       = TRANSPORT_CONTROL_OPCODE_HEARTBEAT;
    control_packet.p_net_secmat = NULL;
    control_packet.src          = src;
    control_packet.dst.type     = nrf_mesh_address_type_get(dst);
    control_packet.dst.value    = dst;
    control_packet.ttl          = rx_pkt_ttl;

    m_control_pkt_data.pdu[0]   = hb_pdu_initttl_oct0;
    m_control_pkt_data.pdu[1]   = (hb_pdu_feature_oct12 & 0xFF00) >> 8;
    m_control_pkt_data.pdu[2]   = (hb_pdu_feature_oct12 & 0xFF);

    control_packet.p_data       = &m_control_pkt_data;
    control_packet.data_len     = 3;
    control_packet.reliable     = false;

    return control_packet;
}

static void helper_do_heartbeat_init(hb_pub_info_getter_t p_getter)
{
    static bool hb_init_done = false;

    if (!hb_init_done)
    {
        transport_control_packet_consumer_add_StubWithCallback(transport_control_packet_consumer_add_stub);
        heartbeat_init();

        hb_init_done = true;
    }

    // Always set the callbacks, tester can change them if required.
    heartbeat_public_info_getter_register(p_getter);
}

static void helper_call_registered_timer_cb(timestamp_t timestamp)
{
    mp_timer_event->cb(timestamp, mp_timer_event->p_context);
}

static void helper_ut_check_transport_control_tx_stub_called(void)
{
    m_exp_transport_control_tx_stub_cnt++;
    TEST_ASSERT_EQUAL(m_exp_transport_control_tx_stub_cnt, m_transport_control_tx_stub_cnt);
}

static void helper_heartbeat_publication_set(uint16_t dst, uint32_t count, uint32_t period, uint8_t ttl,
                                      uint16_t features, uint16_t netkey_index)
{
    mp_hb_pub->dst = dst;
    mp_hb_pub->count = count;
    mp_hb_pub->period = period;
    mp_hb_pub->ttl = ttl;
    mp_hb_pub->features = features;
    mp_hb_pub->netkey_index = netkey_index;
}

static void feature_states_reset(void)
{
    const mesh_opt_core_adv_t entry = {.enabled = false, .tx_count = 1, .tx_interval_ms = 20};
    m_heartbeat_relay_listener.callback(MESH_CONFIG_CHANGE_REASON_SET,
                                        MESH_OPT_CORE_ADV_ROLE_TO_ENTRY_ID(CORE_TX_ROLE_RELAY),
                                        &entry);

    m_heartbeat_proxy_listener.callback(MESH_CONFIG_CHANGE_REASON_SET,
                                             MESH_OPT_GATT_PROXY_EID,
                                             &entry.enabled);
}

/*****************************************************************************/
/** Setup / Tear down */
void setUp(void)
{
    timer_scheduler_mock_Init();
    event_mock_Init();
    rand_mock_Init();
    timer_mock_Init();
    nrf_mesh_opt_mock_Init();
    transport_mock_Init();
    nrf_mesh_externs_mock_Init();
    mesh_config_entry_mock_Init();

    mp_hb_pub = heartbeat_publication_get();
    TEST_ASSERT_NOT_NULL(mp_hb_pub);
    memset(mp_hb_pub, 0, sizeof(heartbeat_publication_state_t));
    feature_states_reset();

    // clean local cb counters
    m_timer_sch_reschedule_stub_cnt   = 0;
    m_exp_timer_sch_reschedule_stub_cnt   = 0;

    m_event_handler_add_stub_cnt    = 0;
    m_exp_event_handler_add_stub_cnt  = 0;

    m_transport_control_tx_stub_cnt = 0;
    m_exp_transport_control_tx_stub_cnt = 0;
}

void tearDown(void)
{
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    nrf_mesh_opt_mock_Verify();
    nrf_mesh_opt_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
}


/*****************************************************************************
* Tests
*****************************************************************************/

// /** Only test setting of "m_hb_net_secmat" */

/** Test heartbeat_init() should ASSERT if called multiple times */
void test_heartbeat_init(void)
{
    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);
}

/** Test setting of the heartbeat pubication state */
void test_heartbeat_publication_state_updated(void)
{
    // TEST: Publication timer aborts if DST == UNASSIGNED
    mp_hb_pub->dst          = UT_ADDRESS_UNASSIGNED_SAMPLE;
    mp_hb_pub->count        = 10;
    mp_hb_pub->period       = 60;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_sch_abort_ExpectAnyArgs();
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    // TEST: Publication timer aborts if count == 0x0000
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = 0x0000;
    mp_hb_pub->period       = 60;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_sch_abort_ExpectAnyArgs();
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    // TEST: Publication timer aborts if period == 0x0000
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = 10;
    mp_hb_pub->period       = 0x0000;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    helper_heartbeat_publication_set(UT_ADDRESS_UNICAST_SAMPLE, 10, 0x0000, 0x7F,
                                     HEARTBEAT_TRIGGER_TYPE_RELAY, 0);

    timer_sch_abort_ExpectAnyArgs();
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    // TEST: When valid publication params are provided,
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = 10;
    mp_hb_pub->period       = 60;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(mp_hb_pub->period));
    event_handler_add_StubWithCallback(event_handler_add_stub);
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    helper_ut_check_event_handler_add_called();

    // TEST: When period is longer than HEARTBEAT_PUBLISH_SUB_INTERVAL_S seconds
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = 10;
    mp_hb_pub->period       = HEARTBEAT_PUBLISH_SUB_INTERVAL_S+1;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(HEARTBEAT_PUBLISH_SUB_INTERVAL_S));
    event_handler_add_StubWithCallback(event_handler_add_stub);
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    helper_ut_check_event_handler_add_called();
}

void test_heartbeat_subscription_set(void)
{
    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);

    //TEST: Null input
    TEST_NRF_MESH_ASSERT_EXPECT(heartbeat_subscription_set(NULL));

    // TEST: Valid inputs
    m_ut_hb_sub.src        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = 0x1000;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    timer_now_ExpectAndReturn(TIME_0);
    timer_sch_reschedule_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);

    heartbeat_subscription_state_t * sub_state;
    sub_state = (heartbeat_subscription_state_t *) heartbeat_subscription_get();
    TEST_ASSERT_EQUAL(sub_state->src, m_ut_hb_sub.src);
    TEST_ASSERT_EQUAL(sub_state->dst, m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(sub_state->count, 0x0000);
    TEST_ASSERT_EQUAL(sub_state->period, m_ut_hb_sub.period);
    TEST_ASSERT_EQUAL(sub_state->min_hops, 0x7F);
    TEST_ASSERT_EQUAL(sub_state->max_hops, 0x00);

    // TEST: Invalid input - period is greater than HEARTBEAT_MAX_PERIOD
    m_ut_hb_sub.src        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = HEARTBEAT_MAX_PERIOD + 1;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_ERROR_INVALID_PARAM);

    // TEST: Invalid input - Src address is neither group or virtual
    m_ut_hb_sub.src        = UT_ADDRESS_GROUP_MAX_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = 0x1000;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_ERROR_INVALID_PARAM);
    m_ut_hb_sub.src        = UT_ADDRESS_VIRTUAL_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = 0x1000;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_ERROR_INVALID_PARAM);

    // TEST: Invalid input - Dst address is virtual
    m_ut_hb_sub.src        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_VIRTUAL_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = 0x1000;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.src);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_ERROR_INVALID_PARAM);

    // TEST: If period is zero, or  src/dst  is unassigned, corresponding params are set to zeros.
    m_ut_hb_sub.src        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = 0x0000;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    timer_now_ExpectAndReturn(TIME_0);
    timer_sch_abort_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);
    TEST_ASSERT_EQUAL(sub_state->src, 0x0000);
    TEST_ASSERT_EQUAL(sub_state->dst, 0x0000);
    TEST_ASSERT_EQUAL(sub_state->period, 0x0000);

    m_ut_hb_sub.src        = UT_ADDRESS_UNASSIGNED_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = 0x1000;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    timer_now_ExpectAndReturn(TIME_0);
    timer_sch_abort_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);
    TEST_ASSERT_EQUAL(sub_state->src, 0x0000);
    TEST_ASSERT_EQUAL(sub_state->dst, 0x0000);
    TEST_ASSERT_EQUAL(sub_state->period, 0x0000);

    m_ut_hb_sub.src        = UT_ADDRESS_UNICAST_SAMPLE;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNASSIGNED_SAMPLE;
    m_ut_hb_sub.count      = 0x0000;
    m_ut_hb_sub.period     = 0x1000;
    m_ut_hb_sub.min_hops   = 0x11;
    m_ut_hb_sub.max_hops   = 0x50;
    timer_now_ExpectAndReturn(TIME_0);
    timer_sch_abort_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);
    TEST_ASSERT_EQUAL(sub_state->src, 0x0000);
    TEST_ASSERT_EQUAL(sub_state->dst, 0x0000);
    TEST_ASSERT_EQUAL(sub_state->period, 0x0000);
}

/** This test will setup the pending heartbeat message and call the heartbeat_core_evt_cb to test
 * the execution.
 */
void test_heartbeat_core_evt_cb_indirect_feat_change(void)
{
    uint8_t     init_hb_count = 0x00;

    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);

    // Setup publication state to known values, without periodic trigger
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = init_hb_count;
    mp_hb_pub->period       = 0x0000;            // this must be zero
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_sch_abort_ExpectAnyArgs();
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    // TEST: Valid behaviour on change in the feature state that is enabled.

    // request heartbeat message due to change in feature state
    event_handler_add_StubWithCallback(event_handler_add_stub);
    const mesh_opt_core_adv_t entry = {.enabled = true, .tx_count = 1, .tx_interval_ms = 20};
    m_heartbeat_relay_listener.callback(MESH_CONFIG_CHANGE_REASON_SET,
                                        MESH_OPT_CORE_ADV_ROLE_TO_ENTRY_ID(CORE_TX_ROLE_RELAY),
                                        &entry);

    // trigger core_evt_cb, to de-register event handler
    packet_mesh_trs_control_packet_t expected_pdu;

    transport_control_tx_StubWithCallback(transport_control_tx_stub);
    event_handler_remove_ExpectAnyArgs();
    event_handle_stub(&m_tx_complete_evt);
    TEST_ASSERT_EQUAL(1, m_transport_control_tx_stub_cnt);
    expected_pdu.pdu[0] = mp_hb_pub->ttl;
    expected_pdu.pdu[1] = 0x00;
    expected_pdu.pdu[2] = HEARTBEAT_TRIGGER_TYPE_RELAY;

    // Test the generated message, and ensure that count is not decremented
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_pdu.pdu, m_generated_hb_pdu.pdu, m_generated_hb_pdu_len);
    TEST_ASSERT_EQUAL(init_hb_count, mp_hb_pub->count);

    /* Test additional trigger */
    mp_hb_pub->features |= HEARTBEAT_TRIGGER_TYPE_PROXY;
    m_heartbeat_proxy_listener.callback(MESH_CONFIG_CHANGE_REASON_SET,
                                             MESH_OPT_GATT_PROXY_EID,
                                             &entry.enabled);
    event_handler_remove_ExpectAnyArgs();
    event_handle_stub(&m_tx_complete_evt);
    TEST_ASSERT_EQUAL(2, m_transport_control_tx_stub_cnt);
    expected_pdu.pdu[0] = mp_hb_pub->ttl;
    expected_pdu.pdu[1] = 0x00;
    expected_pdu.pdu[2] = HEARTBEAT_TRIGGER_TYPE_RELAY | HEARTBEAT_TRIGGER_TYPE_PROXY;

    // Test the generated message, and ensure that count is not decremented
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_pdu.pdu, m_generated_hb_pdu.pdu, m_generated_hb_pdu_len);
    TEST_ASSERT_EQUAL(init_hb_count, mp_hb_pub->count);

}

/** This test will setup the pending heartbeat message and call the heartbeat_core_evt_cb to test
 * the execution.
 */
void test_heartbeat_core_evt_cb_indirect_pub_change(void)
{
    uint8_t     init_hb_count = 10;

    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);

    // Setup publication state to known values, without periodic trigger
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = init_hb_count;
    mp_hb_pub->period       = 0x0010;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = 0x0000;      // this must be zero for this test
    mp_hb_pub->netkey_index = 0;

    // TEST: Valid behaviour of queing up a message for immediate sending on next TX complete.

    // request heartbeat message due to change in publication state
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(mp_hb_pub->period));
    event_handler_add_StubWithCallback(event_handler_add_stub);
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    helper_ut_check_event_handler_add_called();

    // trigger core_evt_cb, to de-register event handler
    packet_mesh_trs_control_packet_t expected_pdu;
    transport_control_tx_StubWithCallback(transport_control_tx_stub);
    event_handler_remove_ExpectAnyArgs();
    event_handle_stub(&m_tx_complete_evt);
    TEST_ASSERT_EQUAL(m_transport_control_tx_stub_cnt, 1);
    expected_pdu.pdu[0] = mp_hb_pub->ttl;
    expected_pdu.pdu[1] = 0x00;
    expected_pdu.pdu[2] = 0x00;

    // Test the generated message, and ensure that count is not decremented
    TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_pdu.pdu, m_generated_hb_pdu.pdu, m_generated_hb_pdu_len);
    TEST_ASSERT_EQUAL(init_hb_count - 1, mp_hb_pub->count);
}

void test_heartbeat_on_feature_change_trigger(void)
{
    uint8_t     init_hb_count = 10;

    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);

    // Setup publication state to known values, without periodic trigger
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = init_hb_count;
    mp_hb_pub->period       = 0x0000;            // this must be zero
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_sch_abort_ExpectAnyArgs();
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    // Test1: DO NOT Trigger on HEARTBEAT_TRIGGER_TYPE_PROXY
    // request heartbeat message due to change in feature state
    bool enabled = true;
    m_heartbeat_proxy_listener.callback(MESH_CONFIG_CHANGE_REASON_SET,
                                             MESH_OPT_GATT_PROXY_EID,
                                             &enabled);

    // emulate TX_COMPLETE event
    // No functions are expected to be called, since HB is not triggered internally
    event_handler_remove_ExpectAnyArgs();
    event_handle_stub(&m_tx_complete_evt);
    TEST_ASSERT_EQUAL(m_transport_control_tx_stub_cnt, 0);
    TEST_ASSERT_EQUAL(init_hb_count, mp_hb_pub->count);


    // TODO: Test2: DO NOT Trigger on HEARTBEAT_TRIGGER_TYPE_FRIEND
    // request heartbeat message due to change in feature state

    // TODO: Test3: DO NOT Trigger on HEARTBEAT_TRIGGER_TYPE_LPN
    // request heartbeat message due to change in feature state
}

/** Indirect TEST: Timed trigger (publication) functionality test */
void test_heartbeat_functionality_periodic_publish_short(void)
{
    uint8_t init_pub_cnt = 10;
    uint8_t init_pub_per = 60;

    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);

    const mesh_opt_core_adv_t entry = {.enabled = true, .tx_count = 1, .tx_interval_ms = 20};
    m_heartbeat_relay_listener.callback(MESH_CONFIG_CHANGE_REASON_SET,
                                        MESH_OPT_CORE_ADV_ROLE_TO_ENTRY_ID(CORE_TX_ROLE_RELAY),
                                        &entry);

    // Setup valid publication params for short publication period
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = init_pub_cnt;
    mp_hb_pub->period       = init_pub_per;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(mp_hb_pub->period));
    event_handler_add_StubWithCallback(event_handler_add_stub);
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    helper_ut_check_event_handler_add_called();


    // TEST: When valid publication params are provided,
    timestamp_t timestamp = TIME_0 + SEC_TO_US(mp_hb_pub->period);


    // Send 10 heartbeats
    for (uint16_t i = 0; i < init_pub_cnt ; i++)
    {
        transport_control_tx_StubWithCallback(transport_control_tx_stub);
        RESCHEDULE_EXPECT(timestamp + SEC_TO_US(mp_hb_pub->period));

        if (i == init_pub_cnt - 1)
        {
            timer_sch_abort_ExpectAnyArgs();
        }
        mp_timer_event->cb(timestamp, mp_timer_event->p_context);
        helper_ut_check_transport_control_tx_stub_called();
        timestamp += SEC_TO_US(mp_hb_pub->period);
    }

    // on 11th call, nothing should happen and timer_sch_abort should be called
    timer_sch_abort_ExpectAnyArgs();
    mp_timer_event->cb(timestamp, mp_timer_event->p_context);

    // TEST: If config server callback fails to return publication information, timer should abort
    // Setup valid publication params for short publication period
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = init_pub_cnt;
    mp_hb_pub->period       = init_pub_per;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(mp_hb_pub->period));
    event_handler_add_StubWithCallback(event_handler_add_stub);
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    helper_ut_check_event_handler_add_called();

    // replace the mock callback to return ERROR_NOT_FOUND
    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_not_found);

    timestamp = TIME_0 + SEC_TO_US(mp_hb_pub->period);

    // try to send heartbeat by triggering publication timer cb
    timer_sch_abort_ExpectAnyArgs();
    mp_timer_event->cb(timestamp, mp_timer_event->p_context);
}

/** Indirect TEST: Timed trigger (publication) functionality test, when period is longer
 * than HEARTBEAT_PUBLISH_SUB_INTERVAL_S seconds.
 */
void test_heartbeat_functionality_periodic_publish_long(void)
{
    uint32_t init_pub_cnt = 2;
    uint32_t init_pub_per = HEARTBEAT_PUBLISH_SUB_INTERVAL_S + 1;
    uint32_t mult = (init_pub_per/HEARTBEAT_PUBLISH_SUB_INTERVAL_S) + ((init_pub_per % HEARTBEAT_PUBLISH_SUB_INTERVAL_S) ? 1 : 0);
    timestamp_t timestamp;

    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);

    const mesh_opt_core_adv_t entry = {.enabled = true, .tx_count = 1, .tx_interval_ms = 20};
    m_heartbeat_relay_listener.callback(MESH_CONFIG_CHANGE_REASON_SET,
                                        MESH_OPT_CORE_ADV_ROLE_TO_ENTRY_ID(CORE_TX_ROLE_RELAY),
                                        &entry);

    // Setup valid publication params for short publication period
    mp_hb_pub->dst          = UT_ADDRESS_UNICAST_SAMPLE;
    mp_hb_pub->count        = init_pub_cnt;
    mp_hb_pub->period       = init_pub_per;
    mp_hb_pub->ttl          = 0x7F;
    mp_hb_pub->features     = HEARTBEAT_TRIGGER_TYPE_RELAY;
    mp_hb_pub->netkey_index = 0;
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(HEARTBEAT_PUBLISH_SUB_INTERVAL_S));
    timestamp = TIME_0 + SEC_TO_US(HEARTBEAT_PUBLISH_SUB_INTERVAL_S);
    event_handler_add_StubWithCallback(event_handler_add_stub);
    mesh_config_entry_set_ExpectAndReturn(MESH_OPT_CORE_HB_PUBLICATION_EID, mp_hb_pub, NRF_SUCCESS);
    heartbeat_publication_state_updated();

    helper_ut_check_event_handler_add_called();


    // TEST: When valid publication params are provided,

    // Send 10 heartbeats
    for (uint16_t i = 0; i < init_pub_cnt * mult ; i++)
    {
        // timer_now_ExpectAndReturn(TIME_1);
        if (((i % mult) == 1))
        {
            transport_control_tx_StubWithCallback(transport_control_tx_stub);

            // Add slight jitter (+ i) to timestamp, to ensure that logic works even if timestamps are
            // slighly further than expected
            RESCHEDULE_EXPECT(timestamp + SEC_TO_US(HEARTBEAT_PUBLISH_SUB_INTERVAL_S) + i);

            if (i == (init_pub_cnt * mult) - 1)
            {
                timer_sch_abort_ExpectAnyArgs();
            }

            mp_timer_event->cb(timestamp + i, mp_timer_event->p_context);
            timestamp += SEC_TO_US(HEARTBEAT_PUBLISH_SUB_INTERVAL_S) + i;
            helper_ut_check_transport_control_tx_stub_called();
        }
        else
        {
            // add slight delta
            RESCHEDULE_EXPECT(timestamp + SEC_TO_US(mp_hb_pub->period % HEARTBEAT_PUBLISH_SUB_INTERVAL_S));
            mp_timer_event->cb(timestamp, mp_timer_event->p_context);
            timestamp += (SEC_TO_US(mp_hb_pub->period % HEARTBEAT_PUBLISH_SUB_INTERVAL_S));
        }
    }

    // Nothing should happen for this callback and timer_sch_abort should be called again
    timer_sch_abort_ExpectAnyArgs();
    mp_timer_event->cb(timestamp, mp_timer_event->p_context);
}

/** Indirect TEST: Check functioning of the opcode handler */
void test_heartbeat_opcode_handle(void)
{
    uint8_t  hb_init_ttl;
    uint8_t  hb_rx_ttl;
    uint16_t hb_features;
    uint16_t hb_src;
    uint16_t hb_dst;

    transport_control_packet_t control_packet;

    helper_do_heartbeat_init(ut_mock_config_server_hb_pub_params_get_ret_success);

    // Reset subscription state, src and dst are UNASSIGNED, and period is zero
    m_ut_hb_sub.src = UT_ADDRESS_UNICAST_SAMPLE2;
    m_ut_hb_sub.dst = UT_ADDRESS_UNICAST_SAMPLE3;
    m_ut_hb_sub.period = 0x0100;
    timer_now_ExpectAndReturn(TIME_0);
    timer_sch_reschedule_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);

    //Test1: HB message are ignored when not subscribed to given src/dst
    // create valid hb message
    hb_init_ttl = 0x05;
    hb_rx_ttl   = 0x02;
    hb_features = HEARTBEAT_TRIGGER_TYPE_RELAY | HEARTBEAT_TRIGGER_TYPE_PROXY;
    hb_src      = UT_ADDRESS_UNICAST_SAMPLE2;
    hb_dst      = UT_ADDRESS_GROUP_MAX_SAMPLE;
    control_packet = helper_create_hb_control_packet (hb_src, hb_dst, hb_rx_ttl, hb_init_ttl, hb_features);
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);

    hb_init_ttl = 0x05;
    hb_rx_ttl   = 0x02;
    hb_features = HEARTBEAT_TRIGGER_TYPE_RELAY | HEARTBEAT_TRIGGER_TYPE_PROXY;
    hb_src      = UT_ADDRESS_GROUP_MAX_SAMPLE;
    hb_dst      = UT_ADDRESS_UNICAST_SAMPLE3;
    control_packet = helper_create_hb_control_packet (hb_src, hb_dst, hb_rx_ttl, hb_init_ttl, hb_features);
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);

    //Test2: HB messages is processed when subscribed to given source and dst, and period is not expired
    hb_init_ttl = 0x05;
    hb_rx_ttl   = 0x02;
    hb_features = HEARTBEAT_TRIGGER_TYPE_RELAY | HEARTBEAT_TRIGGER_TYPE_PROXY;
    hb_src      = UT_ADDRESS_UNICAST_SAMPLE;
    hb_dst      = UT_ADDRESS_UNICAST_SAMPLE2;
    control_packet = helper_create_hb_control_packet (hb_src, hb_dst, hb_rx_ttl, hb_init_ttl, hb_features);

    m_ut_hb_sub.src        = hb_src;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE2;
    m_ut_hb_sub.period     = 10;
    timer_now_ExpectAndReturn(TIME_0);
    timer_sch_reschedule_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);

    nrf_mesh_evt_t evt = {
        .type = NRF_MESH_EVT_HB_MESSAGE_RECEIVED,
        .params.hb_message.init_ttl = hb_init_ttl,
        .params.hb_message.hops     = hb_init_ttl - hb_rx_ttl + 1,
        .params.hb_message.features = hb_features,
        .params.hb_message.src      = hb_src
    };
    event_handle_Expect(&evt);
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);


    //Test3: HB message processing stops when the time runs out
    m_ut_hb_sub.src        = hb_src;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE2;
    m_ut_hb_sub.period     = 10;                              // 10 seconds
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S));
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);
    helper_ut_check_timer_sch_reschedule_called();

    event_handle_Expect(&evt);
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);

    // advance the time by triggering timer callback.
    for (uint32_t i = 0; i < m_ut_hb_sub.period; i++ )
    {
        event_handle_Expect(&evt);
        m_transport_hb_opcode_handler.callback(&control_packet, NULL);
        helper_call_registered_timer_cb(TIMESTAMP_DONT_CARE);
    }

    // no event is generated after time runs out
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);

    // Ensure timer aborts, on the last callback
    timer_sch_abort_ExpectAnyArgs();
    helper_call_registered_timer_cb(TIMESTAMP_DONT_CARE);

    // no event is generated after time runs out
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);


    //Test4: RFU feature bits are set to zero before using
    m_ut_hb_sub.src        = hb_src;
    m_ut_hb_sub.dst        = UT_ADDRESS_UNICAST_SAMPLE2;
    m_ut_hb_sub.period     = 10;                              // 10 seconds
    timer_now_ExpectAndReturn(TIME_0);
    RESCHEDULE_EXPECT(TIME_0 + SEC_TO_US(HEARTBEAT_SUBSCRIPTION_TIMER_GRANULARITY_S));
    nrf_mesh_unicast_address_get_ExpectAnyArgs();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&m_ut_hb_sub.dst);
    TEST_ASSERT_EQUAL(heartbeat_subscription_set(&m_ut_hb_sub), NRF_SUCCESS);
    helper_ut_check_timer_sch_reschedule_called();

    control_packet = helper_create_hb_control_packet (hb_src, hb_dst, hb_rx_ttl, hb_init_ttl, hb_features | 0xFFF0);
    event_handle_Expect(&evt);
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);

    //Test5: initTTL value higher than 0x7F is masked properly before using
    control_packet = helper_create_hb_control_packet (hb_src, hb_dst, hb_rx_ttl, hb_init_ttl | 0x80, hb_features);
    event_handle_Expect(&evt);
    m_transport_hb_opcode_handler.callback(&control_packet, NULL);
}
