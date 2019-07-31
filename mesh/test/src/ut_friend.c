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

#include <unity.h>
#include "mesh_friend.h"
#include "friend_internal.h"
#include "nrf_error.h"
#include "transport_internal.h"
#include "mesh_config_entry_mock.h"
#include "transport_mock.h"
#include "core_tx_friend_mock.h"
#include "nrf_mesh_externs_mock.h"
#include "event_mock.h"
#include "timer_scheduler_mock.h"
#include "friend_sublist_mock.h"
#include "net_state_mock.h"
#include "network_mock.h"
#include "nrf_mesh_events_mock.h"
#include "manual_mock_queue.h"
#include "test_helper.h"

#include "mesh_opt_friend.h"
#include "log.h"

#define FRIEND_SRC 0x0010
#define LPN_SRC    0x00A0
#define LPN_ELEMENT_COUNT 3
#define DEFAULT_TTL 125
#define FRIEND_OPCODE_HANDLER_COUNT 6
#define TIME_NOW 1234
#define RECEIVE_DELAY_MS 80
#define POLL_TIMEOUT_MS 3000
#define RSSI_FACTOR MESH_FRIENDSHIP_RSSI_FACTOR_1_0
#define RECEIVE_WINDOW_FACTOR MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_2_0
#define MIN_FRIEND_QUEUE_SIZE MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_8

/******************************************************************************
 * Static variables
 ******************************************************************************/

static const transport_control_packet_handler_t * mp_trs_opcode_handlers = NULL;
static transport_control_packet_t m_expected_ctrl_pdu;
static nrf_mesh_network_secmat_t m_default_secmat;
static nrf_mesh_network_secmat_t m_friend_secmat;
static uint16_t m_friend_counter;
static uint16_t m_lpn_src = LPN_SRC;
static nrf_mesh_rx_metadata_t m_rx_metadata;
static network_tx_packet_buffer_t m_net_buf;
static uint8_t m_trs_pdu[PACKET_MESH_TRS_UNSEG_ACCESS_MAX_SIZE];
static packet_mesh_trs_packet_t m_expected_trs_pdu;

MOCK_QUEUE_DEF(event_queue, nrf_mesh_evt_t, NULL);
/******************************************************************************
 * Extern Variables
 ******************************************************************************/
extern const mesh_config_entry_params_t m_mesh_opt_friend_params;

/*******************************************************************************
 * Mock Functions
 ******************************************************************************/
uint32_t mesh_config_entry_set_mock(mesh_config_entry_id_t entry_id, const void * p_data, int calls)
{
    UNUSED_PARAMETER(calls);
    return m_mesh_opt_friend_params.callbacks.setter(entry_id, p_data);
}

uint32_t mesh_config_entry_get_mock(mesh_config_entry_id_t entry_id, void * p_entry, int calls)
{
    UNUSED_PARAMETER(calls);
    m_mesh_opt_friend_params.callbacks.getter(entry_id, p_entry);
    return NRF_SUCCESS;
}

uint32_t mesh_config_entry_delete_mock(mesh_config_entry_id_t entry_id, int calls)
{
    UNUSED_PARAMETER(calls);
    m_mesh_opt_friend_params.callbacks.deleter(entry_id);
    return NRF_SUCCESS;
}


static uint32_t transport_control_packet_consumer_add_stub(
    const transport_control_packet_handler_t * p_handlers,
    uint32_t handler_count,
    int num_calls)
{
    TEST_ASSERT_EQUAL(FRIEND_OPCODE_HANDLER_COUNT, handler_count);
    mp_trs_opcode_handlers = p_handlers;
    /* transport_control_packet_consumer_add_StubWithCallback(NULL); */
    return NRF_SUCCESS;
}

static uint32_t transport_control_tx_stub(const transport_control_packet_t * p_params,
                                          nrf_mesh_tx_token_t tx_token,
                                          int num_calls)
{
    const transport_control_packet_t * p_expected = &m_expected_ctrl_pdu;

    TEST_ASSERT_EQUAL(p_expected->data_len, p_params->data_len);
    TEST_ASSERT_EQUAL(p_expected->opcode, p_params->opcode);
    TEST_ASSERT_EQUAL_MEMORY(p_expected->p_data,
                             p_params->p_data,
                             p_params->data_len);
    TEST_ASSERT_EQUAL_PTR(p_expected->p_net_secmat, p_params->p_net_secmat);
    TEST_ASSERT_EQUAL(p_expected->dst.value, p_params->dst.value);
    TEST_ASSERT_EQUAL_HEX(p_expected->bearer_selector, p_params->bearer_selector);
    transport_control_tx_StubWithCallback(NULL);
    memset(&m_expected_ctrl_pdu, 0, sizeof(m_expected_ctrl_pdu));
    return NRF_SUCCESS;
}

static void event_handle_stub(const nrf_mesh_evt_t * p_evt,
                              int num_calls)
{
    nrf_mesh_evt_t expected;
    event_queue_Consume(&expected);
    TEST_ASSERT_EQUAL_nrf_mesh_evt_t(expected, (*p_evt));
}

static void unicast_address_get_Expect(void)
{
    static uint16_t local_address = FRIEND_SRC;
    static uint16_t local_address_count = 2;
    nrf_mesh_unicast_address_get_Expect(NULL, NULL);
    nrf_mesh_unicast_address_get_IgnoreArg_p_addr_start();
    nrf_mesh_unicast_address_get_IgnoreArg_p_addr_count();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&local_address);
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&local_address_count);
}

static void friendship_secmat_get_Expect(void)
{
    /* Must be static to not have the compiler be "clever". */
    static const nrf_mesh_network_secmat_t * p_net_secmat = &m_friend_secmat;
    nrf_mesh_friendship_secmat_get_Expect(m_lpn_src, NULL);
    nrf_mesh_friendship_secmat_get_IgnoreArg_pp_secmat();
    nrf_mesh_friendship_secmat_get_ReturnThruPtr_pp_secmat(&p_net_secmat);
}

static void transport_control_tx_Expect(transport_control_opcode_t opcode,
                                        const packet_mesh_trs_control_packet_t * p_data,
                                        uint32_t length,
                                        uint16_t dst,
                                        core_tx_bearer_type_t bearer_selector,
                                        const nrf_mesh_network_secmat_t * p_net_secmat)
{
    unicast_address_get_Expect();
    m_expected_ctrl_pdu.opcode = opcode;
    m_expected_ctrl_pdu.p_data =  p_data;
    m_expected_ctrl_pdu.data_len = length;
    m_expected_ctrl_pdu.src = FRIEND_SRC;
    m_expected_ctrl_pdu.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    m_expected_ctrl_pdu.dst.value = dst;
    m_expected_ctrl_pdu.dst.p_virtual_uuid = NULL;
    m_expected_ctrl_pdu.p_net_secmat = p_net_secmat;
    m_expected_ctrl_pdu.ttl = DEFAULT_TTL;
    m_expected_ctrl_pdu.bearer_selector = bearer_selector;
    transport_control_tx_StubWithCallback(transport_control_tx_stub);
}

timestamp_t timer_now(void)
{
    return TIME_NOW;
}

static void friend_rx(transport_control_opcode_t opcode,
                      const packet_mesh_trs_control_packet_t * p_pdu,
                      uint32_t length,
                      uint16_t src,
                      uint16_t dst,
                      const nrf_mesh_rx_metadata_t * p_metadata)
{
    transport_control_packet_t ctrl_packet = {0};

    ctrl_packet.opcode = opcode;
    ctrl_packet.p_data = p_pdu;
    ctrl_packet.data_len = length;
    ctrl_packet.src = src;
    ctrl_packet.dst.type = NRF_MESH_ADDRESS_TYPE_UNICAST;
    ctrl_packet.dst.value = dst;
    ctrl_packet.p_net_secmat = &m_default_secmat;


    for (uint32_t i = 0; i < FRIEND_OPCODE_HANDLER_COUNT; ++i)
    {
        if (mp_trs_opcode_handlers[i].opcode == opcode)
        {
            mp_trs_opcode_handlers[i].callback(&ctrl_packet, p_metadata);
            return;
        }
    }

    char buf[64];
    (void) sprintf(buf, "Did not find handler for opcode %x.", opcode);
    TEST_FAIL_MESSAGE(buf);
}

static void friend_poll_Receive(uint8_t fsn)
{
    static packet_mesh_trs_control_packet_t friend_poll;
    packet_mesh_trs_control_friend_poll_fsn_set(&friend_poll, fsn);
    friend_rx(TRANSPORT_CONTROL_OPCODE_FRIEND_POLL,
              &friend_poll,
              PACKET_MESH_TRS_CONTROL_FRIEND_POLL_SIZE,
              m_lpn_src,
              FRIEND_SRC,
              &m_rx_metadata);
}

static void friend_request_Receive(uint16_t prev_address, uint16_t dst)
{
    static packet_mesh_trs_control_packet_t friend_request;
    packet_mesh_trs_control_friend_request_rssi_factor_set(
        &friend_request, RSSI_FACTOR);
    packet_mesh_trs_control_friend_request_receive_window_factor_set(
        &friend_request, RECEIVE_WINDOW_FACTOR);
    packet_mesh_trs_control_friend_request_min_queue_size_log_set(
        &friend_request, MIN_FRIEND_QUEUE_SIZE);
    packet_mesh_trs_control_friend_request_receive_delay_set(
        &friend_request, RECEIVE_DELAY_MS);
    packet_mesh_trs_control_friend_request_poll_timeout_set(
        &friend_request, POLL_TIMEOUT_MS / 100);
    packet_mesh_trs_control_friend_request_previous_address_set(
        &friend_request, prev_address);
    packet_mesh_trs_control_friend_request_num_elements_set(
        &friend_request, LPN_ELEMENT_COUNT);
    packet_mesh_trs_control_friend_request_lpn_counter_set(
        &friend_request, 0);

    friend_rx(TRANSPORT_CONTROL_OPCODE_FRIEND_REQUEST,
              &friend_request,
              PACKET_MESH_TRS_CONTROL_FRIEND_REQUEST_SIZE,
              m_lpn_src,
              dst,
              &m_rx_metadata);
}

static void poll_timeout_schedule_Expect(uint32_t timeout_us)
{
    timer_sch_reschedule_Expect(NULL, timeout_us);
    timer_sch_reschedule_IgnoreArg_p_timer_evt();
}

static uint32_t network_packet_alloc_stub(network_tx_packet_buffer_t * p_buf,
                                          int num_calls)
{
    TEST_ASSERT_EQUAL_network_tx_packet_buffer_t(m_net_buf, (*p_buf));
    p_buf->p_payload = m_net_buf.p_payload;
    network_packet_alloc_StubWithCallback(NULL);
    return NRF_SUCCESS;
}

static void network_packet_send_stub(const network_tx_packet_buffer_t * p_buffer,
                                     int num_calls)
{
    TEST_ASSERT_EQUAL_MEMORY(&m_expected_trs_pdu, p_buffer->p_payload, p_buffer->user_data.payload_len);
    memset(&m_net_buf, 0, sizeof(m_net_buf));
    network_packet_send_StubWithCallback(NULL);
}

static void friend_relay_Expect(const packet_mesh_trs_packet_t * p_packet,
                                uint8_t length,
                                const transport_packet_metadata_t * p_metadata)
{
    friendship_secmat_get_Expect();
    m_net_buf.user_data.token = 0; /* TODO: Fix tokens */
    m_net_buf.user_data.payload_len = length;
    m_net_buf.user_data.bearer_selector = CORE_TX_BEARER_TYPE_FRIEND;
    m_net_buf.user_data.role = CORE_TX_ROLE_RELAY;
    m_net_buf.p_payload = m_trs_pdu;

    memcpy(&m_expected_trs_pdu, p_packet, length);
    network_packet_alloc_StubWithCallback(network_packet_alloc_stub);
    network_packet_send_StubWithCallback(network_packet_send_stub);

    core_tx_friend_schedule_Expect(NULL,
                                   m_rx_metadata.params.scanner.timestamp +
                                   MS_TO_US(RECEIVE_DELAY_MS));
    core_tx_friend_schedule_IgnoreArg_p_bearer();
    poll_timeout_schedule_Expect(m_rx_metadata.params.scanner.timestamp +
                                 MS_TO_US(POLL_TIMEOUT_MS));
}

static void friend_update_Expect(void)
{
    static packet_mesh_trs_control_packet_t friend_update;
    memset(&friend_update, 0, sizeof(friend_update));
    packet_mesh_trs_control_friend_update_key_refresh_flag_set(
        &friend_update, 0);
    packet_mesh_trs_control_friend_update_iv_update_flag_set(
        &friend_update, 0);
    packet_mesh_trs_control_friend_update_iv_index_set(
        &friend_update, 0x1234);
    packet_mesh_trs_control_friend_update_md_set(
        &friend_update, 0);

    net_state_iv_update_get_ExpectAndReturn(NET_STATE_IV_UPDATE_NORMAL);
    nrf_mesh_key_refresh_phase_get_ExpectAndReturn(NULL, NRF_MESH_KEY_REFRESH_PHASE_0);
    nrf_mesh_key_refresh_phase_get_IgnoreArg_p_secmat();
    net_state_beacon_iv_index_get_ExpectAndReturn(0x1234);

    friendship_secmat_get_Expect();
    transport_control_tx_Expect(TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE,
                                &friend_update,
                                PACKET_MESH_TRS_CONTROL_FRIEND_UPDATE_SIZE,
                                m_lpn_src,
                                CORE_TX_BEARER_TYPE_FRIEND,
                                &m_friend_secmat);

    core_tx_friend_schedule_Expect(NULL,
                                   m_rx_metadata.params.scanner.timestamp +
                                   MS_TO_US(RECEIVE_DELAY_MS));
    core_tx_friend_schedule_IgnoreArg_p_bearer();
    poll_timeout_schedule_Expect(m_rx_metadata.params.scanner.timestamp +
                                 MS_TO_US(POLL_TIMEOUT_MS));
}

static void friend_offer_Expect(void)
{
    static packet_mesh_trs_control_packet_t friend_offer;
    packet_mesh_trs_control_friend_offer_receive_window_set(
        &friend_offer, MESH_FRIEND_RECEIVE_WINDOW_DEFAULT_MS);
    packet_mesh_trs_control_friend_offer_queue_size_set(
        &friend_offer, MESH_FRIEND_QUEUE_SIZE);
    packet_mesh_trs_control_friend_offer_subscription_list_size_set(
        &friend_offer, MESH_FRIEND_SUBLIST_SIZE);
    packet_mesh_trs_control_friend_offer_rssi_set(
        &friend_offer, m_rx_metadata.params.scanner.rssi);
    packet_mesh_trs_control_friend_offer_friend_counter_set(
        &friend_offer, m_friend_counter);
    m_friend_counter++;
    core_tx_friend_enable_ExpectAnyArgs();
    transport_control_tx_Expect(TRANSPORT_CONTROL_OPCODE_FRIEND_OFFER,
                                &friend_offer,
                                PACKET_MESH_TRS_CONTROL_FRIEND_OFFER_SIZE,
                                m_lpn_src,
                                CORE_TX_BEARER_TYPE_FRIEND,
                                &m_default_secmat);

    timestamp_t offer_delay = m_rx_metadata.params.scanner.timestamp
        + MS_TO_US(MAX((RECEIVE_WINDOW_FACTOR * MESH_FRIEND_RECEIVE_WINDOW_DEFAULT_MS -
                       RSSI_FACTOR * m_rx_metadata.params.scanner.rssi),
                       100));
    core_tx_friend_schedule_Expect(NULL, offer_delay);
    core_tx_friend_schedule_IgnoreArg_p_bearer();


    static mesh_friendship_t friendship;
    friendship.lpn.src = m_lpn_src;
    friendship.lpn.prev_friend_src = 0;
    friendship.lpn.element_count = LPN_ELEMENT_COUNT;
    friendship.lpn.request_count = 0;
    friendship.poll_timeout_ms = POLL_TIMEOUT_MS;
    friendship.poll_count = 0;
    friendship.receive_delay_ms = RECEIVE_DELAY_MS;
    friendship.receive_window_ms = MESH_FRIEND_RECEIVE_WINDOW_DEFAULT_MS;
    friendship.avg_rssi = m_rx_metadata.params.scanner.rssi;

    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_FRIEND_REQUEST;
    evt.params.friend_request.p_friendship = &friendship;
    evt.params.friend_request.p_net = &m_default_secmat;
    evt.params.friend_request.p_metadata = &m_rx_metadata;
    event_queue_Expect(&evt);
    poll_timeout_schedule_Expect(offer_delay + SEC_TO_US(1));
}

/*******************************************************************************
 * Test Setup
 ******************************************************************************/
void setUp(void)
{
    mesh_config_entry_mock_Init();
    transport_mock_Init();
    core_tx_friend_mock_Init();
    nrf_mesh_externs_mock_Init();
    event_mock_Init();
    timer_scheduler_mock_Init();
    friend_sublist_mock_Init();
    net_state_mock_Init();
    network_mock_Init();
    event_queue_Init();
    nrf_mesh_events_mock_Init();

    __LOG_INIT(0xFFFFFFFF, LOG_LEVEL_DBG3, LOG_CALLBACK_DEFAULT);

    m_rx_metadata.params.scanner.rssi = -8;
    m_rx_metadata.params.scanner.timestamp = TIME_NOW;
    m_friend_counter = 0;
    m_lpn_src = LPN_SRC;
    memset(&m_net_buf, 0, sizeof(m_net_buf));
    memset(m_trs_pdu, 0, sizeof(m_trs_pdu));
    memset(&m_expected_trs_pdu, 0, sizeof(m_expected_trs_pdu));
    memset(&m_expected_ctrl_pdu, 0, sizeof(m_expected_ctrl_pdu));
    event_handle_StubWithCallback(event_handle_stub);
    mesh_config_entry_set_StubWithCallback(mesh_config_entry_set_mock);
    mesh_config_entry_get_StubWithCallback(mesh_config_entry_get_mock);
    mesh_config_entry_delete_StubWithCallback(mesh_config_entry_delete_mock);
    nrf_mesh_evt_handler_add_ExpectAnyArgs();

    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        friend_sublist_init_ExpectAnyArgs();
        core_tx_friend_init_ExpectAnyArgs();
    }

    transport_control_packet_consumer_add_StubWithCallback(
        transport_control_packet_consumer_add_stub);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_friend_init());
    TEST_ASSERT_NOT_NULL_MESSAGE(mp_trs_opcode_handlers,
                                 "Opcode handlers not set.");
}

static bool buffer_is_zeroed(void * buf, size_t size)
{
    unsigned char * p_char_buf = buf;
    return ((p_char_buf[0] == 0) &&
            (memcmp(p_char_buf, p_char_buf + 1, size -1) == 0));

}

void tearDown(void)
{
    TEST_ASSERT_MESSAGE(buffer_is_zeroed(&m_expected_ctrl_pdu, sizeof(m_expected_ctrl_pdu)),
                        "Expected ctrl PDU to be cleared. Perhaps the PDU was not sent?");

    TEST_ASSERT_NULL_MESSAGE(m_net_buf.p_payload, "network_packet_send() call less times than expected");

    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
    transport_mock_Verify();
    transport_mock_Destroy();
    core_tx_friend_mock_Verify();
    core_tx_friend_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    event_queue_Verify();
    event_queue_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    friend_sublist_mock_Verify();
    friend_sublist_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
}

/*******************************************************************************
 * Test functions
 ******************************************************************************/

void test_enable(void)
{
    bool enabled;

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_get(&enabled));
    TEST_ASSERT_FALSE(enabled);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_set(true));
    /* Second attempt returns NRF_SUCCESS */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_set(true));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_get(&enabled));
    TEST_ASSERT_TRUE(enabled);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_set(false));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_set(false));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_get(&enabled));
    TEST_ASSERT_FALSE(enabled);

    mesh_friend_enable();
    /* Second attempt (to check correct double invoking) */
    mesh_friend_enable();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_get(&enabled));
    TEST_ASSERT_TRUE(enabled);

    mesh_friend_disable();
    /* Second attempt (to check correct double invoking) */
    mesh_friend_disable();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_get(&enabled));
    TEST_ASSERT_FALSE(enabled);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_set(true));
    m_mesh_opt_friend_params.callbacks.deleter(MESH_OPT_FRIEND_EID);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_opt_friend_get(&enabled));
    TEST_ASSERT_FALSE(enabled);
}

void test_config_errors(void)
{
    mesh_config_entry_set_StubWithCallback(NULL);
    mesh_config_entry_get_StubWithCallback(NULL);

    mesh_config_entry_set_ExpectAnyArgsAndReturn(NRF_ERROR_NOT_FOUND);
    TEST_ASSERT_EQUAL(NRF_ERROR_NOT_FOUND, mesh_opt_friend_set(true));

    mesh_config_entry_get_ExpectAnyArgsAndReturn(NRF_ERROR_NULL);
    TEST_ASSERT_EQUAL(NRF_ERROR_NULL, mesh_opt_friend_get(NULL));
}

void test_friend_offer(void)
{
    mesh_friend_enable();
    unicast_address_get_Expect();
    nrf_mesh_friendship_secmat_params_set_ExpectAnyArgsAndReturn(NRF_SUCCESS);
    friend_offer_Expect();
    friend_request_Receive(NRF_MESH_ADDR_UNASSIGNED, NRF_MESH_ALL_FRIENDS_ADDR);
}

void test_no_offer_when_all_friends_are_taken(void)
{
    mesh_friend_enable();
    /* Should all succeed. */
    for (uint32_t i = 0; i < MESH_FRIEND_FRIENDSHIP_COUNT; ++i)
    {
        test_friend_offer();
        m_lpn_src++;
    }

    /* Next firend request should not get a reply. There are no more friends :( */
    friend_request_Receive(NRF_MESH_ADDR_UNASSIGNED, NRF_MESH_ALL_FRIENDS_ADDR);
}

void test_no_offer_with_invalid_dst(void)
{
    mesh_friend_enable();
    friend_request_Receive(NRF_MESH_ADDR_UNASSIGNED, FRIEND_SRC);
}

void test_lower_bound_of_friend_offer_delay_is_100ms(void)
{
    TEST_IGNORE_MESSAGE("Not implemented test.");
}

static void friendship_Establish(void)
{
    test_friend_offer();
    friend_update_Expect();
    unicast_address_get_Expect();
    friend_poll_Receive(0);
}

void test_friend_relay(void)
{
    mesh_friend_enable();
    const packet_mesh_trs_packet_t trs_pdu = {.pdu = {
            (1 << 6) | 0x0C,         /* AKF | AID */
            0xde, 0xad, 0xbe, 0xef,  /* Payload */
            0xde, 0xad, 0xca, 0xfe   /* Transport MIC */
        }};
    const uint8_t trs_pdu_length = 9;
    const transport_packet_metadata_t trs_metadata = {
        .segmented = false,
        .receivers = TRANSPORT_PACKET_RECEIVER_FRIEND,
        .mic_size = 4,
        .type.access = {
            .using_app_key = true,
            .app_key_id = 0x0c
        },
        .segmentation = {0},
        .net = {
            .src = 0x00AA,
            .dst = {.value = m_lpn_src + LPN_ELEMENT_COUNT - 1, .type = NRF_MESH_ADDRESS_TYPE_UNICAST}
        },
        .p_security_material = NULL,
        .token = 0xDEAD1337,
        .tx_bearer_selector = CORE_TX_BEARER_TYPE_FRIEND
    };

    friendship_Establish();
    friend_packet_in(&trs_pdu, trs_pdu_length, &trs_metadata, CORE_TX_ROLE_RELAY);
    friend_relay_Expect(&trs_pdu, trs_pdu_length, &trs_metadata);
    friend_poll_Receive(1);

    /* We should get the same PDU again when FSN is the same. */
    friend_relay_Expect(&trs_pdu, trs_pdu_length, &trs_metadata);
    friend_poll_Receive(1);

    /* Now there are no more data, we should get a Friend Update. */
    friend_update_Expect();
    friend_poll_Receive(0);
}
