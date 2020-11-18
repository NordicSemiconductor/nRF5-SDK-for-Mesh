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
#include "friend_sublist_mock.h"
#include "net_state_mock.h"
#include "network_mock.h"
#include "nrf_mesh_events_mock.h"
#include "manual_mock_queue.h"
#include "test_helper.h"
#include "test_assert.h"
#include "mesh_opt_core_mock.h"
#include "device_state_manager_mock.h"
#include "long_timer_mock.h"

#include "mesh_opt_friend.h"
#include "log.h"

#define FRIEND_SRC 0x0010
#define FRIEND_SRC2 0x0011
#define LPN_SRC     0x00A0
#define LPN_SRC2    0x00A2
#define LPN_SRC3    0x00A3
#define LPN_SRC4    0x00A4
#define LPN_SRC5    0x00A5
#define LPN_ELEMENT_COUNT 3
#define DEFAULT_TTL 125
#define FRIEND_OPCODE_HANDLER_COUNT 6
#define TIME_NOW 1234
#define RECEIVE_DELAY_MS 80
#define POLL_TIMEOUT_MS 3000
#define RSSI_FACTOR MESH_FRIENDSHIP_RSSI_FACTOR_1_0
#define RECEIVE_WINDOW_FACTOR MESH_FRIENDSHIP_RECEIVE_WINDOW_FACTOR_2_0
#define MIN_FRIEND_QUEUE_SIZE MESH_FRIENDSHIP_MIN_FRIEND_QUEUE_SIZE_8

#define FRIEND_RECENT_LPNS_LIST_COUNT (MESH_FRIEND_FRIENDSHIP_COUNT + 1)

/******************************************************************************
 * Static variables
 ******************************************************************************/

static const transport_control_packet_handler_t * mp_trs_opcode_handlers = NULL;
static transport_control_packet_t m_expected_ctrl_pdu;
static nrf_mesh_network_secmat_t m_default_secmat;
static nrf_mesh_network_secmat_t m_friend_secmat;
static uint16_t m_friend_counter;
static uint16_t m_lpn_src = LPN_SRC;
static uint32_t m_time_now;
static long_timer_t * mp_confirm_send_timer;
static nrf_mesh_rx_metadata_t m_rx_metadata;
static network_tx_packet_buffer_t m_net_buf;
static uint8_t m_trs_pdu[PACKET_MESH_TRS_UNSEG_ACCESS_MAX_SIZE];
static packet_mesh_trs_packet_t m_expected_trs_pdu;
static mesh_friendship_t m_friendship;

static nrf_mesh_evt_handler_cb_t frnd_event_hndlr;

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
    if (entry_id.file == MESH_OPT_CORE_FILE_ID &&
        entry_id.record == (MESH_OPT_CORE_TX_POWER_RECORD_START + CORE_TX_ROLE_ORIGINATOR))
    {
        *((radio_tx_power_t *)p_entry) = RADIO_POWER_NRF_0DBM;
        return NRF_SUCCESS;
    }
    else
    {
        m_mesh_opt_friend_params.callbacks.getter(entry_id, p_entry);
        return NRF_SUCCESS;
    }
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

uint64_t lt_remaining_time_get_stub(const long_timer_t * p_timer, int num_calls)
{
    TEST_ASSERT_NOT_NULL(p_timer);

    return p_timer->remaining_time_us;
}

static void unicast_address_get_Expect(void)
{
    static uint16_t local_address = FRIEND_SRC;
    static uint16_t local_address_count = 2;
    nrf_mesh_unicast_address_get_Expect(NULL, NULL);
    nrf_mesh_unicast_address_get_IgnoreArg_p_addr_start();
    nrf_mesh_unicast_address_get_IgnoreArg_p_addr_count();
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_start(&local_address);
    nrf_mesh_unicast_address_get_ReturnThruPtr_p_addr_count(&local_address_count);
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
    if (opcode != TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR_CONFIRM)
    {
        unicast_address_get_Expect();
    }
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

static void friendship_established_event_Expect(void)
{
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED;
    evt.params.friendship_established.role = NRF_MESH_FRIENDSHIP_ROLE_FRIEND;
    evt.params.friendship_established.lpn_src = m_lpn_src;
    evt.params.friendship_established.friend_src = FRIEND_SRC;
    event_queue_Expect(&evt);
}

timestamp_t timer_now(void)
{
    return m_time_now;
}

static void lt_schedule_stub_cb(long_timer_t * p_timer, lt_callback_t callback, void * p_context, uint64_t delay_us, int num)
{
    TEST_ASSERT(p_timer != NULL);
    TEST_ASSERT(callback != NULL);
    TEST_ASSERT(p_context != NULL);

    mp_confirm_send_timer = p_timer;
    mp_confirm_send_timer->callback = callback;
    mp_confirm_send_timer->p_context = p_context;
}

static void timer_trigger(long_timer_t * p_timer)
{
    p_timer->callback(p_timer->p_context);
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

static void friend_clear_Receive(uint16_t lpn_addr, uint16_t lpn_counter_new_friend)
{
    static packet_mesh_trs_control_packet_t friend_clear;
    packet_mesh_trs_control_friend_clear_lpn_address_set(&friend_clear, lpn_addr);
    packet_mesh_trs_control_friend_clear_lpn_counter_set(&friend_clear, lpn_counter_new_friend);

    friend_rx(TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR,
              &friend_clear,
              PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_SIZE,
              FRIEND_SRC2,
              FRIEND_SRC,
              &m_rx_metadata);
}

static void friend_friendships_Verify(void)
{
    const mesh_friendship_t * p_friendships[MESH_FRIEND_FRIENDSHIP_COUNT];
    uint8_t count = MESH_FRIEND_FRIENDSHIP_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_friend_friendships_get(&p_friendships[0], &count));
    TEST_ASSERT_EQUAL_UINT8(1, count);
    TEST_ASSERT_EQUAL_MEMORY(&m_friendship, p_friendships[0], sizeof(mesh_friendship_t));
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

static void poll_timeout_schedule_Expect(timestamp_t rx_timestamp, uint32_t timeout_us)
{
    timestamp_t diff = timer_now() - rx_timestamp;
    lt_schedule_Expect(NULL, NULL, NULL, diff < timeout_us ? timeout_us - diff : 0);
    lt_schedule_IgnoreArg_p_timer();
    lt_schedule_IgnoreArg_callback();
    lt_schedule_IgnoreArg_p_context();
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
    m_net_buf.user_data.role = p_metadata->net.control_packet ? CORE_TX_ROLE_ORIGINATOR : CORE_TX_ROLE_RELAY;
    m_net_buf.p_payload = m_trs_pdu;

    memcpy(&m_expected_trs_pdu, p_packet, length);
    network_packet_alloc_StubWithCallback(network_packet_alloc_stub);
    network_packet_send_StubWithCallback(network_packet_send_stub);

    core_tx_friend_schedule_Expect(NULL,
                                   m_rx_metadata.params.scanner.timestamp +
                                   MS_TO_US(RECEIVE_DELAY_MS));
    core_tx_friend_schedule_IgnoreArg_p_bearer();
    poll_timeout_schedule_Expect(m_rx_metadata.params.scanner.timestamp,
                                 MS_TO_US(POLL_TIMEOUT_MS));
}

static void friend_update_Expect(nrf_mesh_key_refresh_phase_t phase)
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
    nrf_mesh_key_refresh_phase_get_ExpectAndReturn(NULL, phase);
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
    poll_timeout_schedule_Expect(m_rx_metadata.params.scanner.timestamp,
                                 MS_TO_US(POLL_TIMEOUT_MS));
}


static void friend_clear_confirm_Expect(uint16_t lpn_addr, uint16_t lpn_counter)
{
    static packet_mesh_trs_control_packet_t friend_clear_confirm;

    memset(&friend_clear_confirm, 0, sizeof(friend_clear_confirm));
    packet_mesh_trs_control_friend_clear_confirm_lpn_address_set(&friend_clear_confirm, lpn_addr);
    packet_mesh_trs_control_friend_clear_confirm_lpn_counter_set(&friend_clear_confirm, lpn_counter);

    transport_control_tx_Expect(TRANSPORT_CONTROL_OPCODE_FRIEND_CLEAR_CONFIRM,
                                &friend_clear_confirm,
                                PACKET_MESH_TRS_CONTROL_FRIEND_CLEAR_CONFIRM_SIZE,
                                FRIEND_SRC2,
                                CORE_TX_BEARER_TYPE_ALLOW_ALL,
                                &m_default_secmat);
}

static void friend_update_enqueue_Expect(void)
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

    unicast_address_get_Expect();
    friendship_secmat_get_Expect();

    net_state_iv_update_get_ExpectAndReturn(NET_STATE_IV_UPDATE_NORMAL);
    nrf_mesh_key_refresh_phase_get_ExpectAndReturn(NULL, NRF_MESH_KEY_REFRESH_PHASE_2);
    nrf_mesh_key_refresh_phase_get_IgnoreArg_p_secmat();
    net_state_beacon_iv_index_get_ExpectAndReturn(0x1234);
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
        + MS_TO_US(MAX(((RECEIVE_WINDOW_FACTOR * 0.5 + 1) * MESH_FRIEND_RECEIVE_WINDOW_DEFAULT_MS -
                       (RSSI_FACTOR * 0.5 + 1) * m_rx_metadata.params.scanner.rssi),
                       100));
    core_tx_friend_schedule_Expect(NULL, offer_delay);
    core_tx_friend_schedule_IgnoreArg_p_bearer();


    m_friendship.lpn.src = m_lpn_src;
    m_friendship.lpn.prev_friend_src = 0;
    m_friendship.lpn.element_count = LPN_ELEMENT_COUNT;
    m_friendship.lpn.request_count = 0;
    m_friendship.poll_timeout_ms = POLL_TIMEOUT_MS;
    m_friendship.poll_count = 0;
    m_friendship.receive_delay_ms = RECEIVE_DELAY_MS;
    m_friendship.receive_window_ms = MESH_FRIEND_RECEIVE_WINDOW_DEFAULT_MS;
    m_friendship.avg_rssi = m_rx_metadata.params.scanner.rssi;

    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_FRIEND_REQUEST;
    evt.params.friend_request.p_friendship = &m_friendship;
    evt.params.friend_request.p_net = &m_default_secmat;
    evt.params.friend_request.p_metadata = &m_rx_metadata;
    event_queue_Expect(&evt);
    poll_timeout_schedule_Expect(0, offer_delay + SEC_TO_US(1));
}

static void friendship_state_reset_Expect(void)
{
    core_tx_friend_disable_ExpectAnyArgs();
    friend_sublist_clear_ExpectAnyArgs();
    lt_abort_ExpectAnyArgs();
}

static void friendship_terminated_event_Expect(void)
{
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_FRIENDSHIP_TERMINATED;
    evt.params.friendship_terminated.role = NRF_MESH_FRIENDSHIP_ROLE_FRIEND;
    evt.params.friendship_terminated.lpn_src = m_lpn_src;
    evt.params.friendship_terminated.friend_src = FRIEND_SRC;
    evt.params.friendship_terminated.reason = NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_USER;
    event_queue_Expect(&evt);
}

static void friendship_terminated_event_Expect_reason6(void)
{
    nrf_mesh_evt_t evt;
    evt.type = NRF_MESH_EVT_FRIENDSHIP_TERMINATED;
    evt.params.friendship_terminated.role = NRF_MESH_FRIENDSHIP_ROLE_FRIEND;
    evt.params.friendship_terminated.lpn_src = m_lpn_src;
    evt.params.friendship_terminated.friend_src = FRIEND_SRC;
    evt.params.friendship_terminated.reason = NRF_MESH_EVT_FRIENDSHIP_TERMINATED_REASON_NEW_FRIEND;
    event_queue_Expect(&evt);
}

static void friendship_Terminate(void)
{
    unicast_address_get_Expect();
    friendship_state_reset_Expect();
    friendship_terminated_event_Expect();
    const mesh_friendship_t * p_friendships[MESH_FRIEND_FRIENDSHIP_COUNT];
    uint8_t count = MESH_FRIEND_FRIENDSHIP_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_friend_friendships_get(&p_friendships[0], &count));
    lt_schedule_StubWithCallback(lt_schedule_stub_cb);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_friend_friendship_terminate(p_friendships[0]));
}

static void friendship_Terminate_by_friend_clear(void)
{
    unicast_address_get_Expect();
    friendship_state_reset_Expect();
    friendship_terminated_event_Expect_reason6();
    const mesh_friendship_t * p_friendships[MESH_FRIEND_FRIENDSHIP_COUNT];
    uint8_t count = MESH_FRIEND_FRIENDSHIP_COUNT;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, mesh_friend_friendships_get(&p_friendships[0], &count));
    lt_schedule_StubWithCallback(lt_schedule_stub_cb);
    friend_clear_confirm_Expect(p_friendships[0]->lpn.src, p_friendships[0]->lpn.request_count);

    friend_clear_Receive(p_friendships[0]->lpn.src, p_friendships[0]->lpn.request_count);
}

void nrf_mesh_evt_handler_add_cb(nrf_mesh_evt_handler_t * p_handler_params, int num_calls)
{
    UNUSED_PARAMETER(num_calls);
    TEST_ASSERT_NOT_NULL(p_handler_params);
    frnd_event_hndlr = p_handler_params->evt_cb;
    TEST_ASSERT_NOT_NULL(frnd_event_hndlr);
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
    friend_sublist_mock_Init();
    net_state_mock_Init();
    network_mock_Init();
    event_queue_Init();
    nrf_mesh_events_mock_Init();
    device_state_manager_mock_Init();
    long_timer_mock_Init();

    __LOG_INIT(0xFFFFFFFF, LOG_LEVEL_DBG3, LOG_CALLBACK_DEFAULT);

    m_rx_metadata.params.scanner.rssi = -8;
    m_rx_metadata.params.scanner.timestamp = TIME_NOW;
    m_friend_counter = 0;
    m_lpn_src = LPN_SRC;
    m_time_now = TIME_NOW;
    memset(&m_net_buf, 0, sizeof(m_net_buf));
    memset(m_trs_pdu, 0, sizeof(m_trs_pdu));
    memset(&m_expected_trs_pdu, 0, sizeof(m_expected_trs_pdu));
    memset(&m_expected_ctrl_pdu, 0, sizeof(m_expected_ctrl_pdu));
    event_handle_StubWithCallback(event_handle_stub);
    mesh_config_entry_set_StubWithCallback(mesh_config_entry_set_mock);
    mesh_config_entry_get_StubWithCallback(mesh_config_entry_get_mock);
    mesh_config_entry_delete_StubWithCallback(mesh_config_entry_delete_mock);
    nrf_mesh_evt_handler_add_StubWithCallback(nrf_mesh_evt_handler_add_cb);

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
    friend_sublist_mock_Verify();
    friend_sublist_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    nrf_mesh_events_mock_Verify();
    nrf_mesh_events_mock_Destroy();
    device_state_manager_mock_Verify();
    device_state_manager_mock_Destroy();
    long_timer_mock_Verify();
    long_timer_mock_Destroy();
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
    friend_update_Expect(NRF_MESH_KEY_REFRESH_PHASE_0);
    unicast_address_get_Expect();
    friendship_established_event_Expect();
    friend_poll_Receive(0);
    friend_friendships_Verify();
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
            .control_packet = false,
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
    friend_update_Expect(NRF_MESH_KEY_REFRESH_PHASE_0);
    friend_poll_Receive(0);

    friendship_Terminate();
}

void test_friend_needs_packet(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(friend_needs_packet(NULL));

    transport_packet_metadata_t trs_metadata = {
        .segmented = false,
        .receivers = TRANSPORT_PACKET_RECEIVER_FRIEND,
        .mic_size = 4,
        .type.access = {
            .using_app_key = true,
            .app_key_id = 0x0c
        },
        .segmentation = {0},
        .net = {
            .control_packet = false,
            .src = 0x00AA,
            .dst = {
                .value = m_lpn_src,
                .type = NRF_MESH_ADDRESS_TYPE_UNICAST
            },
            .ttl = 3,
        },
        .p_security_material = NULL,
        .token = 0xDEAD1337,
        .tx_bearer_selector = CORE_TX_BEARER_TYPE_FRIEND
    };

    /* Friend feature disabled. */
    TEST_ASSERT_FALSE(friend_needs_packet(&trs_metadata));

    /* Enabling friend feature. */
    mesh_friend_enable();
    friendship_Establish();
    TEST_ASSERT_TRUE(friend_needs_packet(&trs_metadata));

    /* No relaying for TTL < 2 (@tagMeshSp section 3.5.5). */
    trs_metadata.net.ttl = 1;
    TEST_ASSERT_FALSE(friend_needs_packet(&trs_metadata));
    trs_metadata.net.ttl = 3;

    /* Friend queue is less than number of segments. */
    trs_metadata.segmented = true;
    trs_metadata.segmentation.last_segment = MESH_FRIEND_QUEUE_SIZE;
    TEST_ASSERT_FALSE(friend_needs_packet(&trs_metadata));
    trs_metadata.segmented = false;

    /* Unknown unicast address. */
    trs_metadata.net.dst.value = m_lpn_src + LPN_ELEMENT_COUNT;
    TEST_ASSERT_FALSE(friend_needs_packet(&trs_metadata));
    trs_metadata.net.dst.value = m_lpn_src;

    /* LPN does not have a group address in subscription. */
    trs_metadata.net.dst.type = NRF_MESH_ADDRESS_TYPE_GROUP;
    trs_metadata.net.dst.value = 0xC001;
    friend_sublist_contains_ExpectAndReturn(NULL, trs_metadata.net.dst.value, NRF_ERROR_NOT_FOUND);
    friend_sublist_contains_IgnoreArg_p_sublist();
    TEST_ASSERT_FALSE(friend_needs_packet(&trs_metadata));

    friend_sublist_contains_ExpectAndReturn(NULL, trs_metadata.net.dst.value, NRF_SUCCESS);
    friend_sublist_contains_IgnoreArg_p_sublist();
    TEST_ASSERT_TRUE(friend_needs_packet(&trs_metadata));
}

void test_confirm_send_timer(void)
{
    mesh_friend_enable();

    /* Case 1: Friendship is established and terminated with Friend Clear.
       Check that the response to friend clear is received afterwards. */
    nrf_mesh_externs_mock_Verify();

    friendship_Establish();
    friendship_Terminate_by_friend_clear();

    nrf_mesh_externs_mock_Verify();

    // Receive a new friend clear, and expect confirm
    friend_clear_confirm_Expect(LPN_SRC, 0);
    friend_clear_Receive(LPN_SRC, 0);

    // Expire the timer, and receive a new friend clear, don't expect confirm
    timer_trigger(mp_confirm_send_timer);
    friend_clear_Receive(LPN_SRC, 1);

    /* Case 2: Friendship is established and terminated.
      Check that the response to friend clear is received afterwards. */
    friendship_Establish();
    friendship_Terminate();

    // Receive a new friend clear, and expect confirm
    friend_clear_confirm_Expect(LPN_SRC, 0);
    friend_clear_Receive(LPN_SRC, 0);

    // Receive a new out of range friend clear, and don't expect confirm
    friend_clear_Receive(LPN_SRC, 257);

    // Expire the timer, and receive a new friend clear, don't expect confirm
    timer_trigger(mp_confirm_send_timer);
    friend_clear_Receive(LPN_SRC, 1);

    /* Case 3: Friendship is established with many LPNs */
    uint32_t i;

    /* Last termination should over-write the first entry (corresponding to LPN_SRC) */
    lt_remaining_time_get_StubWithCallback(lt_remaining_time_get_stub);
    for (i = 0; i < (FRIEND_RECENT_LPNS_LIST_COUNT + 1); i++)
    {
        friendship_Establish();
        friendship_Terminate();

        m_time_now++;
        m_lpn_src++;
    }

    m_lpn_src = LPN_SRC;
    for (i = 0; i < (FRIEND_RECENT_LPNS_LIST_COUNT + 1); i++)
    {
        if (m_lpn_src == LPN_SRC)
        {
            friend_clear_Receive(LPN_SRC, 1);
        }
        else
        {
            friend_clear_confirm_Expect(LPN_SRC2, 0);
            friend_clear_Receive(LPN_SRC2, 0);
        }

        m_lpn_src++;
    }
}

void test_key_refresh_queueing(void)
{
    nrf_mesh_evt_t evt =
    {
        .type = NRF_MESH_EVT_KEY_REFRESH_NOTIFICATION,
    };

    /* Enabling friend feature. */
    mesh_friend_enable();
    friendship_Establish();

    evt.params.key_refresh.phase = NRF_MESH_KEY_REFRESH_PHASE_0;
    nrf_mesh_net_secmat_from_index_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    friendship_secmat_get_Expect();
    nrf_mesh_net_master_secmat_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    frnd_event_hndlr(&evt);
    friend_update_Expect(NRF_MESH_KEY_REFRESH_PHASE_0);
    friend_poll_Receive(1);

    evt.params.key_refresh.phase = NRF_MESH_KEY_REFRESH_PHASE_1;
    nrf_mesh_net_secmat_from_index_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    friendship_secmat_get_Expect();
    nrf_mesh_net_master_secmat_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    frnd_event_hndlr(&evt);
    friend_update_Expect(NRF_MESH_KEY_REFRESH_PHASE_1);
    friend_poll_Receive(0);

    evt.params.key_refresh.phase = NRF_MESH_KEY_REFRESH_PHASE_2;
    nrf_mesh_net_secmat_from_index_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    friendship_secmat_get_Expect();
    nrf_mesh_net_master_secmat_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    friend_update_enqueue_Expect();

    frnd_event_hndlr(&evt);

    const packet_mesh_trs_packet_t friend_update_pdu = {.pdu = {
            0x02,                    /* Friend Update opcode */
            0x01,                    /* Flag key refresh in Phase 2 */
            0x00, 0x00, 0x12, 0x34,  /* IV index */
            0x00,                    /* MD queue is empty */
        }};

    transport_packet_metadata_t trs_metadata = {
        .segmented = false,
        .receivers = TRANSPORT_PACKET_RECEIVER_FRIEND,
        .mic_size = 4,
        .type.control = {
            .opcode = TRANSPORT_CONTROL_OPCODE_FRIEND_UPDATE
        },
        .segmentation = {0},
        .net = {
            .control_packet = true,
            .src = 0x00AA,
            .dst = {
                .value = m_lpn_src,
                .type = NRF_MESH_ADDRESS_TYPE_UNICAST
            },
            .ttl = 3,
        },
        .p_security_material = NULL,
        .token = 0xDEAD1337,
        .tx_bearer_selector = CORE_TX_BEARER_TYPE_FRIEND
    };

    friend_relay_Expect(&friend_update_pdu, 7, &trs_metadata);
    friend_poll_Receive(1);

    evt.params.key_refresh.phase = NRF_MESH_KEY_REFRESH_PHASE_3;
    nrf_mesh_net_secmat_from_index_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    friendship_secmat_get_Expect();
    nrf_mesh_net_master_secmat_get_ExpectAnyArgsAndReturn(&m_friend_secmat);
    frnd_event_hndlr(&evt);
    friend_update_Expect(NRF_MESH_KEY_REFRESH_PHASE_3);
    friend_poll_Receive(0);
}
