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

#include "transport.h"
#include "packet_mesh.h"
#include "log.h"

#include "bearer_event_mock.h"
#include "network_mock.h"
#include "nrf_mesh_mock.h"
#include "enc_mock.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "event_mock.h"
#include "replay_cache_mock.h"
#include "nrf_mesh_externs_mock.h"
#include "core_tx_mock.h"
#include "net_state_mock.h"
#include "mesh_lpn_mock.h"
#include "mesh_lpn_internal_mock.h"
#include "nrf_mesh_events_mock.h"

#include "manual_mock_queue.h"

#define BEARER_FLAG   0xDEADBABE
#define MY_ADDRESS    0x0010
#define OTHER_ADDRESS 0x0020
#define SEQZERO       0x8
#define MIC_BYTE      0x55

/* Unsegmented message with AKF=1 */
#define UNSEGMENTED_ACCESS_PDU {0x40, 0x01, 0x02, 0x03, 0x04, 0x05,     \
                                0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B}

/* Segmented access PDU. SeqZero=123, SegO=SEQZERO, SegN=1. */
#define SEGMENTED_ACCESS_PDU_1 {0xC0, 0x00, (SEQZERO << 2), 0x01, 0x04, 0x05, \
                                0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B}


#define ACCESS_NET_METADATA                                             \
    {                                                                   \
        .dst = {NRF_MESH_ADDRESS_TYPE_UNICAST, MY_ADDRESS, NULL},       \
        .src = OTHER_ADDRESS,                                           \
        .ttl = 1,                                                       \
        .control_packet = false,                                        \
        .internal = {0, 0},                                             \
        .p_security_material = &m_default_network_secmat                \
    }

#define DEFAULT_TX_PARAMS                                               \
    {                                                                   \
        .dst = {NRF_MESH_ADDRESS_TYPE_UNICAST, OTHER_ADDRESS, NULL},    \
        .src = MY_ADDRESS,                                              \
        .ttl = 1,                                                       \
        .force_segmented = false,                                       \
        .transmic_size = NRF_MESH_TRANSMIC_SIZE_DEFAULT,                \
        .p_data = NULL,                                                 \
        .data_len = 0,                                                  \
        .security_material = {.p_net = &m_default_network_secmat,       \
                              .p_app = &m_default_app_secmat},          \
        .tx_token = 0                                                   \
    }

typedef struct
{
    uint8_t * p_payload;
    uint32_t length;
} pdu_queue_data_t;

/* Forward declaration */
static void pdu_queue_elem_free(pdu_queue_data_t * p_data);

MOCK_QUEUE_DEF(network_packet_queue, pdu_queue_data_t, pdu_queue_elem_free);
MOCK_QUEUE_DEF(alloc_packet_queue, pdu_queue_data_t, pdu_queue_elem_free);

static nrf_mesh_evt_t m_expected_evt;
static bool m_in_friendship;
static const nrf_mesh_network_secmat_t m_default_network_secmat;
static const nrf_mesh_application_secmat_t m_default_app_secmat = {.is_device_key = false};
static timer_event_t * mp_timeout_timer;
static nrf_mesh_evt_handler_t * mp_mesh_evt_handler;
static int m_expected_timer_sch_reschedules;
static uint32_t m_seqnum;
static bearer_event_flag_callback_t m_bearer_event_cb;

/*****************************************************************************
 * Stubs
 *****************************************************************************/

static void nrf_mesh_evt_handler_add_stub(nrf_mesh_evt_handler_t * p_evt_handler, int num_calls)
{
    nrf_mesh_evt_handler_add_StubWithCallback(NULL);
    mp_mesh_evt_handler = p_evt_handler;
}

static void event_handle_stub(const nrf_mesh_evt_t * p_evt, int num_calls)
{
    TEST_ASSERT_EQUAL(m_expected_evt.type, p_evt->type);
    if (p_evt->type == NRF_MESH_EVT_SAR_FAILED)
    {
        TEST_ASSERT_EQUAL(m_expected_evt.params.sar_failed.reason, p_evt->params.sar_failed.reason);
    }
    event_handle_StubWithCallback(NULL);
}

static uint32_t network_packet_alloc_stub(network_tx_packet_buffer_t * p_buffer, int num_calls)
{
    TEST_ASSERT_NOT_NULL(p_buffer);
    TEST_ASSERT(network_packet_queue_Pending());

    pdu_queue_data_t data;
    data.p_payload = malloc(p_buffer->user_data.payload_len);
    TEST_ASSERT_NOT_NULL_MESSAGE(data.p_payload, "malloc failed");
    data.length = p_buffer->user_data.payload_len;
    alloc_packet_queue_Expect(&data);

    p_buffer->p_payload = data.p_payload;
    p_buffer->user_data.p_metadata->internal.sequence_number = m_seqnum;
    m_seqnum++;
    return NRF_SUCCESS;
}

static void network_packet_send_stub(const network_tx_packet_buffer_t * p_buffer, int num_calls)
{
    TEST_ASSERT_NOT_NULL(p_buffer);
    TEST_ASSERT_NOT_NULL(p_buffer->p_payload);

    pdu_queue_data_t expected_data;
    pdu_queue_data_t actual_data;

    network_packet_queue_Consume(&expected_data);
    alloc_packet_queue_Consume(&actual_data);

    TEST_ASSERT_MESSAGE(actual_data.p_payload == p_buffer->p_payload,
                        "Unexpected user payload pointer. Most likely network_packet_alloc() was "
                        "called two or more times before network_packet_send(). This test expects "
                        "only one allocation at a time.");

    TEST_ASSERT_EQUAL(expected_data.length, p_buffer->user_data.payload_len);

    packet_mesh_trs_packet_t * p_packet = (packet_mesh_trs_packet_t *) p_buffer->p_payload;
    __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "seqzero %u, sego %u, segn %u\n",
          packet_mesh_trs_seg_seqzero_get(p_packet),
          packet_mesh_trs_seg_sego_get(p_packet),
          packet_mesh_trs_seg_segn_get(p_packet));
    TEST_ASSERT_EQUAL_MEMORY(expected_data.p_payload, p_buffer->p_payload, expected_data.length);

    free(actual_data.p_payload);
    free(expected_data.p_payload);
}

static void timer_sch_reschedule_stub(timer_event_t * p_timer_evt, timestamp_t new_timestamp, int num_calls)
{
    TEST_ASSERT(num_calls <= m_expected_timer_sch_reschedules);
    switch (num_calls)
    {
        case 0:
            /* Incomplete timer (in RX) or TX retry timer (in TX). */
            mp_timeout_timer = p_timer_evt;
            mp_timeout_timer->timestamp = new_timestamp;
            break;

        case 1:
            /* ACK timer */
        default:
            break;
    }
}

void enc_aes_ccm_encrypt_stub(ccm_soft_data_t * ccm_data, int num_calls)
{
    memset(ccm_data->p_mic, MIC_BYTE, ccm_data->mic_len);
    enc_aes_ccm_encrypt_StubWithCallback(NULL);
}

bearer_event_flag_t bearer_event_flag_add_stub(bearer_event_flag_callback_t callback, int num_calls)
{
    m_bearer_event_cb = callback;
    bearer_event_flag_add_StubWithCallback(NULL);
    return BEARER_FLAG;
}

bool mesh_lpn_is_in_friendship_stub(int calls)
{
    return m_in_friendship;
}

/*****************************************************************************
 * Helper functions
 *****************************************************************************/

static void pdu_queue_elem_free(pdu_queue_data_t * p_data)
{
    TEST_ASSERT_NOT_NULL(p_data);
    TEST_ASSERT_NOT_NULL(p_data->p_payload);
    free(p_data->p_payload);
}

static void network_packet_Expect(const uint8_t * p_payload, uint32_t length)
{
    pdu_queue_data_t data;
    data.p_payload = malloc(length);
    TEST_ASSERT_NOT_NULL_MESSAGE(data.p_payload, "malloc failed");

    data.length = length;
    memcpy(data.p_payload, p_payload, length);
    network_packet_queue_Expect(&data);
}

static void block_ack_Expect(uint16_t seqzero, uint32_t block_ack)
{
    packet_mesh_trs_packet_t packet;
    packet_mesh_trs_control_packet_t * p_control_packet =
        (packet_mesh_trs_control_packet_t *) &packet.pdu[PACKET_MESH_TRS_CONTROL_OPCODE_OFFSET + 1];

    memset(&packet, 0, sizeof(packet));
    packet_mesh_trs_control_opcode_set(&packet, TRANSPORT_CONTROL_OPCODE_SEGACK);
    packet_mesh_trs_control_segack_seqzero_set(p_control_packet, seqzero);
    packet_mesh_trs_control_segack_block_ack_set(p_control_packet, block_ack);
    network_packet_Expect(packet.pdu, PACKET_MESH_TRS_CONTROL_SEGACK_SIZE + PACKET_MESH_TRS_UNSEG_PDU_OFFSET);
}

static void segmented_packet_Expect(const uint8_t * p_payload, uint32_t length, uint16_t seqzero, uint8_t sego, uint8_t segn)
{
    packet_mesh_trs_packet_t packet;
    memset(&packet, 0, sizeof(packet));
    packet_mesh_trs_common_seg_set(&packet, 1);
    packet_mesh_trs_access_akf_set(&packet, 1);
    packet_mesh_trs_seg_segn_set(&packet, segn);
    packet_mesh_trs_seg_sego_set(&packet, sego);
    packet_mesh_trs_seg_seqzero_set(&packet, seqzero);
    memcpy(&packet.pdu[PACKET_MESH_TRS_SEG_PDU_OFFSET], p_payload, length);
    network_packet_Expect(packet.pdu, PACKET_MESH_TRS_SEG_PDU_OFFSET + length);
}

static void decrypt_Expect(void)
{
    enc_nonce_generate_ExpectAnyArgs();
    /* No keys => no decryption. */
    nrf_mesh_app_secmat_next_get_ExpectAnyArgs();
}

static void encrypt_Expect(void)
{
    enc_nonce_generate_ExpectAnyArgs();
    enc_aes_ccm_encrypt_StubWithCallback(enc_aes_ccm_encrypt_stub);
}

static void incomplete_timer_reschedule_Expect(void)
{
    m_expected_timer_sch_reschedules++;
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_stub);
    timer_now_ExpectAndReturn(0);
}

static void ack_timer_reschedule_Expect(void)
{
    m_expected_timer_sch_reschedules++;
    timer_now_ExpectAndReturn(0);
}

static void tx_retry_timer_reschedule_Expect(void)
{
    m_expected_timer_sch_reschedules++;
    timer_sch_reschedule_StubWithCallback(timer_sch_reschedule_stub);
    timer_now_ExpectAndReturn(0);
}

static void sar_cancel_evt_Expect(nrf_mesh_sar_session_cancel_reason_t reason)
{
    event_handle_StubWithCallback(event_handle_stub);
    m_expected_evt.type = NRF_MESH_EVT_SAR_FAILED;
    m_expected_evt.params.sar_failed.reason = reason;
}

static void sar_tx_ctx_free_Expect(void)
{
    timer_sch_is_scheduled_IgnoreAndReturn(true);
    timer_sch_abort_ExpectAnyArgs();
    net_state_iv_index_lock_Expect(false);
}

static void sar_rx_ctx_free_Expect(void)
{
    timer_sch_is_scheduled_IgnoreAndReturn(!m_in_friendship);
    timer_sch_is_scheduled_IgnoreAndReturn(!m_in_friendship);

    /* Incomplete timer. */
    timer_sch_abort_ExpectAnyArgs();
    /* ACK timer. */
    timer_sch_abort_ExpectAnyArgs();
    net_state_iv_index_lock_Expect(false);
}

static void transport_packet_in_Expect(void)
{
    static nrf_mesh_address_t dst_addr = {.type = NRF_MESH_ADDRESS_TYPE_UNICAST,
                                          .value = MY_ADDRESS,
                                          .p_virtual_uuid = NULL};

    mesh_lpn_rx_notify_ExpectAnyArgs();
    nrf_mesh_rx_address_get_ExpectAndReturn(dst_addr.value, NULL, true);
    nrf_mesh_rx_address_get_IgnoreArg_p_address();
    nrf_mesh_rx_address_get_ReturnThruPtr_p_address(&dst_addr);
    replay_cache_has_elem_IgnoreAndReturn(false);
    replay_cache_add_IgnoreAndReturn(NRF_SUCCESS);
}

static void incomplete_timeout_trigger(void)
{
    TEST_ASSERT_NOT_NULL(mp_timeout_timer);
    mp_timeout_timer->cb(mp_timeout_timer->timestamp, mp_timeout_timer->p_context);
}

static void tx_retry_timeout_trigger(void)
{
    incomplete_timeout_trigger();
}

static void mesh_event_send(nrf_mesh_evt_type_t evt_type)
{
    TEST_ASSERT_NOT_NULL(mp_mesh_evt_handler);
    nrf_mesh_evt_t evt;
    memset(&evt, 0, sizeof(evt));
    evt.type = evt_type;

    switch (evt.type)
    {
        case NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED:
            evt.params.friendship_established.role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
            break;
        case NRF_MESH_EVT_FRIENDSHIP_TERMINATED:
            evt.params.friendship_terminated.role = NRF_MESH_FRIENDSHIP_ROLE_LPN;
            break;
        default:
            break;
    }

    mp_mesh_evt_handler->evt_cb(&evt);
}

static void expect_init(void)
{
    replay_cache_init_Expect();
    bearer_event_flag_add_StubWithCallback(bearer_event_flag_add_stub);
    core_tx_complete_cb_set_ExpectAnyArgs();
    nrf_mesh_evt_handler_add_StubWithCallback(nrf_mesh_evt_handler_add_stub);
}

static void receive_first_segment(void)
{
    const packet_mesh_trs_packet_t packet = {.pdu = SEGMENTED_ACCESS_PDU_1};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    replay_cache_has_seqauth_IgnoreAndReturn(false);
    replay_cache_seqauth_add_IgnoreAndReturn(NRF_SUCCESS);

    transport_packet_in_Expect();

    if (!m_in_friendship)
    {
        incomplete_timer_reschedule_Expect();
        ack_timer_reschedule_Expect();
    }
    net_state_iv_index_lock_Expect(true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));
}

static void segack_receive(uint16_t seqzero, uint32_t block_ack, uint16_t src)
{
    network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    net_metadata.src = src;
    net_metadata.control_packet = true;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    packet_mesh_trs_packet_t packet;
    packet_mesh_trs_control_packet_t * p_control_packet =
        (packet_mesh_trs_control_packet_t *) &packet.pdu[PACKET_MESH_TRS_CONTROL_OPCODE_OFFSET + 1];

    memset(&packet, 0, sizeof(packet));
    packet_mesh_trs_control_opcode_set(&packet, TRANSPORT_CONTROL_OPCODE_SEGACK);
    packet_mesh_trs_control_segack_seqzero_set(p_control_packet, seqzero);
    packet_mesh_trs_control_segack_block_ack_set(p_control_packet, block_ack);
    transport_packet_in_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS,
                      transport_packet_in(
                          &packet, PACKET_MESH_TRS_CONTROL_SEGACK_SIZE + 1, &net_metadata, &rx_metadata));
}

/*****************************************************************************
 * Tests
 *****************************************************************************/

void setUp(void)
{
    bearer_event_mock_Init();
    network_mock_Init();
    nrf_mesh_mock_Init();
    enc_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
    event_mock_Init();
    replay_cache_mock_Init();
    nrf_mesh_externs_mock_Init();
    core_tx_mock_Init();
    net_state_mock_Init();
    mesh_lpn_mock_Init();
    mesh_lpn_internal_mock_Init();
    network_packet_queue_Init();
    alloc_packet_queue_Init();
    __LOG_INIT(~0, LOG_LEVEL_DBG3, LOG_CALLBACK_DEFAULT);

    m_seqnum = SEQZERO;
    m_expected_timer_sch_reschedules = 0;
    m_in_friendship = false;
    mesh_lpn_is_in_friendship_StubWithCallback(mesh_lpn_is_in_friendship_stub);
    memset(&m_expected_evt, 0, sizeof(m_expected_evt));
    network_packet_alloc_StubWithCallback(network_packet_alloc_stub);
    network_packet_send_StubWithCallback(network_packet_send_stub);
    nrf_mesh_is_address_rx_IgnoreAndReturn(false);

    expect_init();
    transport_init();
}

void tearDown(void)
{
    network_packet_queue_Verify();
    network_packet_queue_Destroy();
    alloc_packet_queue_Verify();
    alloc_packet_queue_Destroy();

    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    nrf_mesh_mock_Verify();
    nrf_mesh_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    replay_cache_mock_Verify();
    replay_cache_mock_Destroy();
    nrf_mesh_externs_mock_Verify();
    nrf_mesh_externs_mock_Destroy();
    core_tx_mock_Verify();
    core_tx_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    mesh_lpn_mock_Verify();
    mesh_lpn_mock_Destroy();
    mesh_lpn_internal_mock_Verify();
    mesh_lpn_internal_mock_Destroy();
}

/*****************************************************************************
 * RX path
 *****************************************************************************/

void test_rx_notify_before_replay_check(void)
{
    const packet_mesh_trs_packet_t packet = {.pdu = UNSEGMENTED_ACCESS_PDU};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    mesh_lpn_rx_notify_ExpectAnyArgs();
    nrf_mesh_rx_address_get_IgnoreAndReturn(true);
    replay_cache_has_elem_IgnoreAndReturn(true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));
}

void test_unsegmented_packet_in(void)
{
    const packet_mesh_trs_packet_t packet = {.pdu = UNSEGMENTED_ACCESS_PDU};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    transport_packet_in_Expect();
    decrypt_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));
}

void test_receive_first_segment(void)
{
    m_in_friendship = true;
    receive_first_segment();
}


void test_ignoring_old_segments(void)
{
    m_in_friendship = true;
    receive_first_segment();

    packet_mesh_trs_packet_t packet = {.pdu = SEGMENTED_ACCESS_PDU_1};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    transport_packet_in_Expect();

    packet_mesh_trs_seg_sego_set(&packet, 1);
    packet_mesh_trs_seg_seqzero_set(&packet, SEQZERO-1);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));

    /* Reset the seqzero and complete the transaction. */
    packet_mesh_trs_seg_seqzero_set(&packet, SEQZERO);
    packet_mesh_trs_seg_sego_set(&packet, 1);

    transport_packet_in_Expect();

    sar_rx_ctx_free_Expect();

    decrypt_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));
}


void test_cancel_existing_session_if_new(void)
{
    m_in_friendship = true;
    receive_first_segment();

    packet_mesh_trs_packet_t packet = {.pdu = SEGMENTED_ACCESS_PDU_1};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    transport_packet_in_Expect();

    /* SeqZero is new and the existing session is cancelled. */
    packet_mesh_trs_seg_seqzero_set(&packet, SEQZERO+1);

    sar_cancel_evt_Expect(NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION);
    sar_rx_ctx_free_Expect();

    /* A new session is created. */
    net_state_iv_index_lock_Expect(true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));

    /* Complete the new transaction. */
    packet_mesh_trs_seg_sego_set(&packet, 1);
    transport_packet_in_Expect();

    sar_rx_ctx_free_Expect();
    decrypt_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));
}

void test_receive_full_transaction(void)
{
    m_in_friendship = true;
    receive_first_segment();

    packet_mesh_trs_packet_t packet = {.pdu = SEGMENTED_ACCESS_PDU_1};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    transport_packet_in_Expect();

    sar_rx_ctx_free_Expect();

    /* Final segment. */
    packet_mesh_trs_seg_sego_set(&packet, 1);

    decrypt_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));
}

void test_normal_behavior_when_not_in_friendship(void)
{
    m_in_friendship = false;
    packet_mesh_trs_packet_t packet = {.pdu = SEGMENTED_ACCESS_PDU_1};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    receive_first_segment();

    /* Final segment. */
    packet_mesh_trs_seg_sego_set(&packet, 1);

    transport_packet_in_Expect();

    /* Incomplete timer is rescheduled (then aborted again...) */
    incomplete_timer_reschedule_Expect();

    block_ack_Expect(SEQZERO, 0x3);
    sar_rx_ctx_free_Expect();
    decrypt_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));
}

void test_rx_fail_when_poll_complete(void)
{
    m_in_friendship = true;
    receive_first_segment();

    sar_cancel_evt_Expect(NRF_MESH_SAR_CANCEL_REASON_LPN_RX_NOT_COMPLETE);
    sar_rx_ctx_free_Expect();
    mesh_event_send(NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE);
}

void test_no_poll_when_rx_times_out(void)
{
    m_in_friendship = false;
    receive_first_segment();

    sar_cancel_evt_Expect(NRF_MESH_SAR_CANCEL_REASON_TIMEOUT);
    sar_rx_ctx_free_Expect();
    incomplete_timeout_trigger();
}

void test_sar_rx_fail_on_friendship_termination(void)
{
    m_in_friendship = true;
    receive_first_segment();

    sar_cancel_evt_Expect(NRF_MESH_SAR_CANCEL_REASON_FRIENDSHIP_TERMINATED);
    sar_rx_ctx_free_Expect();
    m_in_friendship = false;
    mesh_event_send(NRF_MESH_EVT_FRIENDSHIP_TERMINATED);
}

void test_sar_rx_fail_on_friendship_establishment(void)
{
    m_in_friendship = false;
    receive_first_segment();

    sar_cancel_evt_Expect(NRF_MESH_SAR_CANCEL_REASON_FRIENDSHIP_ESTABLISHED);
    sar_rx_ctx_free_Expect();
    m_in_friendship = true;
    mesh_event_send(NRF_MESH_EVT_FRIENDSHIP_ESTABLISHED);
}

void test_sar_always_room_for_an_rx_in_friendship(void)
{
    m_in_friendship = true;

    /* Receive one segment. Should be placed in reserved RX context/buffer. */
    receive_first_segment();

    /* Fill up the remaining (TX) contexts.  */
    nrf_mesh_tx_params_t tx_params = DEFAULT_TX_PARAMS;
    const uint8_t access_pdu[12] = {0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xca, 0xfe, 0xde, 0xad, 0xba, 0xbe};
    const uint8_t mic[4] = {MIC_BYTE, MIC_BYTE, MIC_BYTE, MIC_BYTE};
    tx_params.p_data = access_pdu;
    tx_params.data_len = sizeof(access_pdu);

    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX-1; ++i)
    {
        net_state_iv_index_lock_Expect(true);
        encrypt_Expect();

        segmented_packet_Expect(access_pdu, sizeof(access_pdu), SEQZERO + 2*i, 0, 1);
        segmented_packet_Expect(mic, sizeof(mic), SEQZERO + 2*i, 1, 1);

        tx_retry_timer_reschedule_Expect();
        TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

        // Pick different destinations for every packet to avoid duplicate sessions
        tx_params.dst.value++;
    }

    /* All the contexts should be allocated now. */
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, transport_tx(&tx_params, NULL));

    /* Send new segment. See that the previous RX session is canceled and the new one is started. */
    packet_mesh_trs_packet_t packet = {.pdu = SEGMENTED_ACCESS_PDU_1};
    const network_packet_metadata_t net_metadata = ACCESS_NET_METADATA;
    const nrf_mesh_rx_metadata_t rx_metadata = {0};

    transport_packet_in_Expect();

    /* SeqZero is new and the existing session is cancelled. */
    packet_mesh_trs_seg_seqzero_set(&packet, SEQZERO+1);

    sar_cancel_evt_Expect(NRF_MESH_SAR_CANCEL_PEER_STARTED_ANOTHER_SESSION);
    sar_rx_ctx_free_Expect();

    /* A new session is created. */
    net_state_iv_index_lock_Expect(true);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));

    /* Complete the new transaction. */
    packet_mesh_trs_seg_sego_set(&packet, 1);
    transport_packet_in_Expect();

    sar_rx_ctx_free_Expect();
    decrypt_Expect();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_packet_in(&packet, sizeof(packet.pdu), &net_metadata, &rx_metadata));

    /* Cleanup TX contexts by acking all of them */
    for (uint32_t i = 0; i < TRANSPORT_SAR_SESSIONS_MAX-1; ++i)
    {
        /* ACK segment -> successful TX. */
        event_handle_StubWithCallback(event_handle_stub);
        m_expected_evt.type = NRF_MESH_EVT_TX_COMPLETE;
        timer_now_ExpectAndReturn(0);
        sar_tx_ctx_free_Expect();
        segack_receive(SEQZERO + 2*i, 0x03, OTHER_ADDRESS + i);
    }
}

/*****************************************************************************
 * TX path
 *****************************************************************************/

void test_poll_on_tx_retry_timeout(void)
{
    m_in_friendship = true;
    nrf_mesh_tx_params_t tx_params = DEFAULT_TX_PARAMS;
    const uint8_t access_pdu[12] = {0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xca, 0xfe, 0xde, 0xad, 0xba, 0xbe};
    const uint8_t mic[4] = {MIC_BYTE, MIC_BYTE, MIC_BYTE, MIC_BYTE};
    tx_params.p_data = access_pdu;
    tx_params.data_len = sizeof(access_pdu);

    net_state_iv_index_lock_Expect(true);
    encrypt_Expect();

    segmented_packet_Expect(access_pdu, sizeof(access_pdu), SEQZERO, 0, 1);
    segmented_packet_Expect(mic, sizeof(mic), SEQZERO, 1, 1);

    tx_retry_timer_reschedule_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    for (uint32_t i = 0; i < TRANSPORT_SAR_TX_RETRIES_DEFAULT; ++i)
    {
        __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "TX retry: %u/%u\n", i, TRANSPORT_SAR_TX_RETRIES_DEFAULT);

        /* Retry timer should trigger a poll(). */
        mesh_lpn_friend_poll_ExpectAndReturn(0, NRF_SUCCESS);
        tx_retry_timeout_trigger();

        /* On the POLL_COMPLETE, the state is reset and bearer event flag triggered. */
        bearer_event_flag_set_Expect(BEARER_FLAG);
        mesh_event_send(NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE);

        /* Calling the bearer event callback, triggers sending of segments and resetting of retry
         * timer. */
        segmented_packet_Expect(access_pdu, sizeof(access_pdu), SEQZERO, 0, 1);
        segmented_packet_Expect(mic, sizeof(mic), SEQZERO, 1, 1);
        tx_retry_timer_reschedule_Expect();
        m_bearer_event_cb();
    }

    /* Final segment. */
    __LOG(LOG_SRC_TEST, LOG_LEVEL_INFO, "TX retry: %u/%u\n", 4, TRANSPORT_SAR_TX_RETRIES_DEFAULT);

    /* Retry timer should trigger a poll(). */
    mesh_lpn_friend_poll_ExpectAndReturn(0, NRF_SUCCESS);
    tx_retry_timeout_trigger();

    /* On the final POLL_COMPLETE, the SAR TX is considered failed. */
    sar_cancel_evt_Expect(NRF_MESH_SAR_CANCEL_REASON_RETRY_OVER);
    sar_tx_ctx_free_Expect();
    mesh_event_send(NRF_MESH_EVT_LPN_FRIEND_POLL_COMPLETE);
}


void test_successful_tx_while_waiting_for_poll_complete(void)
{
    m_in_friendship = true;
    nrf_mesh_tx_params_t tx_params = DEFAULT_TX_PARAMS;
    const uint8_t access_pdu[12] = {0xde, 0xad, 0xbe, 0xef, 0xde, 0xad, 0xca, 0xfe, 0xde, 0xad, 0xba, 0xbe};
    const uint8_t mic[4] = {MIC_BYTE, MIC_BYTE, MIC_BYTE, MIC_BYTE};
    tx_params.p_data = access_pdu;
    tx_params.data_len = sizeof(access_pdu);

    net_state_iv_index_lock_Expect(true);
    encrypt_Expect();

    segmented_packet_Expect(access_pdu, sizeof(access_pdu), SEQZERO, 0, 1);
    segmented_packet_Expect(mic, sizeof(mic), SEQZERO, 1, 1);

    tx_retry_timer_reschedule_Expect();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, transport_tx(&tx_params, NULL));

    /* Retry timer should trigger a poll(). */
    mesh_lpn_friend_poll_ExpectAndReturn(0, NRF_SUCCESS);
    tx_retry_timeout_trigger();

    /* ACK the first segment. Expect the second segment to be sent again. */
    segmented_packet_Expect(mic, sizeof(mic), SEQZERO, 1, 1);
    tx_retry_timer_reschedule_Expect();
    segack_receive(SEQZERO, 0x01, OTHER_ADDRESS);

    /* Retry timer should trigger a poll(). */
    mesh_lpn_friend_poll_ExpectAndReturn(0, NRF_SUCCESS);
    tx_retry_timeout_trigger();

    /* ACK second segment -> successful TX. */
    event_handle_StubWithCallback(event_handle_stub);
    m_expected_evt.type = NRF_MESH_EVT_TX_COMPLETE;
    timer_now_ExpectAndReturn(0);
    sar_tx_ctx_free_Expect();
    segack_receive(SEQZERO, 0x03, OTHER_ADDRESS);
}
