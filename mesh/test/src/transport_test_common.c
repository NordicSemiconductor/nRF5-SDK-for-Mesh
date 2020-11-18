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
#include "transport_test_common.h"

#include "manual_mock_queue.h"

static nrf_mesh_network_secmat_t m_net_secmat;
static nrf_mesh_application_secmat_t m_app_secmat;
static const uint8_t m_virtual_uuid[NRF_MESH_UUID_SIZE] = {1, 2, 3, 4, 5, 6, 7, 8, 1, 2, 3, 4, 5, 6, 7, 8};
static const nrf_mesh_rx_metadata_t m_rx_metadata = {
    .source = NRF_MESH_RX_SOURCE_SCANNER,
    .params.scanner = {
        .rssi = -80,
        .channel = 38,
    },
};

static timestamp_t m_time_now;
static bearer_event_flag_callback_t m_sar_process_flag;
static core_tx_complete_cb_t m_tx_complete_cb;
static nrf_mesh_evt_handler_t * mp_replay_cache_evt_handler;
static packet_mesh_trs_packet_t m_packet_send_packet_expect;

MOCK_QUEUE_DEF(net_state_iv_index_lock_queue, bool, NULL);

/*****************************************************************************
* Externed variables
*****************************************************************************/

bool m_rx_addr_ok;
bool m_decrypt_ok;
bool m_send_ok;
bool m_ack_on_behalf_of_friend;
uint32_t m_iv_index;

/*****************************************************************************
* Static functions
*****************************************************************************/

static nrf_mesh_address_t dst_addr(nrf_mesh_address_type_t type)
{
    nrf_mesh_address_t addr;
    switch (type)
    {
        case NRF_MESH_ADDRESS_TYPE_INVALID:
            addr.value = NRF_MESH_ADDR_UNASSIGNED;
            addr.p_virtual_uuid = NULL;
            break;
        case NRF_MESH_ADDRESS_TYPE_UNICAST:
            addr.value = UNICAST_ADDR;
            addr.p_virtual_uuid = NULL;
            break;
        case NRF_MESH_ADDRESS_TYPE_GROUP:
            addr.value = GROUP_ADDR;
            addr.p_virtual_uuid = NULL;
            break;
        case NRF_MESH_ADDRESS_TYPE_VIRTUAL:
            addr.value = VIRTUAL_ADDR;
            addr.p_virtual_uuid = m_virtual_uuid;
            break;
    }
    addr.type = type;
    return addr;
}

static void enc_aes_ccm_decrypt_stub(ccm_soft_data_t * const p_ccm_data, bool * const p_mic_passed, int calls)
{
    TEST_ASSERT_EQUAL_PTR(m_app_secmat.key, p_ccm_data->p_key);
    if (p_ccm_data->p_a && p_ccm_data->a_len > 0)
    {
        TEST_ASSERT_EQUAL_PTR(m_virtual_uuid, p_ccm_data->p_a);
    }
    memcpy(p_ccm_data->p_out, p_ccm_data->p_m, p_ccm_data->m_len);
    *p_mic_passed = m_decrypt_ok;
}

static uint32_t network_packet_alloc_stub(network_tx_packet_buffer_t * p_buffer, int calls)
{
    static packet_mesh_trs_packet_t buffer;
    p_buffer->p_payload = (uint8_t *) &buffer;
    return NRF_SUCCESS;
}

static void network_packet_send_stub(const network_tx_packet_buffer_t * p_buffer, int calls)
{
    TEST_ASSERT_EQUAL_HEX8_ARRAY(&m_packet_send_packet_expect, p_buffer->p_payload, p_buffer->user_data.payload_len);

    if (p_buffer->user_data.p_metadata->control_packet &&
        TRANSPORT_CONTROL_OPCODE_SEGACK == packet_mesh_trs_control_opcode_get((const packet_mesh_trs_packet_t *) p_buffer->p_payload))
    {
        TEST_ASSERT_EQUAL_UINT32(PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_CONTROL_SEGACK_SIZE, p_buffer->user_data.payload_len);
    }

    if (p_buffer->user_data.p_metadata->control_packet &&
        TRANSPORT_CONTROL_OPCODE_SEGACK == packet_mesh_trs_control_opcode_get((const packet_mesh_trs_packet_t *) p_buffer->p_payload) &&
        packet_mesh_trs_control_segack_obo_get((const packet_mesh_trs_control_packet_t *) &p_buffer->p_payload[PACKET_MESH_TRS_UNSEG_PDU_OFFSET]))
    {
        TEST_ASSERT_EQUAL_UINT16(LOCAL_ADDR, p_buffer->user_data.p_metadata->src);
    }
    else
    {
        TEST_ASSERT_EQUAL_UINT16(UNICAST_ADDR, p_buffer->user_data.p_metadata->src);
    }
}

/*****************************************************************************
* Manual mocks
*****************************************************************************/

void nrf_mesh_app_secmat_next_get(const nrf_mesh_network_secmat_t * p_network_secmat,
                                  uint8_t aid,
                                  const nrf_mesh_application_secmat_t ** pp_app_secmat,
                                  const nrf_mesh_application_secmat_t ** pp_app_secmat_secondary)
{
    (void) pp_app_secmat_secondary;

    TEST_ASSERT_EQUAL(AID, aid);
    TEST_ASSERT_EQUAL_PTR(&m_net_secmat, p_network_secmat);

    if (*pp_app_secmat == NULL)
    {
        *pp_app_secmat = &m_app_secmat;
    }
    else
    {
        *pp_app_secmat = NULL;
    }
}

void nrf_mesh_devkey_secmat_get(uint16_t owner_addr, const nrf_mesh_application_secmat_t ** pp_devkey_secmat)
{
    TEST_FAIL_MESSAGE("Wasn't expecting transport to attempt devkeys");
}

bool nrf_mesh_rx_address_get(uint16_t short_value, nrf_mesh_address_t * p_addr)
{
    p_addr->value = short_value;
    p_addr->p_virtual_uuid = NULL;
    switch (short_value)
    {
        case UNICAST_ADDR:
            p_addr->type = NRF_MESH_ADDRESS_TYPE_UNICAST;
            return m_rx_addr_ok;
        case GROUP_ADDR:
            p_addr->type = NRF_MESH_ADDRESS_TYPE_GROUP;
            return m_rx_addr_ok;
        case VIRTUAL_ADDR:
            p_addr->type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
            p_addr->p_virtual_uuid = m_virtual_uuid;
            return m_rx_addr_ok;
        default:
            return false;
    }
}

void nrf_mesh_unicast_address_get(uint16_t * p_addr_start, uint16_t * p_addr_count)
{
    TEST_ASSERT_NOT_NULL(p_addr_start);
    TEST_ASSERT_NOT_NULL(p_addr_count);
    *p_addr_start = LOCAL_ADDR;
    *p_addr_count = 1;
}

bearer_event_flag_t bearer_event_flag_add(bearer_event_flag_callback_t callback)
{
    m_sar_process_flag = callback;
    return BEARER_EVENT_FLAG;
}

void bearer_event_flag_set(bearer_event_flag_t flag)
{
    TEST_ASSERT_EQUAL(BEARER_EVENT_FLAG, flag);
}

void core_tx_complete_cb_set(core_tx_complete_cb_t tx_complete_callback)
{
    m_tx_complete_cb = tx_complete_callback;
}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_evt_handler)
{
    TEST_ASSERT_NULL(mp_replay_cache_evt_handler); // at the time of writing we'll only add one event handler. This'll assert if that changes.
    mp_replay_cache_evt_handler = p_evt_handler;
}

uint32_t net_state_beacon_iv_index_get_cb(int cmock_num_calls)
{
    UNUSED_VARIABLE(cmock_num_calls);
    return m_iv_index;
}

void net_state_iv_index_lock_cb(bool lock, int cmock_num_calls)
{
    bool expected_lock;
    net_state_iv_index_lock_queue_Consume(&expected_lock);
    TEST_ASSERT_EQUAL_UINT8(expected_lock, lock);
}

timestamp_t timer_now(void)
{
    return m_time_now;
}

bool nrf_mesh_is_address_rx(const nrf_mesh_address_t * p_addr)
{
    (void)p_addr;
    return false;
}

/*****************************************************************************
* Utility functions
*****************************************************************************/

void expect_sar_ctx_alloc(void)
{
    bool lock = true;
    net_state_iv_index_lock_queue_Expect(&lock);
}

void expect_sar_ctx_free(void)
{
    bool lock = false;
    net_state_iv_index_lock_queue_Expect(&lock);
}

void expect_sar_cancel(nrf_mesh_sar_session_cancel_reason_t reason)
{
    static nrf_mesh_evt_t evt;
    evt = (nrf_mesh_evt_t) {
        .type = NRF_MESH_EVT_SAR_FAILED,
        .params.sar_failed = {
            .token = NRF_MESH_INITIAL_TOKEN,
            .reason = reason,
        }
    };
    event_handle_Expect(&evt);
}

void expect_access_rx(uint16_t length, uint16_t src, nrf_mesh_address_type_t dst_addr_type)
{
    static nrf_mesh_evt_t evt;
    evt = (nrf_mesh_evt_t) {
        .type = NRF_MESH_EVT_MESSAGE_RECEIVED,
        .params.message = {
            .p_buffer = NULL,
            .length = length,
            .src = {
                .value = src,
                .type = NRF_MESH_ADDRESS_TYPE_UNICAST,
                .p_virtual_uuid = NULL,
            },
            .dst = dst_addr(dst_addr_type),
            .secmat = {
                .p_app = &m_app_secmat,
                .p_net = &m_net_secmat,
            },
            .ttl = 0,
            .p_metadata = &m_rx_metadata,
        }
    };
    event_handle_Expect(&evt);
}

void expect_replay_cache_full(uint32_t src, uint8_t ivi, nrf_mesh_rx_failed_reason_t reason)
{
    static nrf_mesh_evt_t evt;
    evt = (nrf_mesh_evt_t) {
        .type = NRF_MESH_EVT_RX_FAILED,
        .params.rx_failed = {
            .src = src,
            .ivi = ivi,
            .reason = reason,
        }
    };
    event_handle_Expect(&evt);
}

void expect_sar_ack(const sar_session_t * p_session, uint32_t block_ack_value)
{
    if (m_send_ok)
    {
        packet_mesh_trs_common_seg_set(&m_packet_send_packet_expect, 0);
        packet_mesh_trs_control_opcode_set(&m_packet_send_packet_expect, TRANSPORT_CONTROL_OPCODE_SEGACK);

        packet_mesh_trs_control_packet_t * p_control_packet = (packet_mesh_trs_control_packet_t *) packet_mesh_trs_unseg_payload_get(&m_packet_send_packet_expect);
        packet_mesh_trs_control_segack_obo_set(p_control_packet, m_ack_on_behalf_of_friend);
        packet_mesh_trs_control_segack_seqzero_set(p_control_packet, p_session->seqzero);
        packet_mesh_trs_control_segack_block_ack_set(p_control_packet, block_ack_value);

        network_packet_alloc_StubWithCallback(network_packet_alloc_stub);
        network_packet_send_StubWithCallback(network_packet_send_stub);
    }
    else
    {
        network_packet_alloc_ExpectAnyArgsAndReturn(NRF_ERROR_NO_MEM);
    }
}

void net_meta_build(uint16_t src, uint32_t seq, uint32_t iv_index, nrf_mesh_address_type_t dst_addr_type, network_packet_metadata_t * p_meta)
{
    p_meta->p_security_material = &m_net_secmat;
    p_meta->dst = dst_addr(dst_addr_type);
    p_meta->ttl = 0;
    p_meta->src = src;
    p_meta->internal.sequence_number = seq;
    p_meta->internal.iv_index = iv_index;
    p_meta->control_packet = false;
}

void packet_unseg_rx(const network_packet_metadata_t * p_meta, uint32_t length)
{
    packet_mesh_trs_packet_t packet;
    packet_mesh_trs_common_seg_set(&packet, 0);
    packet_mesh_trs_access_aid_set(&packet, AID);
    packet_mesh_trs_access_akf_set(&packet, 1);

    transport_packet_in(&packet, length + PACKET_MESH_TRS_UNSEG_PDU_OFFSET + PACKET_MESH_TRS_TRANSMIC_SMALL_SIZE, p_meta, &m_rx_metadata);
}

void packet_seg_rx(const network_packet_metadata_t * p_meta, uint32_t length, uint8_t segment_index, const sar_session_t * p_session)
{
    packet_mesh_trs_packet_t packet;
    packet_mesh_trs_common_seg_set(&packet, 1);
    packet_mesh_trs_access_aid_set(&packet, AID);
    packet_mesh_trs_access_akf_set(&packet, 1);
    packet_mesh_trs_seg_sego_set(&packet, segment_index);
    packet_mesh_trs_seg_segn_set(&packet, p_session->segment_count - 1);
    packet_mesh_trs_seg_szmic_set(&packet, NRF_MESH_TRANSMIC_SIZE_SMALL);
    packet_mesh_trs_seg_seqzero_set(&packet, p_session->seqzero);

    transport_packet_in(&packet, length + PACKET_MESH_TRS_SEG_PDU_OFFSET, p_meta, &m_rx_metadata);
}

void do_iv_update(uint32_t new_iv_index)
{
    m_iv_index = new_iv_index;
    nrf_mesh_evt_t evt = {
        .type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION,
    };
    mp_replay_cache_evt_handler->evt_cb(&evt);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void transport_test_common_setup(void)
{
    m_iv_index = 0;
    mp_replay_cache_evt_handler = NULL;
    m_rx_addr_ok = true;
    m_decrypt_ok = true;
    m_send_ok = true;
    m_ack_on_behalf_of_friend = false;

    event_mock_Init();
    enc_mock_Init();
    timer_scheduler_mock_Init();
    network_mock_Init();
    rand_mock_Init();
    net_state_mock_Init();
    net_state_iv_index_lock_queue_Init();

    net_state_beacon_iv_index_get_StubWithCallback(net_state_beacon_iv_index_get_cb);
    net_state_iv_index_lock_StubWithCallback(net_state_iv_index_lock_cb);

    enc_nonce_generate_Ignore();
    enc_aes_ccm_decrypt_StubWithCallback(enc_aes_ccm_decrypt_stub);

    transport_init();
    transport_enable();
}

void transport_test_common_teardown(void)
{
    event_mock_Verify();
    event_mock_Destroy();
    enc_mock_Verify();
    enc_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    net_state_iv_index_lock_queue_Verify();
    net_state_iv_index_lock_queue_Destroy();
}
