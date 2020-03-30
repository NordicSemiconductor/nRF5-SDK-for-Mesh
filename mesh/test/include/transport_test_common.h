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
#ifndef TRANSPORT_TEST_COMMON_H__
#define TRANSPORT_TEST_COMMON_H__

/*****************************************************************************
 * Common functions and structures for ut_transport_replay.c and
 * ut_transport_friend.c unit tests.
 *****************************************************************************/

#include "unity.h"
#include "cmock.h"

#include "transport.h"
#include "bearer_event.h"
#include "nrf_mesh_events.h"
#include "enc_mock.h"
#include "core_tx.h"
#include "replay_cache.h"
#include "net_state.h"

#include "event_mock.h"
#include "enc_mock.h"
#include "timer_scheduler_mock.h"
#include "network_mock.h"
#include "rand_mock.h"
#include "net_state_mock.h"


#define LOCAL_ADDR      0x0200
#define UNICAST_ADDR    0x0100
#define GROUP_ADDR      0xFFF0
#define VIRTUAL_ADDR    0x8880
#define AID             5
#define BEARER_EVENT_FLAG    0x1234

typedef struct
{
    uint8_t segment_count;
    uint32_t total_len;
    uint32_t seqzero;
} sar_session_t;

extern bool m_rx_addr_ok;
extern bool m_decrypt_ok;
extern bool m_send_ok;
extern bool m_ack_on_behalf_of_friend;
extern uint32_t m_iv_index;
/*****************************************************************************
* Utility functions
*****************************************************************************/

void expect_sar_ctx_alloc(void);

void expect_sar_ctx_free(void);

void expect_sar_cancel(nrf_mesh_sar_session_cancel_reason_t reason);

void expect_access_rx(uint16_t length, uint16_t src, nrf_mesh_address_type_t dst_addr_type);

void expect_replay_cache_full(uint32_t src, uint8_t ivi, nrf_mesh_rx_failed_reason_t reason);

void expect_sar_ack(const sar_session_t * p_session, uint32_t block_ack_value);

void net_meta_build(uint16_t src, uint32_t seq, uint32_t iv_index, nrf_mesh_address_type_t dst_addr_type, network_packet_metadata_t * p_meta);

void packet_unseg_rx(const network_packet_metadata_t * p_meta, uint32_t length);

void packet_seg_rx(const network_packet_metadata_t * p_meta, uint32_t length, uint8_t segment_index, const sar_session_t * p_session);

void do_iv_update(uint32_t new_iv_index);

/*****************************************************************************
* Interface functions
*****************************************************************************/

void transport_test_common_setup(void);

void transport_test_common_teardown(void);

#endif /* TRANSPORT_TEST_COMMON_H__ */
