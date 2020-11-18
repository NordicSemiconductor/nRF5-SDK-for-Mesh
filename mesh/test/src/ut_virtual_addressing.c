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

#include <cmock.h>
#include <unity.h>

#include <stdlib.h>
#include <stdbool.h>

#include "transport.h"
#include "ccm_soft.h"
#include "nrf_error.h"
#include "nrf_mesh.h"
#include "nrf_mesh_utils.h"
#include "packet.h"
#include "packet_mesh.h"
#include "log.h"
#include "utils.h"

#include "enc_mock.h"
#include "event_mock.h"
#include "timer_mock.h"
#include "timer_scheduler_mock.h"
#include "net_state_mock.h"
#include "bearer_event_mock.h"
#include "network_mock.h"
#include "rand_mock.h"
#include "replay_cache_mock.h"

#define UNICAST_ADDR            0x1201
#define VIRTUAL_ADDR            0x8080
#define VIRTUAL_ADDRESS_AMOUNT  3

static nrf_mesh_application_secmat_t m_app_dummy;
static nrf_mesh_network_secmat_t     m_nwk_dummy;
static nrf_mesh_network_secmat_t     m_net_secmat_dummy;
static uint32_t                      m_iv_index_dummy;

static const uint8_t m_virtual_uuid[VIRTUAL_ADDRESS_AMOUNT][NRF_MESH_UUID_SIZE] =
{
    {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f},
    {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f},
    {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f}
};

/************************/
/* Test setup functions */
/************************/
void setUp(void)
{
    __LOG_INIT((LOG_SRC_TRANSPORT | LOG_SRC_TEST), LOG_LEVEL_INFO, LOG_CALLBACK_DEFAULT);

    enc_mock_Init();
    event_mock_Init();
    timer_mock_Init();
    timer_scheduler_mock_Init();
    net_state_mock_Init();
    bearer_event_mock_Init();
    network_mock_Init();
    rand_mock_Init();
    replay_cache_mock_Init();

    event_handle_Ignore();
    timer_now_IgnoreAndReturn(0);
    timer_sch_schedule_Ignore();
    timer_sch_abort_Ignore();
    net_state_iv_index_lock_Ignore();
    net_state_rx_iv_index_get_IgnoreAndReturn(m_iv_index_dummy);
    net_state_tx_iv_index_get_IgnoreAndReturn(m_iv_index_dummy);
    net_state_iv_index_and_seqnum_alloc_IgnoreAndReturn(NRF_SUCCESS);
    bearer_event_critical_section_begin_Ignore();
    bearer_event_critical_section_end_Ignore();
    network_packet_alloc_IgnoreAndReturn(NULL);
    network_packet_send_Ignore();
    rand_hw_rng_get_Ignore();
    enc_nonce_generate_Ignore();
    replay_cache_init_Ignore();
    replay_cache_add_IgnoreAndReturn(NRF_SUCCESS);
    replay_cache_has_room_IgnoreAndReturn(true);
    replay_cache_has_elem_IgnoreAndReturn(false);
}

void tearDown(void)
{
    enc_mock_Verify();
    enc_mock_Destroy();
    event_mock_Verify();
    event_mock_Destroy();
    timer_mock_Verify();
    timer_mock_Destroy();
    timer_scheduler_mock_Verify();
    timer_scheduler_mock_Destroy();
    net_state_mock_Verify();
    net_state_mock_Destroy();
    bearer_event_mock_Verify();
    bearer_event_mock_Destroy();
    network_mock_Verify();
    network_mock_Destroy();
    rand_mock_Verify();
    rand_mock_Destroy();
    replay_cache_mock_Verify();
    replay_cache_mock_Destroy();
}

/**************************************************************************/
/* Local mocks. There are no the functions in the auto generated files    */
/**************************************************************************/
void nrf_mesh_app_secmat_next_get(const nrf_mesh_network_secmat_t * p_network_secmat,
                                  uint8_t aid,
                                  const nrf_mesh_application_secmat_t ** pp_app_secmat,
                                  const nrf_mesh_application_secmat_t ** pp_app_secmat_secondary)
{
    (void) pp_app_secmat_secondary;
    if (*pp_app_secmat == NULL)
    {
        *pp_app_secmat = &m_app_dummy;
        return;
    }
    *pp_app_secmat = NULL;
}

bool nrf_mesh_rx_address_get(uint16_t raw_address, nrf_mesh_address_t * p_address)
{
    static uint8_t cnt = 0;

    if (raw_address == VIRTUAL_ADDR)
    {
        p_address->type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
        p_address->value = VIRTUAL_ADDR;
        p_address->p_virtual_uuid = &m_virtual_uuid[cnt][0];

        cnt = (cnt + 1) % VIRTUAL_ADDRESS_AMOUNT;

        return true;
    }

    return false;
}

void nrf_mesh_devkey_secmat_get(uint16_t unicast_addr,
                                const nrf_mesh_application_secmat_t ** pp_app_secmat)
{
    (void)unicast_addr;

    if (*pp_app_secmat == NULL)
    {
        *pp_app_secmat = &m_app_dummy;
        return;
    }
}

/******************/
/* Test functions */
/******************/
static void mock_enc_aes_ccm_decrypt_cb(ccm_soft_data_t* const p_ccm_data, bool* const p_mic_passed, int cmock_num_calls)
{
    static uint8_t amount = 0;

    if (memcmp(&m_virtual_uuid[cmock_num_calls][0], p_ccm_data->p_a, NRF_MESH_UUID_SIZE) == 0 &&
        p_ccm_data->a_len == NRF_MESH_UUID_SIZE)
    {
        amount++;
    }

    if (amount == VIRTUAL_ADDRESS_AMOUNT)
    {
        amount = 0;
        *p_mic_passed = true;
    }
    else
    {
        *p_mic_passed = false;
    }
}

void test_general_walking_through_virtual_addresses(void)
{
    packet_mesh_trs_packet_t trs_packet;
    nrf_mesh_rx_metadata_t rx_metadata;

    enc_aes_ccm_decrypt_StubWithCallback(mock_enc_aes_ccm_decrypt_cb);

    network_packet_metadata_t net_meta;
    net_meta.control_packet = false;
    net_meta.dst.value = VIRTUAL_ADDR;
    net_meta.dst.type = NRF_MESH_ADDRESS_TYPE_VIRTUAL;
    net_meta.dst.p_virtual_uuid = NULL;
    net_meta.internal.iv_index = 0;
    net_meta.internal.sequence_number = 0;
    net_meta.p_security_material = &m_net_secmat_dummy;
    net_meta.src = UNICAST_ADDR;
    net_meta.ttl = 3;

    packet_mesh_trs_access_akf_set(&trs_packet, PACKET_MESH_TRS_ACCESS_AKF_MASK);

    TEST_ASSERT_EQUAL(
        NRF_SUCCESS,
        transport_packet_in(&trs_packet, PACKET_MESH_TRS_UNSEG_MAX_SIZE, &net_meta, &rx_metadata));
}
