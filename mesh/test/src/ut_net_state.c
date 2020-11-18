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
#include <cmock.h>

#include <stddef.h>

#include "net_state.h"
#include "list.h"
#include "test_assert.h"
#include "timer_scheduler.h"
#include "mesh_opt_net_state.h"
#include "nrf_mesh_events.h"

#include "mesh_config_entry_mock.h"
#include "mesh_config_mock.h"
#include "event_mock.h"

/*****************************************************************************
* Defines
*****************************************************************************/
#define SEQ_NUM_MAX 0xFFFFFF
#define EXPECTED_NET_STATE_TIMER_RES 60000000 /* From net_state.c */
#define BUFFER_SEAL 0xCABACABA
#define MEMORY_LISTENERS_MAX 2
#define HANDLE_SEQNUM  0x0001
#define HANDLE_IV_DATA 0x0002

/* Calculated integer value of IV_UPDATE_TIMEOUT_PERIODIC_SAVE_MINUTES from net_state.c */
#define CALCULATED_IV_UPDATE_TIMEOUT_PERIODIC_SAVE_MINUTES 30

/*****************************************************************************
* UT globals
*****************************************************************************/
static timer_event_t *mp_iv_update_timer;
static uint32_t m_iv_update_timeout_counter;
static int m_critical_section_counter;

static mesh_opt_iv_index_persist_data_t m_expected_iv_idx_input;
static mesh_opt_seqnum_persist_data_t m_expected_seqnum_input;
static uint8_t m_synchro_index_tst;

static list_node_t * m_evt_handler_list_head;

/*****************************************************************************
* Extern stubs
*****************************************************************************/
extern void net_state_disable(void);
extern const mesh_config_entry_params_t m_seqnum_block_params;
extern const mesh_config_entry_params_t m_iv_index_params;
extern const mesh_config_entry_params_t m_seqnum_block_legacy_params;
extern const mesh_config_entry_params_t m_iv_index_legacy_params;

/*****************************************************************************
* Mocks
*****************************************************************************/
void timer_sch_schedule(timer_event_t * p_timer)
{
    mp_iv_update_timer = p_timer;
    TEST_ASSERT_EQUAL(EXPECTED_NET_STATE_TIMER_RES, p_timer->interval);
}

void bearer_event_critical_section_begin(void)
{
    m_critical_section_counter++;

}

void bearer_event_critical_section_end(void)
{
    m_critical_section_counter--;
}

timestamp_t timer_now(void)
{
    return 0;
}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_handler_params)
{
    list_add(&m_evt_handler_list_head, &p_handler_params->node);
}

/*****************************************************************************
* Setup functions
*****************************************************************************/

void setUp(void)
{
    m_critical_section_counter = 0;
    event_mock_Init();
    mesh_config_entry_mock_Init();
    mesh_config_mock_Init();

    net_state_init();
}

void tearDown(void)
{
    m_evt_handler_list_head = NULL;
    m_synchro_index_tst = 0;
    m_iv_update_timeout_counter = 0;
    net_state_disable();

    event_mock_Verify();
    event_mock_Destroy();
    TEST_ASSERT_EQUAL(0, m_critical_section_counter);
    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
}

static void evt_notify(nrf_mesh_evt_type_t evt_type)
{
    nrf_mesh_evt_t evt =
    {
       .type = evt_type
    };

    LIST_FOREACH(p_node, m_evt_handler_list_head)
    {
        nrf_mesh_evt_handler_t * p_evt_handler = PARENT_BY_FIELD_GET(nrf_mesh_evt_handler_t, node, p_node);
        p_evt_handler->evt_cb(&evt);
    }
}

static void skip_minutes(uint32_t minutes)
{
    for (uint32_t i = 0; i < minutes; ++i)
    {
        mp_iv_update_timer->cb(0, NULL);
    }
}

static void skip_minutes_and_expect_config_iv_data(uint32_t minutes, uint32_t iv_index, net_state_iv_update_t iv_update_state)
{
    for (uint32_t i = 0; i < minutes; ++i)
    {
        m_iv_update_timeout_counter++;

        if (m_iv_update_timeout_counter % CALCULATED_IV_UPDATE_TIMEOUT_PERIODIC_SAVE_MINUTES == 0)
        {
            m_expected_iv_idx_input.iv_index = iv_index;
            m_expected_iv_idx_input.iv_update_in_progress = iv_update_state;
            m_expected_iv_idx_input.synchro_index = m_synchro_index_tst;
            m_expected_iv_idx_input.iv_update_timeout_counter = m_iv_update_timeout_counter;

            mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input, sizeof(m_expected_iv_idx_input), NRF_SUCCESS);

            m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input);
        }

        mp_iv_update_timer->cb(0, NULL);

        mesh_config_entry_mock_Verify();
    }
}

static void expect_config_seqnum(uint32_t seqnum)
{
    if (seqnum > NETWORK_SEQNUM_MAX + 1)
    {
        return;
    }

    m_expected_seqnum_input.next_block = seqnum;
    m_expected_seqnum_input.synchro_index = m_synchro_index_tst;

    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);

    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
}

static void expect_config_iv_data(uint32_t iv_index, net_state_iv_update_t iv_update_state)
{
    /* When IV Index or state is changed, IV Update timeout shall be zero. */
    m_iv_update_timeout_counter = 0;

    m_expected_iv_idx_input.iv_index = iv_index;
    m_expected_iv_idx_input.iv_update_in_progress = iv_update_state;
    m_expected_iv_idx_input.synchro_index = ++m_synchro_index_tst;
    m_expected_iv_idx_input.iv_update_timeout_counter = m_iv_update_timeout_counter;

    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input, sizeof(m_expected_iv_idx_input), NRF_SUCCESS);

    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input);
}

static void verify_ivi_state_normal(uint32_t iv_index)
{
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(iv_index, net_state_rx_iv_index_get(iv_index & 0x1));
    TEST_ASSERT_EQUAL(iv_index - 1, net_state_rx_iv_index_get((iv_index - 1) & 0x1));
}

static void verify_ivi_state_in_progress(uint32_t iv_index)
{
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(iv_index - 1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(iv_index,     net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(iv_index,     net_state_rx_iv_index_get(iv_index & 0x1));
    TEST_ASSERT_EQUAL(iv_index - 1, net_state_rx_iv_index_get((iv_index - 1) & 0x1));
}

static void iv_update_state_normal_trigger(uint32_t iv_index)
{
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(iv_index, NET_STATE_IV_UPDATE_NORMAL);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    skip_minutes(1);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    verify_ivi_state_normal(iv_index);
}

static void iv_update_state_in_progress_trigger(uint32_t iv_index)
{
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    skip_minutes(1);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    verify_ivi_state_in_progress(iv_index);
}

static void iv_index_set_trigger(uint32_t iv_index, net_state_iv_update_t iv_update_state)
{
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(iv_index, iv_update_state);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_set(iv_index, iv_update_state == NET_STATE_IV_UPDATE_IN_PROGRESS));
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
}

static void iv_update_start_trigger(uint32_t iv_index)
{
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_update_start());
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());

    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(iv_index - 1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(iv_index, net_state_beacon_iv_index_get());

    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(iv_index - 1, net_state_rx_iv_index_get((iv_index - 1) & 0x1));
    TEST_ASSERT_EQUAL(iv_index, net_state_rx_iv_index_get(iv_index & 0x1));
}

static void net_state_enable_trigger(void)
{
    mesh_config_entry_delete_ExpectAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID, NRF_SUCCESS);
    mesh_config_entry_delete_ExpectAndReturn(MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID, NRF_SUCCESS);
    net_state_enable();
}

static void start_from_scratch(void)
{
    m_synchro_index_tst = 0;

    m_expected_iv_idx_input.iv_index = 0;
    m_expected_iv_idx_input.iv_update_in_progress = NET_STATE_IV_UPDATE_NORMAL;
    m_expected_iv_idx_input.synchro_index = ++m_synchro_index_tst;
    m_expected_iv_idx_input.iv_update_timeout_counter = 0;

    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input, sizeof(m_expected_iv_idx_input), NRF_SUCCESS);

    m_expected_seqnum_input.next_block = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = m_synchro_index_tst;

    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);

    TEST_ASSERT_TRUE(net_state_is_seqnum_block_ready());
    net_state_enable_trigger();
    TEST_ASSERT_FALSE(net_state_is_seqnum_block_ready());

    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input);

    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
    TEST_ASSERT_TRUE(net_state_is_seqnum_block_ready());
}

static void start_from_persist_state(uint32_t iv_index, uint32_t seqnum, net_state_iv_update_t iv_update_state, uint32_t iv_update_timeout_minutes)
{
    m_expected_iv_idx_input.iv_index = iv_index;
    m_expected_iv_idx_input.iv_update_in_progress = iv_update_state;
    m_expected_iv_idx_input.synchro_index = ++m_synchro_index_tst;
    m_expected_iv_idx_input.iv_update_timeout_counter = iv_update_timeout_minutes;
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input);

    m_expected_seqnum_input.next_block = seqnum;
    m_expected_seqnum_input.synchro_index = m_synchro_index_tst;
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);

    m_expected_seqnum_input.next_block = seqnum + NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = m_synchro_index_tst;
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);

    TEST_ASSERT_TRUE(net_state_is_seqnum_block_ready());
    net_state_enable_trigger();
    TEST_ASSERT_FALSE(net_state_is_seqnum_block_ready());

    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);

    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
    TEST_ASSERT_TRUE(net_state_is_seqnum_block_ready());
}

static void net_state_module_reset(void)
{
    m_synchro_index_tst = 0;
    m_evt_handler_list_head = NULL;
    net_state_disable();
    mesh_config_entry_mock_Verify();
}

static void persist_state_check(uint32_t iv_index, uint32_t seqnum, bool in_iv_update, uint8_t iv_update_timeout_counter)
{
    mesh_opt_iv_index_persist_data_t expected_iv_idx_input;
    mesh_opt_seqnum_persist_data_t expected_seqnum_input;

    m_iv_index_params.callbacks.getter(MESH_OPT_NET_STATE_IV_INDEX_EID, &expected_iv_idx_input);
    m_seqnum_block_params.callbacks.getter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &expected_seqnum_input);

    TEST_ASSERT_TRUE(expected_iv_idx_input.iv_index == iv_index);
    TEST_ASSERT_TRUE(expected_iv_idx_input.iv_update_in_progress == (in_iv_update ? NET_STATE_IV_UPDATE_IN_PROGRESS : NET_STATE_IV_UPDATE_NORMAL));
    TEST_ASSERT_TRUE(expected_iv_idx_input.iv_update_timeout_counter == iv_update_timeout_counter);
    TEST_ASSERT_TRUE(expected_seqnum_input.next_block == seqnum);
    TEST_ASSERT_TRUE(expected_seqnum_input.synchro_index == expected_iv_idx_input.synchro_index);
}

static void beacon_rx(uint32_t iv_index, bool iv_update, bool key_refresh)
{
    nrf_mesh_beacon_info_t beacon_info;
    nrf_mesh_evt_t beacon_evt;
    nrf_mesh_rx_metadata_t rx_meta = {.source = NRF_MESH_RX_SOURCE_SCANNER};
    beacon_info.iv_update_permitted = true;

    beacon_evt.type = NRF_MESH_EVT_NET_BEACON_RECEIVED;
    beacon_evt.params.net_beacon.iv_index          = iv_index;
    beacon_evt.params.net_beacon.p_beacon_info     = &beacon_info;
    beacon_evt.params.net_beacon.p_beacon_secmat   = &beacon_info.secmat;
    beacon_evt.params.net_beacon.p_rx_metadata     = &rx_meta;
    beacon_evt.params.net_beacon.flags.iv_update   = iv_update;
    beacon_evt.params.net_beacon.flags.key_refresh = key_refresh;

    LIST_FOREACH(p_node, m_evt_handler_list_head)
    {
        nrf_mesh_evt_handler_t * p_evt_handler = PARENT_BY_FIELD_GET(nrf_mesh_evt_handler_t, node, p_node);
        p_evt_handler->evt_cb(&beacon_evt);
    }
}

static void drain_seqnums(void)
{
    uint32_t iv_index = 0;
    uint32_t seqnum = 0;
    uint32_t allocated_seqnums = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;

    /* First seqnum should be 0 after init, and following seqnums are sequential */
    for (uint32_t seq_expect = 0; seq_expect < NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD; ++seq_expect)
    {
        if (seq_expect == allocated_seqnums - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD)
        {
            allocated_seqnums += NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
            expect_config_seqnum(allocated_seqnums);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
        TEST_ASSERT_EQUAL(m_expected_iv_idx_input.iv_index, iv_index);
        TEST_ASSERT_EQUAL(seq_expect, seqnum);

        if (seq_expect == allocated_seqnums - NETWORK_SEQNUM_FLASH_BLOCK_SIZE - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD)
        {
            evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
        }
    }
}

/*****************************************************************************
* Tests
*****************************************************************************/

/** IV Update Summarized:
 * Can't start iv update after an init, conditions of an iv_update are:
 *    - Current state is Normal Operation state
 *    - No ongoing SAR transmissions.
 *    - 96 hours of operating in Normal Operation,
 *    - One of the two
 *      - About to exhaust sequence numbers (Must start 96 hours before the sequence numbers are
 *        exhausted if other conditions are met)
 *      - Receives a Secure Network beacon indicating that IV update is in progress.
 *
 * Can't complete IV update without meeting the listed conditions:
 *    - No ongoing SAR transmissions.
 *    - 96 hours of operation in IV Update in Progress state
 *    - One of the two:
 *      -- Receive a Secure Network beacon with IV Update Flag set to Normal Operation and IV index
 *         equal to or greater than current IV index
 *      -- The node has been in IV Update in Progress state for 144 hours.
 *
 * An IVI Recovery procedure can be started if the listed conditions are met:
 *    - Another IVI recovery has not been done in the last 192 hours.
 *    - The IVI of the received Secure Network beacon is greater than the current IVI of the node
 *      but not greater than current IVI + 42.
 */

void test_iv_user_initiated(void)
{
    /* Start from the scratch */
    start_from_scratch();

    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());

    /* No iv_update can be started due to the 96 hour limit */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* 3 minute drift should be expected */
    skip_minutes_and_expect_config_iv_data(96*60, current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    skip_minutes(2);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    skip_minutes(1);

    current_iv_index++;
    iv_update_start_trigger(current_iv_index);

    /* Already in progress */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* Must stay in iv update in progress state for at least 96 hours (3 mins of  drift added) */
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    /* Must not stay in iv update in progress state for more than 144 hours
    (18 mins of  drift deducted) */
    skip_minutes_and_expect_config_iv_data(48*60-18, current_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    /* skip one more, and the IV Update should finish */
    iv_update_state_normal_trigger(current_iv_index);
}

void test_iv_seqnum_initiated(void)
{
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t iv_index;
    uint32_t seqnum;

    /* We have to allocate sequence numbers in flash before we start transmitting */
    start_from_scratch();
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    drain_seqnums();
    verify_ivi_state_normal(current_iv_index);

    /* One more seqnum alloc and we should automatically start iv update. */
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(current_iv_index + 1, NET_STATE_IV_UPDATE_IN_PROGRESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    verify_ivi_state_in_progress(current_iv_index);

    /* Already in progress */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* Move to normal state with minimum amount of time passing (i.e.  Receive a Secure Network
     * beacon with IV Update Flag set to Normal Operation and iv index using the new index). */
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    event_handle_ExpectAnyArgs();

    expect_config_iv_data(current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    beacon_rx(current_iv_index, false, false);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    /* next allocated seqnum is reset to 0 */
    uint32_t new_seqnum;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &new_seqnum));
    TEST_ASSERT_EQUAL(1, iv_index);
    TEST_ASSERT_NOT_EQUAL(seqnum+1, new_seqnum);
    TEST_ASSERT_EQUAL(0, new_seqnum);
}

void test_iv_lock(void)
{
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t iv_index;
    uint32_t seqnum;

    /* We have to allocate sequence numbers in flash before we start transmitting */
    start_from_scratch();

    net_state_iv_index_lock(true);
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_NORMAL);

    /* First seqnum should be 0 after init, and following seqnums are sequential */
    uint32_t allocated_seqnums = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    for (uint32_t seq_expect = 0; seq_expect <= SEQ_NUM_MAX; ++seq_expect)
    {
        if (seq_expect == allocated_seqnums - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD &&
            seq_expect != SEQ_NUM_MAX)
        {
            allocated_seqnums += NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
            expect_config_seqnum(allocated_seqnums);
        }

        TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
        TEST_ASSERT_EQUAL(0, iv_index);
        TEST_ASSERT_EQUAL(seq_expect, seqnum);

        if (seq_expect == allocated_seqnums - NETWORK_SEQNUM_FLASH_BLOCK_SIZE - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD &&
            seq_expect != SEQ_NUM_MAX)
        {
            evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
        }
    }
    /* Can't allow a wrap-around */
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    /* Even though we're out of seq no and it's been more than 96 hours, no iv_update can be started
     * due to the lock. */
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    verify_ivi_state_normal(current_iv_index);

    /* Unlock should start the iv-update since we have depleted our seq no and spent 96 hours */
    current_iv_index++;
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(current_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    net_state_iv_index_lock(false);
    verify_ivi_state_in_progress(current_iv_index);

    /* Already in progress */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    /* Still using the old Seq No, which we are out of. */
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
}

void test_iv_beacons(void)
{
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t iv_index;
    uint32_t seqnum;

    /* We have to allocate sequence numbers in flash before we start transmitting */
    start_from_scratch();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    verify_ivi_state_normal(current_iv_index);

    /* The other guy is doing an iv update */
    beacon_rx(current_iv_index+1, true, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(1, seqnum);
    /* We will not follow until 96 hours has passed */
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(2, seqnum);
    /* A timer event after the limit has passed will trigger the state change */
    current_iv_index++;
    iv_update_state_in_progress_trigger(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(3, seqnum);
    verify_ivi_state_in_progress(current_iv_index);

    /* The other guy is already in normal state */
    beacon_rx(current_iv_index, false, false);
    verify_ivi_state_in_progress(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(4, seqnum);

    /* We will not follow until 96 hours has passed */
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    verify_ivi_state_in_progress(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(5, seqnum);
    /* A timer event after the limit has passed will trigger the state change */
    iv_update_state_normal_trigger(current_iv_index);
    /* Reset seqnum */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(1, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);

    /* Some other node is trying to catch up with the IV update procedure */
    beacon_rx(current_iv_index, true, false);
    verify_ivi_state_normal(current_iv_index);

    /* Time passes and we can do iv update again */
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(1, iv_index);
    TEST_ASSERT_EQUAL(1, seqnum);

    /* Some other node is behind on its updates and gets ignored */
    beacon_rx(current_iv_index, true, false);
    verify_ivi_state_normal(current_iv_index);

    /* The other guy starts before we do */
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(current_iv_index + 1, NET_STATE_IV_UPDATE_IN_PROGRESS);
    beacon_rx(current_iv_index+1, true, false);
    current_iv_index++;
    verify_ivi_state_in_progress(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(1, iv_index);
    TEST_ASSERT_EQUAL(2, seqnum);

    /* IV Update in Progress state is timed out. Switching to Normal state. */
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    skip_minutes_and_expect_config_iv_data(48*60 - 18, current_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    iv_update_state_normal_trigger(current_iv_index);

    /* Starting IV Recovery procedure */
    /* The other guy is way ahead of us for some reason */
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(current_iv_index+10, NET_STATE_IV_UPDATE_NORMAL);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    beacon_rx(current_iv_index+10, false, false);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
    current_iv_index += 10;
    verify_ivi_state_normal(current_iv_index);

    /* Reset seqnum */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(12, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);

    beacon_rx(current_iv_index+10, false, false);
    verify_ivi_state_normal(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(12, iv_index);
    TEST_ASSERT_EQUAL(1, seqnum);

    /* This time 96 hours is not sufficient since we can't have another IV Index recovery
     * within a period of 192 hours. */
    skip_minutes_and_expect_config_iv_data(96*60 + 4, current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    beacon_rx(current_iv_index+10, false, false);
    verify_ivi_state_normal(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(12, iv_index);
    TEST_ASSERT_EQUAL(2, seqnum);

    /* Another 96 hours and we are allowed to do IV Recovery again. */
    /* IV Update Normal Operation state timeout is passed, the timeout counter won't be changed anymore. */
    skip_minutes(96*60+3);
    verify_ivi_state_normal(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(12, iv_index);
    TEST_ASSERT_EQUAL(3, seqnum);

    /* Do a recovery with the IV update flag set */
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(current_iv_index+11, NET_STATE_IV_UPDATE_NORMAL);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    beacon_rx(current_iv_index + 11, true, false);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
    current_iv_index += 11;
    verify_ivi_state_normal(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(23, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);

    /* Elapse some time so we can do recovery again */
    skip_minutes_and_expect_config_iv_data(96*60 + 3, current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    /* IV Update Normal Operation state timeout is passed, the timeout counter won't be changed anymore. */
    skip_minutes(96*60 + 4);

    /* But we will not accept IV indices larger than NETWORK_IV_RECOVERY_LIMIT */
    beacon_rx(current_iv_index + NETWORK_IV_RECOVERY_LIMIT+1, false, false);
    verify_ivi_state_normal(current_iv_index);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(23, iv_index);
    TEST_ASSERT_EQUAL(1, seqnum);

    event_handle_ExpectAnyArgs();
    expect_config_iv_data(current_iv_index+NETWORK_IV_RECOVERY_LIMIT, NET_STATE_IV_UPDATE_NORMAL);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    beacon_rx(current_iv_index + NETWORK_IV_RECOVERY_LIMIT, false, false);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
    current_iv_index += NETWORK_IV_RECOVERY_LIMIT;
    verify_ivi_state_normal(current_iv_index);
    /* Reset seqnum */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(23 + NETWORK_IV_RECOVERY_LIMIT, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
}

void test_iv_testmode(void)
{
    /* We have to allocate sequence numbers in flash before we start transmitting */
    start_from_scratch();

    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t iv_index;
    uint32_t seqnum;
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    verify_ivi_state_normal(current_iv_index);

    /* No iv_update can be started due to the 96 hour limit */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    /* Set testmode on so we can ignore the 96 hour time limit */
    net_state_iv_update_test_mode_set(true);
    /* In test mode we should be able to start iv_update */
    current_iv_index++;
    iv_update_start_trigger(current_iv_index);

    /* Already in iv update state */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* We get a beacon with normal state, should switch despite the time limit not being met */
    event_handle_ExpectAnyArgs();
    expect_config_iv_data(current_iv_index, NET_STATE_IV_UPDATE_NORMAL);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    beacon_rx(current_iv_index, false, false);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);
    verify_ivi_state_normal(current_iv_index);

    /* Turn off test mode and 96 hours should be enforced again */
    net_state_iv_update_test_mode_set(false);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    /* Lock should stop iv update even in test mode */
    net_state_iv_update_test_mode_set(true);
    net_state_iv_index_lock(true);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* Start IV Update after unlock */
    net_state_iv_index_lock(false);
    current_iv_index++;
    iv_update_start_trigger(current_iv_index);

    /* Test mode off and we will not change states when a beacon arrives*/
    net_state_iv_update_test_mode_set(false);
    beacon_rx(current_iv_index, false, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
}

void test_mesh_config_recover(void)
{
    mesh_opt_iv_index_persist_data_t dummy_iv_idx;
    mesh_opt_seqnum_persist_data_t dummy_seqnum;
    uint32_t iv_index;
    uint32_t seqnum;

    /* We have to allocate sequence numbers in flash before we start transmitting */
    start_from_scratch();

    /* Now we get a sequence number */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);

    /* reset net state module */
    net_state_module_reset();
    /* Load again with different parameters */
    start_from_persist_state(0x1234, 0x5678, NET_STATE_IV_UPDATE_IN_PROGRESS, 0);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x1234 - 1, iv_index);
    TEST_ASSERT_EQUAL(0x5678, seqnum);
    TEST_ASSERT_EQUAL(0x1233, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    persist_state_check(0x1234, 0x5678 + NETWORK_SEQNUM_FLASH_BLOCK_SIZE, true, 0);

    /* reset net state module */
    net_state_module_reset();
    start_from_persist_state(0x7890, 0xABCD, NET_STATE_IV_UPDATE_NORMAL, 0);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x7890, iv_index);
    TEST_ASSERT_EQUAL(0xABCD, seqnum);
    TEST_ASSERT_EQUAL(0x7890, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    persist_state_check(0x7890, 0xABCD + NETWORK_SEQNUM_FLASH_BLOCK_SIZE, false, 0);

    /* reset net state module */
    net_state_module_reset();
    /* Find IV index entry after sequence number entry, should reset the sequence number */
    dummy_seqnum.next_block = 0x2000;
    dummy_seqnum.synchro_index = 1;
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &dummy_seqnum);
    dummy_iv_idx.iv_index = 0x1000;
    dummy_iv_idx.iv_update_in_progress = NET_STATE_IV_UPDATE_NORMAL;
    dummy_iv_idx.synchro_index = 2;
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &dummy_iv_idx);

    m_expected_seqnum_input.next_block = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = 2;
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x1000, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0x1000, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    /* Find IV index entry after sequence number entry, but be in IVU state. We should then adopt the sequence number */
    dummy_seqnum.next_block = 0x2000;
    dummy_seqnum.synchro_index = 1;
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &dummy_seqnum);
    dummy_iv_idx.iv_index = 0x1000;
    dummy_iv_idx.iv_update_in_progress = NET_STATE_IV_UPDATE_IN_PROGRESS;
    dummy_iv_idx.synchro_index = 2;
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &dummy_iv_idx);

    m_expected_seqnum_input.next_block = 0x2000 + NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = 2;
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x1000-1, iv_index);
    TEST_ASSERT_EQUAL(0x2000, seqnum);
    TEST_ASSERT_EQUAL(0x1000-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    /* Only find seqnum data, ignore it */
    dummy_seqnum.next_block = 0x2000;
    dummy_seqnum.synchro_index = 2;
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &dummy_seqnum);

    m_expected_iv_idx_input.iv_index = 0;
    m_expected_iv_idx_input.iv_update_in_progress = NET_STATE_IV_UPDATE_NORMAL;
    m_expected_iv_idx_input.synchro_index = ++m_synchro_index_tst;
    m_expected_seqnum_input.next_block = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = m_synchro_index_tst;

    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input, sizeof(m_expected_iv_idx_input), NRF_SUCCESS);
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    /* Only find IV index data, ignore it. */
    dummy_iv_idx.iv_index = 0;
    dummy_iv_idx.iv_update_in_progress = NET_STATE_IV_UPDATE_NORMAL;
    dummy_iv_idx.synchro_index = 1;
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &dummy_iv_idx);

    m_expected_iv_idx_input.iv_index = 0;
    m_expected_iv_idx_input.iv_update_in_progress = NET_STATE_IV_UPDATE_NORMAL;
    m_expected_iv_idx_input.synchro_index = ++m_synchro_index_tst;
    m_expected_seqnum_input.next_block = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = m_synchro_index_tst;

    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input, sizeof(m_expected_iv_idx_input), NRF_SUCCESS);
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    /* Don't find any data. */
    m_expected_iv_idx_input.iv_index = 0;
    m_expected_iv_idx_input.iv_update_in_progress = NET_STATE_IV_UPDATE_NORMAL;
    m_expected_iv_idx_input.synchro_index = ++m_synchro_index_tst;
    m_expected_seqnum_input.next_block = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = m_synchro_index_tst;

    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input, sizeof(m_expected_iv_idx_input), NRF_SUCCESS);
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    m_iv_index_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_EID, &m_expected_iv_idx_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    /* Data is restored from API then it tries to restore them from persist system. */
    const uint32_t test_iv_index = 1542;
    iv_index_set_trigger(test_iv_index, NET_STATE_IV_UPDATE_IN_PROGRESS);
    persist_state_check(test_iv_index, NETWORK_SEQNUM_FLASH_BLOCK_SIZE, true, 0);
    net_state_enable_trigger();
}

void test_reset(void)
{
    /* We have to allocate sequence numbers in flash before we start transmitting */
    start_from_persist_state(0x1234, 0x5000, NET_STATE_IV_UPDATE_NORMAL, 0);

    uint32_t iv_index;
    uint32_t seqnum;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x1234, iv_index);
    TEST_ASSERT_EQUAL(0x5000, seqnum);
    TEST_ASSERT_EQUAL(0x1234, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    mesh_config_file_clear_Expect(MESH_OPT_NET_STATE_FILE_ID);
    m_seqnum_block_params.callbacks.deleter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID);
    m_iv_index_params.callbacks.deleter(MESH_OPT_NET_STATE_IV_INDEX_EID);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);

    net_state_reset();
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
}

void test_iv_index_set(void)
{
    /* Start from the scratch */
    start_from_scratch();

    const uint32_t TEST_IV_INDEX = 1542;
    iv_index_set_trigger(TEST_IV_INDEX, NET_STATE_IV_UPDATE_IN_PROGRESS);

    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_index_set(0, false));

    /* -1 because the IV index update is in progress */
    TEST_ASSERT_EQUAL(TEST_IV_INDEX - 1 , net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
}

void test_testmode_transition_run(void)
{
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;

    /* Start from the scratch */
    start_from_scratch();

    event_handle_Ignore();

    /* Set testmode on so we can ignore the 96 hour time limit */
    net_state_iv_update_test_mode_set(true);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_test_mode_transition_run(NET_STATE_TO_NORMAL_SIGNAL));
    expect_config_iv_data(current_iv_index + 1, NET_STATE_IV_UPDATE_IN_PROGRESS);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_test_mode_transition_run(NET_STATE_TO_IV_UPDATE_IN_PROGRESS_SIGNAL));
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_test_mode_transition_run(NET_STATE_TO_IV_UPDATE_IN_PROGRESS_SIGNAL));
    expect_config_iv_data(current_iv_index + 1, NET_STATE_IV_UPDATE_NORMAL);
    expect_config_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_test_mode_transition_run(NET_STATE_TO_NORMAL_SIGNAL));
    /* Clear testmode on so we can ignore the 96 hour time limit */
    net_state_iv_update_test_mode_set(false);
}

void test_restoring_from_legacy(void)
{
    uint32_t iv_index;
    uint32_t seqnum;
    mesh_opt_seqnum_persist_data_legacy_t dummy_legacy_seqnum;
    mesh_opt_iv_index_persist_data_legacy_t dummy_legacy_iv_idx;
    // both legacy parameters are restored
    // Find IV index entry before sequence number entry.
    dummy_legacy_iv_idx.iv_index = 0x1000;
    dummy_legacy_iv_idx.iv_update_in_progress = 0;
    m_iv_index_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID, &dummy_legacy_iv_idx);
    dummy_legacy_seqnum.next_block = 0x2000;
    m_seqnum_block_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID, &dummy_legacy_seqnum);

    m_expected_seqnum_input.next_block = 0x2000 + NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = 0;
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x1000, iv_index);
    TEST_ASSERT_EQUAL(0x2000, seqnum);
    TEST_ASSERT_EQUAL(0x1000, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    // Find IV index entry before sequence number entry in IVU state.
    dummy_legacy_iv_idx.iv_index = 0x1000;
    dummy_legacy_iv_idx.iv_update_in_progress = 1;
    m_iv_index_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID, &dummy_legacy_iv_idx);
    dummy_legacy_seqnum.next_block = 0x2000;
    m_seqnum_block_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID, &dummy_legacy_seqnum);

    m_expected_seqnum_input.next_block = 0x2000 + NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = 0;
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x1000-1, iv_index);
    TEST_ASSERT_EQUAL(0x2000, seqnum);
    TEST_ASSERT_EQUAL(0x1000-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    // Find sequence number entry before IV index entry.
    dummy_legacy_seqnum.next_block = 0x2000;
    m_seqnum_block_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID, &dummy_legacy_seqnum);
    dummy_legacy_iv_idx.iv_index = 0x1000;
    dummy_legacy_iv_idx.iv_update_in_progress = 0;
    m_iv_index_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID, &dummy_legacy_iv_idx);

    m_expected_seqnum_input.next_block = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    m_expected_seqnum_input.synchro_index = 0;
    mesh_config_entry_set_ExpectWithArrayAndReturn(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input, sizeof(m_expected_seqnum_input), NRF_SUCCESS);
    net_state_enable_trigger();
    m_seqnum_block_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_EID, &m_expected_seqnum_input);
    evt_notify(NRF_MESH_EVT_CONFIG_STABLE);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0x1000, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0x1000, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    // Find only sequence number entry. Inconsistent data, start from scratch is expected.
    dummy_legacy_seqnum.next_block = 0x2000;
    m_seqnum_block_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_SEQ_NUM_BLOCK_LEGACY_EID, &dummy_legacy_seqnum);

    start_from_scratch();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* reset net state module */
    net_state_module_reset();
    // Find only IV index entry. Inconsistent data, start from scratch is expected.
    dummy_legacy_iv_idx.iv_index = 0x1000;
    dummy_legacy_iv_idx.iv_update_in_progress = 0;
    m_iv_index_legacy_params.callbacks.setter(MESH_OPT_NET_STATE_IV_INDEX_LEGACY_EID, &dummy_legacy_iv_idx);

    start_from_scratch();

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, iv_index);
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
}

void test_verify_timeout_counter_restored_after_restart(void)
{
    uint32_t iv_index;
    uint32_t seqnum = 0;

    struct {
        uint32_t restore_time;
        uint32_t current_iv_index;
        uint32_t next_iv_index;
        uint32_t state_timeout;
        net_state_iv_update_t current_iv_update_state;
        net_state_iv_update_t next_iv_update_state;
    } variants[] = {
        {20 * CALCULATED_IV_UPDATE_TIMEOUT_PERIODIC_SAVE_MINUTES, 0, 1, 96*60 + 3,       NET_STATE_IV_UPDATE_NORMAL,      NET_STATE_IV_UPDATE_IN_PROGRESS},
        {20 * CALCULATED_IV_UPDATE_TIMEOUT_PERIODIC_SAVE_MINUTES, 1, 1, 144*60 - 5 - 10, NET_STATE_IV_UPDATE_IN_PROGRESS, NET_STATE_IV_UPDATE_NORMAL},
    };

    for (size_t var = 0; var < ARRAY_SIZE(variants); var++)
    {
        seqnum = 0;

        /* Restore the net_state as it allocated enough SEQNUMs to trigger IV Update,
         * but the time for IV Update is not elapsed. This shall not trigger IV Update,
         * but shall set the pending flag. */
        m_iv_update_timeout_counter = variants[var].restore_time;
        start_from_persist_state(variants[var].current_iv_index, NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD, variants[var].current_iv_update_state, m_iv_update_timeout_counter);

        /* This shall not trigger IV Update. */
        TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
        TEST_ASSERT_EQUAL(NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD, seqnum);

        if (variants[var].current_iv_update_state == NET_STATE_IV_UPDATE_NORMAL)
        {
            TEST_ASSERT_EQUAL(variants[var].current_iv_index, iv_index);
        }
        else
        {
            TEST_ASSERT_EQUAL(variants[var].current_iv_index - 1, iv_index);
        }

        /* Skip the remaining time before IV Update state transition. */
        skip_minutes_and_expect_config_iv_data(variants[var].state_timeout - m_iv_update_timeout_counter, variants[var].current_iv_index, variants[var].current_iv_update_state);

        /* Now IV Update shall be triggered. */
        if (variants[var].next_iv_update_state == NET_STATE_IV_UPDATE_NORMAL)
        {
            iv_update_state_normal_trigger(variants[var].next_iv_index);
        }
        else
        {
            iv_update_state_in_progress_trigger(variants[var].next_iv_index);
        }

        net_state_module_reset();
    }
}

void test_provisioning_iv_update_normal(void)
{
    uint32_t iv_index;
    uint32_t seqnum;

    /* Start from the scratch. */
    start_from_scratch();

    /* Provision the device. */
    const uint32_t TEST_IV_INDEX = 1542;
    iv_index_set_trigger(TEST_IV_INDEX, NET_STATE_IV_UPDATE_NORMAL);

    /* Drain seqnums so that the next seqnum allocation will trigger IV Update. */
    drain_seqnums();

    /* Allocate one more seqnum to set the pending flag to true, but
     * this shall not trigger IV Update. */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(TEST_IV_INDEX, iv_index);
    TEST_ASSERT_EQUAL(NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD, seqnum);

    /* Wait for IV Update process to be ready to transition to Normal Operation state.
     * This shall not change IV Update state yet. */
    skip_minutes_and_expect_config_iv_data(96*60+3, TEST_IV_INDEX, NET_STATE_IV_UPDATE_NORMAL);

    /* This shall change IV Update state from Normal Operation to Update in Progress. */
    iv_update_state_in_progress_trigger(TEST_IV_INDEX + 1);
}

void test_provisioning_iv_update_in_progress(void)
{
    /* Start from the scratch. */
    start_from_scratch();

    /* Provision the device when IV Update in Progress. */
    const uint32_t TEST_IV_INDEX = 1542;
    iv_index_set_trigger(TEST_IV_INDEX, NET_STATE_IV_UPDATE_IN_PROGRESS);

    /* Wait for IV Update process to be ready to transition to Normal Operation state.
     * This shall not change IV Update state yet. */
    skip_minutes_and_expect_config_iv_data(144*60-15, TEST_IV_INDEX, NET_STATE_IV_UPDATE_IN_PROGRESS);

    /* This shall change IV Update state from Update in Progress to Normal Operation. */
    iv_update_state_normal_trigger(TEST_IV_INDEX);
}

void test_seqnum_protection(void)
{
    uint32_t iv_index;
    uint32_t seqnum;

    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    /* Start from the scratch. */
    start_from_scratch();
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_index_and_seqnum_alloc(&iv_index, &seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
}
