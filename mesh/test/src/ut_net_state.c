/* Copyright (c) 2010 - 2017, Nordic Semiconductor ASA
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

#include "net_state.h"
#include "timer_scheduler.h"
#include "event_mock.h"
#include "flash_manager_mock.h"

/*****************************************************************************
* Defines
*****************************************************************************/
#define SEQ_NUM_MAX 0xFFFFFF
#define EXPECTED_NET_STATE_TIMER_RES 60000000 /* From net_state.c */
#define BUFFER_SEAL 0xCABACABA
#define MEMORY_LISTENERS_MAX 2
#define HANDLE_SEQNUM  0x0001
#define HANDLE_IV_DATA 0x0002
/*****************************************************************************
* UT globals
*****************************************************************************/
static timer_event_t *mp_iv_update_timer;
static int m_critical_section_counter;
static flash_manager_page_t m_flash_pages[2];
static flash_manager_t * mp_manager;
static fm_entry_t * mp_expected_seqnum_flash_buffer;
static fm_entry_t * mp_expected_iv_data_flash_buffer;
static uint32_t m_flash_buffer[4];
static bool m_did_dummy_seqnum_block_alloc;
static fm_mem_listener_t * mp_listeners[MEMORY_LISTENERS_MAX];
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

static uint32_t flash_manager_add_callback(flash_manager_t * p_manager, const flash_manager_config_t * p_config, int calls)
{
    TEST_ASSERT_NOT_NULL(p_manager);
    TEST_ASSERT_NOT_NULL(p_config);
    mp_manager = p_manager;
    memcpy(&p_manager->config, p_config, sizeof(flash_manager_config_t));
    return NRF_SUCCESS;
}

static fm_entry_t * flash_manager_entry_alloc_dummy(flash_manager_t * p_manager, fm_handle_t handle, uint32_t data_size, int calls)
{
    TEST_ASSERT_EQUAL_PTR(mp_manager, p_manager);
    TEST_ASSERT_TRUE(handle == HANDLE_SEQNUM || handle == HANDLE_IV_DATA);
    TEST_ASSERT_TRUE(data_size <= 8);
    fm_entry_t * p_entry = (fm_entry_t *) m_flash_buffer;
    p_entry->header.handle = handle;
    p_entry->header.len_words = (data_size + 3) / 4 + 1;
    if (handle == HANDLE_SEQNUM)
    {
        m_did_dummy_seqnum_block_alloc = true;
    }
    return p_entry;
}

static void flash_manager_mem_listener_register_callback(fm_mem_listener_t * p_listener, int calls)
{
    TEST_ASSERT_NOT_NULL(p_listener);
    TEST_ASSERT_NOT_NULL(p_listener->callback);
    for (uint32_t i = 0; i < MEMORY_LISTENERS_MAX; i++)
    {
        if (mp_listeners[i] == NULL)
        {
            mp_listeners[i] = p_listener;
            return;
        }
    }
    TEST_FAIL_MESSAGE("Not enough memory listeners created, increase it in the test.");

}

void fire_mem_listeners(void)
{
    for (uint32_t i = 0; i < MEMORY_LISTENERS_MAX; i++)
    {
        if (mp_listeners[i] != NULL)
        {
            fm_mem_listener_t * p_listener = mp_listeners[i];
            mp_listeners[i] = NULL;
            p_listener->callback(p_listener->p_args);
        }
    }
}

void event_handle_flash_fail_callback(nrf_mesh_evt_t * p_evt, int calls)
{
    TEST_ASSERT_NOT_NULL(p_evt);
    TEST_ASSERT_EQUAL(NRF_MESH_EVT_FLASH_FAILED, p_evt->type);
    TEST_ASSERT_EQUAL(mp_manager->config.p_area, p_evt->params.flash_failed.p_area);
    TEST_ASSERT_EQUAL(NRF_MESH_FLASH_USER_CORE, p_evt->params.flash_failed.user);
    TEST_ASSERT_EQUAL(NET_FLASH_PAGE_COUNT, p_evt->params.flash_failed.page_count);
}
/*****************************************************************************
* Setup functions
*****************************************************************************/

void setUp(void)
{
    m_critical_section_counter = 0;
    mp_iv_update_timer = NULL;
    event_mock_Init();
    flash_manager_mock_Init();

    flash_manager_recovery_page_get_ExpectAndReturn(&m_flash_pages[1]);
    flash_manager_add_StubWithCallback(flash_manager_add_callback);

    net_state_init();
}

void tearDown(void)
{
    event_mock_Verify();
    TEST_ASSERT_EQUAL(0, m_critical_section_counter);
    event_mock_Verify();
    event_mock_Destroy();
    flash_manager_mock_Verify();
    flash_manager_mock_Destroy();
}

static void m_skip_minutes(uint32_t minutes)
{
    for (uint32_t i = 0; i < minutes; ++i)
    {
        mp_iv_update_timer->cb(0, NULL);
    }
}

void expect_flash_seqnum(uint32_t seqnum)
{
    static uint32_t buffer[3];
    fm_entry_t * p_entry = (fm_entry_t *) buffer;
    p_entry->header.handle = HANDLE_SEQNUM;
    p_entry->header.len_words = 2;
    buffer[2] = BUFFER_SEAL; /* Make a seal to detect out-of-bounds writing. */
    flash_manager_entry_alloc_ExpectAndReturn(mp_manager, HANDLE_SEQNUM, 4, p_entry);

    static uint32_t expected_buffer[3];
    mp_expected_seqnum_flash_buffer = (fm_entry_t *) expected_buffer;
    mp_expected_seqnum_flash_buffer->header.handle = p_entry->header.handle;
    mp_expected_seqnum_flash_buffer->header.len_words = p_entry->header.len_words;
    mp_expected_seqnum_flash_buffer->data[0] = seqnum;
    expected_buffer[2] = BUFFER_SEAL;
    flash_manager_entry_commit_ExpectWithArray(mp_expected_seqnum_flash_buffer, 3);
}

void expect_flash_iv_data(uint32_t iv_index, bool in_iv_update)
{
    static uint32_t buffer[4];
    fm_entry_t * p_entry = (fm_entry_t *) buffer;
    p_entry->header.handle = HANDLE_IV_DATA;
    p_entry->header.len_words = 3;
    buffer[3] = BUFFER_SEAL; /* Make a seal to detect out-of-bounds writing. */
    flash_manager_entry_alloc_ExpectAndReturn(mp_manager, HANDLE_IV_DATA, 8, p_entry);

    static uint32_t expected_buffer[4];
    mp_expected_iv_data_flash_buffer = (fm_entry_t *) expected_buffer;
    mp_expected_iv_data_flash_buffer->header.handle = p_entry->header.handle;
    mp_expected_iv_data_flash_buffer->header.len_words = p_entry->header.len_words;
    mp_expected_iv_data_flash_buffer->data[0] = iv_index;
    mp_expected_iv_data_flash_buffer->data[1] = (uint8_t) in_iv_update;
    expected_buffer[3] = BUFFER_SEAL;
    flash_manager_entry_commit_ExpectWithArray(mp_expected_iv_data_flash_buffer, 4);
}

void expect_flash_load(uint32_t iv_index, uint32_t seqnum, bool in_iv_update)
{
    static uint32_t buffer[5];
    fm_entry_t * p_iv_entry = (fm_entry_t *) buffer;
    fm_entry_t * p_seqnum_entry = (fm_entry_t *) &buffer[3];
    p_iv_entry->header.handle = 2;
    p_iv_entry->header.len_words = 3;
    p_iv_entry->data[0] = iv_index;
    p_iv_entry->data[1] = in_iv_update;
    p_seqnum_entry->header.handle = 1;
    p_seqnum_entry->header.len_words = 2;
    p_seqnum_entry->data[0] = seqnum;
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, NULL, p_iv_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_iv_entry, p_seqnum_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_seqnum_entry, NULL);
    expect_flash_seqnum(seqnum + NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
}

static void notify_flash_write_complete(fm_entry_t * p_entry)
{
    m_did_dummy_seqnum_block_alloc = false;
    mp_manager->config.write_complete_cb(mp_manager,
                                         p_entry,
                                         FM_RESULT_SUCCESS);
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
 *      - Receives a Secure Netowrk beacon indicating that IV update is in progress.
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
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());

    /* No iv_update can be started due to the 96 hour limit */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* 3 minute drift should be expected */
    m_skip_minutes(96*60);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    m_skip_minutes(2);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    m_skip_minutes(1);

    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index + 1, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_update_start());
    notify_flash_write_complete(mp_expected_iv_data_flash_buffer);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(1));
    /* Already in progress */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* Must stay in iv update in progress state for at least 96 hours (3 mins of  drift added) */
    m_skip_minutes(96*60 + 3);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    /* Must not stay in iv update in progress state for more than 144 hours
    (18 mins of  drift deducted) */
    m_skip_minutes(48*60-18);
    /* skip one more, and the IV Update should finish */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    m_skip_minutes(1);
    notify_flash_write_complete(mp_expected_iv_data_flash_buffer);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(1));
}


void test_iv_seqnum_initiated(void)
{
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t seqnum;
    m_skip_minutes(96*60 + 3);

    /* We have to allocate sequence numbers in flash before we start transmitting */
    expect_flash_load(0, 0, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    uint32_t allocated_seqnums = NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
    flash_manager_mock_Verify();

    /* First seqnum should be 0 after init, and following seqnums are sequential */
    uint32_t seq_expect = 0;
    for (; seq_expect < NETWORK_SEQNUM_FLASH_BLOCK_SIZE * 2; ++seq_expect)
    {
        for (; seq_expect < allocated_seqnums - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD; ++seq_expect) /*lint !e445 Weird, but intentional */
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
            TEST_ASSERT_EQUAL(seq_expect, seqnum);
            flash_manager_mock_Verify();
        }
        /* Once we start running out of allocated seqnums, we expect an alloc call. */
        allocated_seqnums += NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
        expect_flash_seqnum(allocated_seqnums);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
        TEST_ASSERT_EQUAL(seq_expect, seqnum);

        notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
        flash_manager_mock_Verify();
    }
    /* CMock runs out of memory if we let it run through all seqnums with the alloc calls. Ignore the rest of the alloc calls */
    flash_manager_entry_alloc_StubWithCallback(flash_manager_entry_alloc_dummy);
    flash_manager_entry_commit_Ignore();
    for (; seq_expect < NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD;)
    {
        for (; seq_expect <= allocated_seqnums - NETWORK_SEQNUM_FLASH_BLOCK_THRESHOLD && seq_expect < NETWORK_SEQNUM_IV_UPDATE_START_THRESHOLD; ++seq_expect)
        {
            TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
            TEST_ASSERT_EQUAL(seq_expect, seqnum);
            flash_manager_mock_Verify();
        }
        /* Once we start running out of allocated seqnums, we expect an alloc call. */
        if (m_did_dummy_seqnum_block_alloc)
        {
            allocated_seqnums += NETWORK_SEQNUM_FLASH_BLOCK_SIZE;
            notify_flash_write_complete((fm_entry_t *) m_flash_buffer);
        }
    }
    flash_manager_entry_alloc_StubWithCallback(NULL);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In normal iv_update state we accept from iv indices n, n-1 */
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL((uint32_t)(current_iv_index-1), net_state_rx_iv_index_get(1));

    /* One more seqnum alloc and we should automatically start iv update. */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index + 1, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(1));
    /* Already in progress */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());

    /* Move to normal state with minimum amount of time passing (i.e.  Receive a Secure Network
     * beacon with IV Update Flag set to Normal Operation and iv index using the new index). */
    m_skip_minutes(96*60 + 3);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    event_handle_Ignore();

    expect_flash_iv_data(current_iv_index, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    net_state_beacon_received(current_iv_index, false, false);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    /* next allocated seqnum is reset to 0 */
    uint32_t new_seqnum;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&new_seqnum));
    TEST_ASSERT_NOT_EQUAL(seqnum+1, new_seqnum);
    TEST_ASSERT_EQUAL(0, new_seqnum);
}


void test_iv_lock(void)
{
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t seqnum;

    /* We have to allocate sequence numbers in flash before we start transmitting */
    expect_flash_load(0, 0, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    net_state_iv_index_lock(true);
    m_skip_minutes(96*60 + 3);

    /* First seqnum should be 0 after init, and following seqnums are sequential */
    flash_manager_entry_alloc_StubWithCallback(flash_manager_entry_alloc_dummy);
    flash_manager_entry_commit_Ignore();
    for (int seq_expect = 0; seq_expect <= SEQ_NUM_MAX; ++seq_expect)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
        TEST_ASSERT_EQUAL(seq_expect, seqnum);

        if (m_did_dummy_seqnum_block_alloc)
        {
            notify_flash_write_complete((fm_entry_t *) m_flash_buffer);
        }
    }
    /* Can't allow a wrap-around */
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_seqnum_alloc(&seqnum));
    /* Even though we're out of seq no and it's been more than 96 hours, no iv_update can be started
     * due to the lock. */
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In normal iv_update state we accept from iv indices n, n-1 */
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL((uint32_t)(current_iv_index-1), net_state_rx_iv_index_get(1));

    /* Unlock should start the iv-update since we have depleted our seq no and spent 96 hours */
    event_handle_Ignore();
    net_state_iv_index_lock(false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL((uint32_t)(current_iv_index), net_state_rx_iv_index_get(1));
    /* Already in progress */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    /* Still using the old Seq No, which we are out of. */
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_seqnum_alloc(&seqnum));
}

void test_iv_beacons(void)
{
    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t seqnum;

    /* We have to allocate sequence numbers in flash before we start transmitting */
    expect_flash_load(0, 0, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL((uint32_t)(current_iv_index-1), net_state_rx_iv_index_get(1));

    /* The other guy is doing an iv update */
    net_state_beacon_received(current_iv_index+1, true, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(1, seqnum);
    /* We will not follow until 96 hours has passed */
    m_skip_minutes(96*60 + 3);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(2, seqnum);
    /* A timer event after the limit has passed will trigger the state change */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index + 1, true);
    m_skip_minutes(1);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(3, seqnum);
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL((uint32_t)(current_iv_index), net_state_rx_iv_index_get(1));

    /* The other guy is already in normal state */
    net_state_beacon_received(current_iv_index, false, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(4, seqnum);

    /* We will not follow until 96 hours has passed */
    m_skip_minutes(96*60 + 3);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(5, seqnum);
    /* A timer event after the limit has passed will trigger the state change */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    m_skip_minutes(1);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(1));
    /* Reset seqnum */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);

    /* Some other node is trying to catch up with the IV update procedure */
    net_state_beacon_received(current_iv_index, true, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* Time passes and we can do iv update again */
    m_skip_minutes(96*60 + 3);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(1, seqnum);

    /* Some other node is behind on its updates and gets ignored */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index + 1, true);
    net_state_beacon_received(current_iv_index, true, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* The other guy starts before we do */
    event_handle_Ignore();

    net_state_beacon_received(current_iv_index+1, true, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(2, seqnum);
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept from iv indices m-1, m */
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL((uint32_t)(current_iv_index - 1), net_state_rx_iv_index_get(1));

    m_skip_minutes(96*60 + 4);
    expect_flash_iv_data(current_iv_index, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    m_skip_minutes(96*60 + 4);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* The other guy is way ahead of us for some reason */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index+10, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    net_state_beacon_received(current_iv_index+10, false, false);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    current_iv_index += 10;
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(1));

    /* Reset seqnum */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);

    net_state_beacon_received(current_iv_index+10, false, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(1, seqnum);

    /* This time 96 hours is not sufficient since we can't have another IV Index recovery
     * within a period of 192 hours. */
    m_skip_minutes(96*60 + 4);
    net_state_beacon_received(current_iv_index+10, false, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(2, seqnum);

    /* Another 96 hours and we are allowed to do IV Recovery again */
    m_skip_minutes(96*60 + 4);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(1));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(3, seqnum);

    /* Do a recovery with the IV update flag set */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index+11, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    net_state_beacon_received(current_iv_index + 11, true, false);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    current_iv_index += 11;
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);

    /* Elapse some time so we can do recovery again */
    m_skip_minutes(96 * 60 + 3);
    m_skip_minutes(96 * 60 + 4);

    /* But we will not accept IV indices larger than NETWORK_IV_RECOVERY_LIMIT */
    net_state_beacon_received(current_iv_index + NETWORK_IV_RECOVERY_LIMIT+1, false, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(1, seqnum);
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index+NETWORK_IV_RECOVERY_LIMIT, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    net_state_beacon_received(current_iv_index + NETWORK_IV_RECOVERY_LIMIT, false, false);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    current_iv_index += NETWORK_IV_RECOVERY_LIMIT;
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* Reset seqnum */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
}


void test_iv_testmode(void)
{

    /* We have to allocate sequence numbers in flash before we start transmitting */
    expect_flash_load(0, 0, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    /* iv index is always 0 at init */
    uint32_t current_iv_index = 0;
    uint32_t seqnum;
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL((uint32_t)(current_iv_index-1), net_state_rx_iv_index_get(1));

    /* No iv_update can be started due to the 96 hour limit */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    /* Set testmode on so we can ignore the 96 hour time limit */
    net_state_iv_update_test_mode_set(true);
    /* In test mode we should be able to start iv_update */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index+1, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_update_start());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    /* In update-in-progress iv_update state we accept iv indices m-1, m */
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(1));

    /* Already in iv update state */
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    /* We get a beacon with normal state, should switch despite the time limit not being met */
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    net_state_beacon_received(current_iv_index, false, false);
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(1));

    /* Turn off test mode and 96 hours should be enforced again */
    net_state_iv_update_test_mode_set(false);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    /* Lock should stop iv update even in test mode */
    net_state_iv_update_test_mode_set(true);
    net_state_iv_index_lock(true);
    TEST_ASSERT_EQUAL(NRF_ERROR_INVALID_STATE, net_state_iv_update_start());
    net_state_iv_index_lock(false);
    event_handle_Ignore();
    expect_flash_iv_data(current_iv_index+1, true);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_update_start());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    current_iv_index++;
    /* TX iv index has not changed yet*/
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_beacon_iv_index_get());
    TEST_ASSERT_EQUAL(current_iv_index, net_state_rx_iv_index_get(0));
    TEST_ASSERT_EQUAL(current_iv_index-1, net_state_rx_iv_index_get(1));

    /* Test mode off and we will not change states when a beacon arrives*/
    net_state_iv_update_test_mode_set(false);
    net_state_beacon_received(current_iv_index, false, false);
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
}

void test_flash_recover(void)
{
    /* before recovering, we're unable to allocate a sequence number, but we'll trigger an allocation call */
    uint32_t seqnum;
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_seqnum_alloc(&seqnum));
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    /* We have to allocate sequence numbers in flash before we start transmitting */
    expect_flash_load(0, 0, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    /* Now we get a sequence number */
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);

    flash_manager_mock_Verify();
    /* Load again with different parameters */
    expect_flash_load(0x1234, 0x5678, true);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0x5678, seqnum);
    TEST_ASSERT_EQUAL(0x1233, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());


    expect_flash_load(0x7890, 0xABCD, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0xABCD, seqnum);
    TEST_ASSERT_EQUAL(0x7890, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* Find IV index entry after sequence number entry, should reset the sequence number */
    static uint32_t buffer[5];
    fm_entry_t * p_seqnum_entry = (fm_entry_t *) &buffer[0];
    fm_entry_t * p_iv_entry = (fm_entry_t *) &buffer[2];
    p_iv_entry->header.handle = 2;
    p_iv_entry->header.len_words = 3;
    p_iv_entry->data[0] = 0x1000;
    p_iv_entry->data[1] = NET_STATE_IV_UPDATE_NORMAL;
    p_seqnum_entry->header.handle = 1;
    p_seqnum_entry->header.len_words = 2;
    p_seqnum_entry->data[0] = 0x2000;
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, NULL, p_seqnum_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_seqnum_entry, p_iv_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_iv_entry, NULL);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);

    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0x1000, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* Find IV index entry after sequence number entry, but be in IVU state. We should then adopt the sequence number */

    p_iv_entry->data[1] = NET_STATE_IV_UPDATE_IN_PROGRESS;
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, NULL, p_seqnum_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_seqnum_entry, p_iv_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_iv_entry, NULL);
    expect_flash_seqnum(0x2000 + NETWORK_SEQNUM_FLASH_BLOCK_SIZE);

    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0x2000, seqnum);
    TEST_ASSERT_EQUAL(0x1000-1, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_IN_PROGRESS, net_state_iv_update_get());
    flash_manager_mock_Verify();
    /* Only find seqnum data, ignore it */
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, NULL, p_seqnum_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_seqnum_entry, NULL);
    expect_flash_iv_data(0, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);

    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* Only find IV index data, ignore it. */
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, NULL, p_iv_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_iv_entry, NULL);
    expect_flash_iv_data(0, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);

    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* Don't find any data. */
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, NULL, NULL);
    expect_flash_iv_data(0, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);

    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
}

void test_flash_failures(void)
{
    event_handle_Ignore();
    /* We have to allocate sequence numbers in flash before we start transmitting */
    expect_flash_load(0, 0, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    uint32_t seqnum;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    /* setup for easy testing */
    net_state_iv_update_test_mode_set(true);
    flash_manager_mem_listener_register_StubWithCallback(flash_manager_mem_listener_register_callback);

    /* Fail at allocating a new flash iv index data structure, it should retry with a mem listener */
    flash_manager_entry_alloc_ExpectAndReturn(mp_manager, HANDLE_IV_DATA, 8, NULL);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_iv_update_start());
    TEST_ASSERT_NOT_NULL(mp_listeners[0]);

    /* succeed on the retry */
    expect_flash_iv_data(1, true);
    fire_mem_listeners();
    flash_manager_mock_Verify();

    /* Do the same procedure on the IV update end, this time fail both IV index alloc and seqnum alloc */
    flash_manager_entry_alloc_ExpectAndReturn(mp_manager, HANDLE_IV_DATA, 8, NULL);
    flash_manager_entry_alloc_ExpectAndReturn(mp_manager, HANDLE_SEQNUM, 4, NULL);
    net_state_beacon_received(1, false, false);
    /* should have registered two listeners */
    TEST_ASSERT_NOT_NULL(mp_listeners[0]);
    TEST_ASSERT_NOT_NULL(mp_listeners[1]);

    /* succeed on retry for both */
    expect_flash_iv_data(1, false);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    fire_mem_listeners();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);
    flash_manager_mock_Verify();

    /* fail init */
    flash_manager_add_StubWithCallback(NULL);

    flash_manager_recovery_page_get_ExpectAndReturn(&m_flash_pages[1]);
    flash_manager_add_ExpectAndReturn(mp_manager, NULL, NRF_ERROR_NO_MEM);
    flash_manager_add_IgnoreArg_p_config();
    net_state_init();
    TEST_ASSERT_NOT_NULL(mp_listeners[0]);
    TEST_ASSERT_NULL(mp_listeners[1]);

    flash_manager_recovery_page_get_ExpectAndReturn(&m_flash_pages[1]);
    flash_manager_add_ExpectAndReturn(mp_manager, NULL, NRF_ERROR_NO_MEM);
    flash_manager_add_IgnoreArg_p_config();
    fire_mem_listeners();
    TEST_ASSERT_NOT_NULL(mp_listeners[0]);
    TEST_ASSERT_NULL(mp_listeners[1]);

    flash_manager_recovery_page_get_ExpectAndReturn(&m_flash_pages[1]);
    flash_manager_add_ExpectAndReturn(mp_manager, NULL, NRF_SUCCESS);
    flash_manager_add_IgnoreArg_p_config();
    fire_mem_listeners();
    TEST_ASSERT_NULL(mp_listeners[0]);

    /* flash error result */
    fm_entry_t entry;
    entry.header.handle = HANDLE_SEQNUM;
    entry.header.len_words = 1;

    event_handle_StubWithCallback(event_handle_flash_fail_callback);
    mp_manager->config.write_complete_cb(mp_manager, &entry, FM_RESULT_ERROR_FLASH_MALFUNCTION);
    TEST_NRF_MESH_ASSERT_EXPECT(mp_manager->config.write_complete_cb(mp_manager, &entry, FM_RESULT_ERROR_AREA_FULL));
    TEST_NRF_MESH_ASSERT_EXPECT(mp_manager->config.write_complete_cb(mp_manager, &entry, FM_RESULT_ERROR_NOT_FOUND));
    flash_manager_mock_Verify();

    /* Unexpected flash data */
    static uint32_t buffer[7];
    fm_entry_t * p_useless_noise = (fm_entry_t *) &buffer[0];
    fm_entry_t * p_seqnum_entry = (fm_entry_t *) &buffer[2];
    fm_entry_t * p_iv_entry = (fm_entry_t *) &buffer[4];
    p_useless_noise[0].header.handle = 0x1234;
    p_useless_noise[1].header.handle = 0x19ab;
    p_iv_entry->header.handle = 2;
    p_iv_entry->header.len_words = 3;
    p_iv_entry->data[0] = 0x1000;
    p_iv_entry->data[1] = NET_STATE_IV_UPDATE_NORMAL;
    p_seqnum_entry->header.handle = 1;
    p_seqnum_entry->header.len_words = 2;
    p_seqnum_entry->data[0] = 0x2000;
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, NULL, &p_useless_noise[0]);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, &p_useless_noise[0], &p_useless_noise[1]);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, &p_useless_noise[1], p_seqnum_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_seqnum_entry, p_iv_entry);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, p_iv_entry, &p_useless_noise[1]);
    flash_manager_entry_next_get_ExpectAndReturn(mp_manager, NULL, &p_useless_noise[1], NULL);
    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);

    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
    TEST_ASSERT_EQUAL(0x1000, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());
    flash_manager_mock_Verify();
}

void test_reset(void)
{
    /* We have to allocate sequence numbers in flash before we start transmitting */
    expect_flash_load(0x1234, 0x5000, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    uint32_t seqnum;
    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0x5000, seqnum);
    TEST_ASSERT_EQUAL(0x1234, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());


    flash_manager_entry_invalidate_ExpectAndReturn(mp_manager, HANDLE_SEQNUM, NRF_SUCCESS);
    flash_manager_entry_invalidate_ExpectAndReturn(mp_manager, HANDLE_IV_DATA, NRF_SUCCESS);
    net_state_reset();

    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_seqnum_alloc(&seqnum));
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);

    /* try and fail */
    flash_manager_mem_listener_register_StubWithCallback(
        flash_manager_mem_listener_register_callback);
    expect_flash_load(0x1234, 0x5000, false);
    net_state_recover_from_flash();
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    /* fail one */
    flash_manager_entry_invalidate_ExpectAndReturn(mp_manager, HANDLE_SEQNUM, NRF_ERROR_NO_MEM);
    net_state_reset();
    TEST_ASSERT_NOT_NULL(mp_listeners[0]);
    /* retry, fail second */
    flash_manager_entry_invalidate_ExpectAndReturn(mp_manager, HANDLE_SEQNUM, NRF_SUCCESS);
    flash_manager_entry_invalidate_ExpectAndReturn(mp_manager, HANDLE_IV_DATA, NRF_ERROR_NO_MEM);
    fire_mem_listeners();
    TEST_ASSERT_NOT_NULL(mp_listeners[0]);
    /* retry, succeed */
    flash_manager_entry_invalidate_ExpectAndReturn(mp_manager, HANDLE_SEQNUM, NRF_SUCCESS);
    flash_manager_entry_invalidate_ExpectAndReturn(mp_manager, HANDLE_IV_DATA, NRF_SUCCESS);
    fire_mem_listeners();

    TEST_ASSERT_EQUAL(0, net_state_tx_iv_index_get());
    TEST_ASSERT_EQUAL(NET_STATE_IV_UPDATE_NORMAL, net_state_iv_update_get());

    expect_flash_seqnum(NETWORK_SEQNUM_FLASH_BLOCK_SIZE);
    TEST_ASSERT_EQUAL(NRF_ERROR_FORBIDDEN, net_state_seqnum_alloc(&seqnum));
    notify_flash_write_complete(mp_expected_seqnum_flash_buffer);

    TEST_ASSERT_EQUAL(NRF_SUCCESS, net_state_seqnum_alloc(&seqnum));
    TEST_ASSERT_EQUAL(0, seqnum);
}
