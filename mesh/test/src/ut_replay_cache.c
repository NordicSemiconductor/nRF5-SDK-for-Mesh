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

#include <stdint.h>
#include <string.h>

#include <unity.h>
#include "test_assert.h"

#include <nrf_error.h>

#include "replay_cache.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_events.h"
#include "mesh_opt.h"

#include "mesh_config_entry_mock.h"
#include "mesh_config_mock.h"

#define ADDR_BASE   1000

/* These are the copy of definitions from replay_cache.c. They shall be synchronized. */
/** Replay cache start ID of the item range */
#define MESH_OPT_REPLY_CACHE_RECORD      0x0001
/** SeqZero cache start ID of the item range */
#define MESH_OPT_SEQZERO_CACHE_RECORD    (MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES)

/*****************************************************************************
* Extern stub
*****************************************************************************/
extern const mesh_config_entry_params_t m_replay_cache_params;
extern const mesh_config_entry_params_t m_seqzero_cache_params;

static nrf_mesh_evt_handler_cb_t m_evt_handler;
static uint32_t m_iv_index;
static bool m_is_iv_index_in_progress;
static uint16_t m_fragmented_record_id;

static uint32_t entry_set_cb(mesh_config_entry_id_t id, const void* p_entry, int num_calls)
{
    (void)num_calls;

    TEST_ASSERT_EQUAL(MESH_OPT_REPLAY_CACHE_FILE_ID, id.file);

    if (m_is_iv_index_in_progress)
    {
        m_fragmented_record_id = id.record;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_REPLY_CACHE_RECORD,
                    MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1))
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_replay_cache_params.callbacks.setter(id, p_entry));
        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(id.record, MESH_OPT_SEQZERO_CACHE_RECORD,
                    MESH_OPT_SEQZERO_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1))
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, m_seqzero_cache_params.callbacks.setter(id, p_entry));
        return NRF_SUCCESS;
    }

    TEST_FAIL();
    return NRF_ERROR_INTERNAL;
}

static uint32_t entry_delete_cb(mesh_config_entry_id_t id, int num_calls)
{
    (void)num_calls;

    TEST_ASSERT_TRUE(m_is_iv_index_in_progress);
    TEST_ASSERT_EQUAL(MESH_OPT_REPLAY_CACHE_FILE_ID, id.file);

    if (IS_IN_RANGE(m_fragmented_record_id, MESH_OPT_REPLY_CACHE_RECORD,
                    MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1))
    {
        TEST_ASSERT_TRUE(IS_IN_RANGE(id.record, MESH_OPT_REPLY_CACHE_RECORD,
                         MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1));
        return NRF_SUCCESS;
    }

    if (IS_IN_RANGE(m_fragmented_record_id, MESH_OPT_SEQZERO_CACHE_RECORD,
                    MESH_OPT_SEQZERO_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1))
    {
        TEST_ASSERT_TRUE(IS_IN_RANGE(id.record, MESH_OPT_SEQZERO_CACHE_RECORD,
                         MESH_OPT_SEQZERO_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1));
        return NRF_SUCCESS;
    }

    TEST_FAIL();
    return NRF_ERROR_INTERNAL;
}

static void do_iv_update(uint32_t new_iv_index)
{
    m_is_iv_index_in_progress = true;
    m_iv_index = new_iv_index;
    const nrf_mesh_evt_t evt = {
        .type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION, // don't really care about the params.
    };
    m_evt_handler(&evt);
    m_is_iv_index_in_progress = false;
}

void nrf_mesh_evt_handler_add(nrf_mesh_evt_handler_t * p_evt_handler)
{
    TEST_ASSERT_NOT_NULL(p_evt_handler);
    TEST_ASSERT_NOT_NULL(p_evt_handler->evt_cb);
    m_evt_handler = p_evt_handler->evt_cb;
}

uint32_t net_state_beacon_iv_index_get(void)
{
    return m_iv_index;
}

void setUp(void)
{
    mesh_config_entry_mock_Init();
    mesh_config_mock_Init();

    mesh_config_entry_set_StubWithCallback(entry_set_cb);
    mesh_config_entry_delete_StubWithCallback(entry_delete_cb);

    m_iv_index = 0;
    replay_cache_init();
    TEST_ASSERT_NOT_NULL(m_evt_handler);
    replay_cache_enable();
}

void tearDown(void)
{
    mesh_config_file_clear_Expect(MESH_OPT_REPLAY_CACHE_FILE_ID);
    replay_cache_clear();

    mesh_config_entry_mock_Verify();
    mesh_config_entry_mock_Destroy();
    mesh_config_mock_Verify();
    mesh_config_mock_Destroy();
}

void test_add(void)
{
    for (uint32_t i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE + i, 0, 0));
        TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE + i, 0, 0));
        TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE + i, 1, 0)); //seqnum too high
    }
    /* We've filled the list */
    TEST_ASSERT_EQUAL(NRF_ERROR_NO_MEM, replay_cache_add(ADDR_BASE + REPLAY_CACHE_ENTRIES, 0, 0));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE + REPLAY_CACHE_ENTRIES, 0, 0));

    /* We can safely add the same entries again (with higher seqnums) */
    for (uint32_t i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE + i, 1, 0));
        TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE + i, 0, 0)); // we also have lower seqnums
        TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE + i, 1, 0));
        TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE + i, 2, 0)); //seqnum too high
    }
}

void test_iv_update(void)
{
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE, 1, 0));

    // Update to IV index = 1, should keep the entries around and not change behavior.
    do_iv_update(1);
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 0, 0)); // we also have lower seqnums
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 1, 0));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0)); // same IV index, higher seqnum
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 0, 1)); // higher IV index, lower seqnum
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 1)); // higher IV index, same seqnum
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 1)); // higher IV index, higher seqnum

    // add an entry with current iv index
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE, 1, 1));

    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 0, 0)); // we also have lower iv indexes
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 1, 0)); // we also have lower iv indexes
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 0, 1)); // same IV index, lower seqnum
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 1, 1)); // same IV index, same seqnum
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 1)); // same IV index, higher seqnum

    // fill the replay cache with IV index = 0 messages
    uint16_t addr = ADDR_BASE + 1;
    while (replay_cache_add(addr, 1, 0) == NRF_SUCCESS)
    {
        TEST_ASSERT_TRUE(replay_cache_has_elem(addr, 1, 0));
        addr++;
    }
    TEST_ASSERT_EQUAL(ADDR_BASE + REPLAY_CACHE_ENTRIES, addr);

    /* Update to IV index = 2. Should discard all IV index == 0 entries, as we can't receive on those
     * anymore (IVI bit can only let us receive packets with current IV index or current IV index - 1) */
    do_iv_update(2);
    for (uint32_t i = ADDR_BASE + 1; i <= addr; ++i)
    {
        TEST_ASSERT_FALSE(replay_cache_has_elem(i, 1, 0));
    }

    // We're still keeping the IV index = 1 entry, as that's still eligible for RX:
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 1, 1));

    // fill the replay cache with IV index = 2 messages
    addr = ADDR_BASE + 1;
    while (replay_cache_add(addr, 1, 2) == NRF_SUCCESS)
    {
        TEST_ASSERT_TRUE(replay_cache_has_elem(addr, 1, 2));
        addr++;
    }
    TEST_ASSERT_EQUAL(ADDR_BASE + REPLAY_CACHE_ENTRIES, addr);

    /* Update to IV index = 10. Should discard all entries, as they're all on old IV indexes */
    do_iv_update(10);
    for (uint32_t i = ADDR_BASE; i <= addr; ++i)
    {
        TEST_ASSERT_FALSE(replay_cache_has_elem(i, 1, 1));
    }

    // Test rollover on short-version of IV index:

    /* Add an entry with IV index 0xffff */
    do_iv_update(0xffff);
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE, 1, 0xffff));

    // roll the IV index over to 0x10000, the short version of this IV index is 0x0000.
    do_iv_update(0x10000);
    // should still know the entry, as it's just 1 less than the current IV index:
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 0, 0xffff));
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 1, 0xffff));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x10000)); // IV index too high
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0xffff)); // seqnum too high

    // Add an entry on the right IV index
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE, 1, 0x10000));

    /* Bump the IV index to 0x20001, the short version of this IV index is 0x0001 (and could therefore
     * have been a valid number), but we should discard all entries, as they're really out of range. */
    do_iv_update(0x20001);
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 0, 0x10000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x10000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x20000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0x20000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x20001));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0x20001));

    // Add an entry on the right IV index
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE, 1, 0x20001));

    /* Bump the IV index to 0x55555, the short version of this IV index is 0x0001 (and could therefore
     * have been a valid number), but we should discard all entries, as they're really out of range. */
    do_iv_update(0x55555);
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 0, 0x10000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x10000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x20000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0x20000));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x20001));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0x20001));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0x55554));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 1, 0x55555));
    TEST_ASSERT_FALSE(replay_cache_has_elem(ADDR_BASE, 2, 0x55555));
}

void test_clear(void)
{
    uint16_t addr = ADDR_BASE;
    while (replay_cache_add(addr, 1, 0) == NRF_SUCCESS)
    {
        TEST_ASSERT_TRUE(replay_cache_has_elem(addr, 1, 0));
        addr++;
    }
    TEST_ASSERT_EQUAL(ADDR_BASE + REPLAY_CACHE_ENTRIES, addr);

    mesh_config_file_clear_Expect(MESH_OPT_REPLAY_CACHE_FILE_ID);
    replay_cache_clear();
    for (uint32_t i = ADDR_BASE; i <= addr; ++i)
    {
        TEST_ASSERT_FALSE(replay_cache_has_elem(i, 1, 1));
    }

    // should be space for new entries again:
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(addr, 1, 0));
}

void test_adding_old_entry(void)
{
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE, 10, 0));
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 10, 0));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(ADDR_BASE, 9, 0));
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 10, 0));
    TEST_ASSERT_TRUE(replay_cache_has_elem(ADDR_BASE, 9, 0));
}

void test_seqauth_init(void)
{
    // Check that no seqauth in the replay cache yet
    for (uint32_t src = 0; src <= 0xFFFF; src++)
    {
        TEST_ASSERT_FALSE(replay_cache_has_seqauth(src, 0, 0, 0));
    }
}

void test_seqauth_seq_less_than_seqzero(void)
{
    TEST_NRF_MESH_ASSERT_EXPECT(replay_cache_seqauth_add(ADDR_BASE, 10, 0, 12));
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(ADDR_BASE, 10, 0, 12));
    TEST_ASSERT_FALSE(replay_cache_is_seqauth_last(ADDR_BASE, 10, 0, 12));
}

void test_seqauth_adding_less_than_exists(void)
{
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_seqauth_add(ADDR_BASE, 10, 0, 10));
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_seqauth_add(ADDR_BASE, 5, 0, 5));
    TEST_ASSERT_TRUE(replay_cache_has_seqauth(ADDR_BASE, 10, 0, 10));
}

void test_last_seqauth_in_cache(void)
{
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_seqauth_add(ADDR_BASE, 10, 0, 10));
    TEST_ASSERT_TRUE(replay_cache_is_seqauth_last(ADDR_BASE, 10, 0, 10));
    TEST_ASSERT_FALSE(replay_cache_is_seqauth_last(ADDR_BASE, 9, 0, 9));
    TEST_ASSERT_FALSE(replay_cache_is_seqauth_last(ADDR_BASE, 11, 0, 11));
}

void test_seqauth_cache(void)
{
    struct {
        uint32_t iv_index;
        uint32_t seqnum;
        uint16_t seqzero;
        uint32_t seqnum_first;
        uint32_t seqnum_last;
    } seqauth_variants[] = {
            {0, 0, 0, 0, 8191},  // SeqAuth = 0
            {0, 10, 10, 10, 8201},  // SeqAuth = 10
            {0, 8191, 8191, 8191, 8191 + 8191},  // SeqAuth = 8191
            {0, 0x801FFF, 0x1FFF, 0x801FFF, 0x801FFF + 8191},  // SeqAuth = 0x801FFF
            {1, 0x801FFF, 0x1FFF, 0x801FFF, 0x801FFF + 8191},  // SeqAuth = 0x1801FFF
    };

    for (size_t i = 0; i < ARRAY_SIZE(seqauth_variants); i++)
    {
        do_iv_update(seqauth_variants[i].iv_index);

        TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_seqauth_add(ADDR_BASE,
                                                                seqauth_variants[i].seqnum,
                                                                seqauth_variants[i].iv_index,
                                                                seqauth_variants[i].seqzero));

        for (size_t segment_seqnum = seqauth_variants[i].seqnum_first;
             segment_seqnum <= seqauth_variants[i].seqnum_last;
             segment_seqnum++)
        {
            TEST_ASSERT_TRUE(replay_cache_has_seqauth(ADDR_BASE,
                                                      segment_seqnum,
                                                      seqauth_variants[i].iv_index,
                                                      seqauth_variants[i].seqzero));
        }

        // The next seqnum after the last one should correspond to a new SeqAuth
        TEST_ASSERT_FALSE(replay_cache_has_seqauth(ADDR_BASE,
                                                   seqauth_variants[i].seqnum_last + 1,
                                                   seqauth_variants[i].iv_index,
                                                   seqauth_variants[i].seqzero));

        // The next seqzero should correspond to a new SeqAuth
        TEST_ASSERT_FALSE(replay_cache_has_seqauth(ADDR_BASE,
                                                   seqauth_variants[i].seqnum_last + 1,
                                                   seqauth_variants[i].iv_index,
                                                   (seqauth_variants[i].seqzero + 1) & 0x1FFF));

        if (seqauth_variants[i].seqnum_first > 0
            && seqauth_variants[i].seqzero > 0)
        {
            // The previous seqzero should correspond to an old SeqAuth
            TEST_ASSERT_TRUE(replay_cache_has_seqauth(ADDR_BASE,
                                                      seqauth_variants[i].seqnum_first - 1,
                                                      seqauth_variants[i].iv_index,
                                                      seqauth_variants[i].seqzero - 1));
        }
    }
}

