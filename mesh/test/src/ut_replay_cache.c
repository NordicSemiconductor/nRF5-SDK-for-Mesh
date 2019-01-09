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

#include <stdint.h>
#include <string.h>

#include <unity.h>

#include <nrf_error.h>

#include "replay_cache.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_events.h"

#define ADDR_BASE   1000

static nrf_mesh_evt_handler_cb_t m_evt_handler;
static uint32_t m_iv_index;

static void do_iv_update(uint32_t new_iv_index)
{
    m_iv_index = new_iv_index;
    const nrf_mesh_evt_t evt = {
        .type = NRF_MESH_EVT_IV_UPDATE_NOTIFICATION, // don't really care about the params.
    };
    m_evt_handler(&evt);
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
    m_iv_index = 0;
    replay_cache_init();
    TEST_ASSERT_NOT_NULL(m_evt_handler);
    replay_cache_enable();
}

void tearDown(void)
{
    replay_cache_clear();
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

    replay_cache_clear();
    for (uint32_t i = ADDR_BASE; i <= addr; ++i)
    {
        TEST_ASSERT_FALSE(replay_cache_has_elem(i, 1, 1));
    }

    // should be space for new entries again:
    TEST_ASSERT_EQUAL(NRF_SUCCESS, replay_cache_add(addr, 1, 0));
}
