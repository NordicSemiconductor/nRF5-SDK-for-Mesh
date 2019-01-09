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
#include <string.h>
#include <nrf_error.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_core.h"
#include "replay_cache.h"
#include "utils.h"
#include "net_state.h"
#include "nrf_mesh_events.h"

typedef struct
{
    uint32_t seqnum;
    uint16_t src;

    /**
     * Lowest 16 bits of the iv index. Kept short to prevent spending another 4 bytes for each entry.
     *
     * As the network layer will only receive packets on the current IV index or the current IV index - 1,
     * we can always extrapolate the full IV index from this and the current IV index. Every time the
     * current IV index is updated, we purge old entries, so there's no chance of a double rollover.
     */
    uint16_t iv_index;
} replay_cache_entry_t;

static uint32_t m_current_iv_index;
static replay_cache_entry_t m_replay_cache[REPLAY_CACHE_ENTRIES];
static uint32_t m_entry_count;

/**
 * Reconstruct the IV index from the entry's trimmed value and the current IV index.
 *
 * As it's not possible to receive on entries with a higher IV index than the current, we can assume
 * that the entries with a higher masked IV index have rolled over.
 *
 * @param[in] p_entry Entry to get IV index of.
 *
 * @returns The IV index of @p p_entry.
 */
static inline uint32_t iv_index_get(const replay_cache_entry_t * p_entry)
{
    uint16_t masked_iv_index = (uint16_t)(m_current_iv_index & UINT16_MAX);

    /* We should have purged the entry if it's not one of iv index or iv index - 1 */
    NRF_MESH_ASSERT_DEBUG(masked_iv_index == p_entry->iv_index ||
                          masked_iv_index == (uint16_t)(p_entry->iv_index + 1));

    /* If we the entry's lower 16 bits are higher than our current lower 16 bits, it means that we've rolled over */
    uint32_t upper_bits = ((p_entry->iv_index <= masked_iv_index) ? m_current_iv_index : (m_current_iv_index - 1));

    return (upper_bits & ~UINT16_MAX) + p_entry->iv_index;
}

static void on_iv_update(void)
{
    /* The IV index in the IV_UPDATE_NOTIFICATION represents the IV index we should use when
     * sending. To get the actual network IV index value, we'll need to get it from net_state: */
    uint32_t new_iv_index = net_state_beacon_iv_index_get();

    for (uint32_t i = 0; i < m_entry_count; )
    {
        uint32_t iv_index = iv_index_get(&m_replay_cache[i]);

        /* According to the Mesh Profile Specification v1.0 Section 3.10.5, we can only receive
         * on the current IV index and current IV index - 1. If an entry is older than this,
         * there's no point in keeping it, as we'll never receive a packet that doesn't qualify. */
        if (iv_index != new_iv_index && iv_index != (new_iv_index - 1))
        {
            // copy the last entry to this one and reduce count, wiping this entry while avoiding holes:
            m_replay_cache[i] = m_replay_cache[--m_entry_count];
        }
        else
        {
            // Should only iterate if we're on an already processed entry:
            ++i;
        }
    }
    m_current_iv_index = new_iv_index;
}

static void evt_handler(const nrf_mesh_evt_t * p_evt)
{
    if (p_evt->type == NRF_MESH_EVT_IV_UPDATE_NOTIFICATION)
    {
        on_iv_update();
    }
}

void replay_cache_init(void)
{
    static nrf_mesh_evt_handler_t event_handler = {.evt_cb = evt_handler};
    nrf_mesh_evt_handler_add(&event_handler);

    replay_cache_clear();
}

void replay_cache_enable(void)
{
    m_current_iv_index = net_state_beacon_iv_index_get();
}

uint32_t replay_cache_add(uint16_t src, uint32_t seqnum, uint32_t iv_index)
{
    for (uint_fast8_t i = 0; i < m_entry_count; ++i)
    {
        if (m_replay_cache[i].src == src)
        {
            m_replay_cache[i].iv_index = (uint16_t) iv_index;
            m_replay_cache[i].seqnum = seqnum;
            return NRF_SUCCESS;
        }
    }

    if (m_entry_count < REPLAY_CACHE_ENTRIES)
    {
        m_replay_cache[m_entry_count].src = src;
        m_replay_cache[m_entry_count].iv_index = (uint16_t) iv_index;
        m_replay_cache[m_entry_count].seqnum = seqnum;
        m_entry_count++;
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}


bool replay_cache_has_elem(uint16_t src, uint32_t seqnum, uint32_t iv_index)
{
    for (uint_fast8_t i = 0; i < m_entry_count; ++i)
    {
        if (m_replay_cache[i].src == src)
        {
            uint32_t entry_iv_index = iv_index_get(&m_replay_cache[i]);

            /* According to Mesh Profile Specification v1.0 Section 3.8.8, we should discard packets
             * coming IV indexes lower than a previous packet from the same source as well as
             * packets on the same IV index with lower sequence numbers. */
            return ((iv_index == entry_iv_index && m_replay_cache[i].seqnum >= seqnum) ||
                    (entry_iv_index > iv_index));
        }
    }

    return false;
}

void replay_cache_clear(void)
{
    m_entry_count = 0;
    memset(m_replay_cache, 0, sizeof(m_replay_cache));
}
