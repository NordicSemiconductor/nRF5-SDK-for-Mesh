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
#include <string.h>
#include <nrf_error.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_core.h"
#include "replay_cache.h"
#include "utils.h"
#include "net_state.h"
#include "nrf_mesh_events.h"
#include "transport_internal.h"
#include "mesh_opt.h"
#include "mesh_config_entry.h"
#include "mesh_config.h"

/** Definition for the invalid SeqZero cache entry. */
#define SEQZERO_CACHE_ENTRY_INVALID 0xFFFF

/** Replay cache start ID of the item range */
#define MESH_OPT_REPLY_CACHE_RECORD      0x0001
/** SeqZero cache start ID of the item range */
#define MESH_OPT_SEQZERO_CACHE_RECORD    (MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES)
/** Replay cache entry ID */
#define MESH_OPT_REPLAY_CACHE_EID   MESH_CONFIG_ENTRY_ID(MESH_OPT_REPLAY_CACHE_FILE_ID, MESH_OPT_REPLY_CACHE_RECORD)
/** SeqZero cache entry ID  */
#define MESH_OPT_SEQZERO_CACHE_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_REPLAY_CACHE_FILE_ID, MESH_OPT_SEQZERO_CACHE_RECORD)

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
/** Cache for SeqZero values where each index corresponds to the m_replay_cache index. */
static uint16_t m_seqzero_cache[REPLAY_CACHE_ENTRIES];
static bool m_is_enabled;

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

        /* According to @tagMeshSp section 3.10.5, we can only receive
         * on the current IV index and current IV index - 1. If an entry is older than this,
         * there's no point in keeping it, as we'll never receive a packet that doesn't qualify. */
        if (iv_index != new_iv_index && iv_index != (new_iv_index - 1))
        {
            // copy the last entry to this one and reduce count, wiping this entry while avoiding holes:
            m_entry_count--;

            mesh_config_entry_id_t id = MESH_OPT_REPLAY_CACHE_EID;
            id.record += i;
            NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &m_replay_cache[m_entry_count]));
            id.record = m_entry_count + MESH_OPT_REPLY_CACHE_RECORD;
            NRF_MESH_ERROR_CHECK(mesh_config_entry_delete(id));

            id = MESH_OPT_SEQZERO_CACHE_EID;
            id.record += i;
            NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &m_seqzero_cache[m_entry_count]));
            id.record = m_entry_count + MESH_OPT_SEQZERO_CACHE_RECORD;
            NRF_MESH_ERROR_CHECK(mesh_config_entry_delete(id));
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

static inline bool packet_is_new(const replay_cache_entry_t * p_entry, uint32_t new_iv_index, uint32_t new_seqnum)
{
    uint32_t entry_iv_index = iv_index_get(p_entry);

    /* According to @tagMeshSp section 3.8.8, we should discard packets
     * coming from IV indexes lower than a previous packet from the same source, as well as
     * packets on the same IV index with lower sequence numbers. */
    return ((new_iv_index == entry_iv_index && new_seqnum > p_entry->seqnum) ||
            (new_iv_index > entry_iv_index));
}

static inline bool seqauth_is_new(uint32_t entry_index, uint32_t new_iv_index, uint32_t new_seqnum, uint16_t new_seqzero)
{
    uint64_t entry_seqauth = transport_sar_seqauth_get(iv_index_get(&m_replay_cache[entry_index]),
                                                       m_replay_cache[entry_index].seqnum,
                                                       m_seqzero_cache[entry_index]);
    uint64_t new_seqauth = transport_sar_seqauth_get(new_iv_index, new_seqnum, new_seqzero);

    return new_seqauth > entry_seqauth;
}

static uint32_t entry_add(uint16_t src, uint32_t seqnum, uint32_t iv_index, uint32_t *p_index)
{
    mesh_config_entry_id_t id;

    for (uint_fast8_t i = 0; i < m_entry_count; ++i)
    {
        if (m_replay_cache[i].src == src)
        {
            if (packet_is_new(&m_replay_cache[i], iv_index, seqnum))
            {
                m_replay_cache[i].iv_index = (uint16_t) iv_index;
                m_replay_cache[i].seqnum = seqnum;
                id = MESH_OPT_REPLAY_CACHE_EID;
                id.record += i;
                NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &m_replay_cache[i]));
                /* Do not modify SeqZero. */
            }

            if (p_index != NULL)
            {
                *p_index = i;
            }
            return NRF_SUCCESS;
        }
    }

    if (m_entry_count < REPLAY_CACHE_ENTRIES)
    {
        m_replay_cache[m_entry_count].src = src;
        m_replay_cache[m_entry_count].iv_index = (uint16_t) iv_index;
        m_replay_cache[m_entry_count].seqnum = seqnum;
        id = MESH_OPT_REPLAY_CACHE_EID;
        id.record += m_entry_count;
        NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &m_replay_cache[m_entry_count]));

        /* Reset SeqZero cache entry since the address is new. */
        m_seqzero_cache[m_entry_count] = SEQZERO_CACHE_ENTRY_INVALID;
        id = MESH_OPT_SEQZERO_CACHE_EID;
        id.record += m_entry_count;
        NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &m_seqzero_cache[m_entry_count]));

        if (p_index != NULL)
        {
            *p_index = m_entry_count;
        }
        m_entry_count++;
        return NRF_SUCCESS;
    }

    return NRF_ERROR_NO_MEM;
}

static uint32_t replay_cache_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT(IS_IN_RANGE(entry_id.record, MESH_OPT_REPLY_CACHE_RECORD,
                                      MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1));

    uint16_t idx = entry_id.record - MESH_OPT_REPLY_CACHE_RECORD;
    memcpy(&m_replay_cache[idx], p_entry, sizeof(replay_cache_entry_t));

    if (!m_is_enabled)
    {
        m_entry_count++;
    }

    return NRF_SUCCESS;
}

static void replay_cache_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(entry_id.record, MESH_OPT_REPLY_CACHE_RECORD,
                                      MESH_OPT_REPLY_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1));

    uint16_t idx = entry_id.record - MESH_OPT_REPLY_CACHE_RECORD;
    memcpy(p_entry, &m_replay_cache[idx], sizeof(replay_cache_entry_t));
}

static uint32_t seqzero_cache_setter(mesh_config_entry_id_t entry_id, const void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(entry_id.record, MESH_OPT_SEQZERO_CACHE_RECORD,
                                      MESH_OPT_SEQZERO_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1));

    uint16_t idx = entry_id.record - MESH_OPT_SEQZERO_CACHE_RECORD;
    memcpy(&m_seqzero_cache[idx], p_entry, sizeof(uint16_t));

    return NRF_SUCCESS;
}

static void seqzero_cache_getter(mesh_config_entry_id_t entry_id, void * p_entry)
{
    NRF_MESH_ASSERT_DEBUG(IS_IN_RANGE(entry_id.record, MESH_OPT_SEQZERO_CACHE_RECORD,
                                      MESH_OPT_SEQZERO_CACHE_RECORD + REPLAY_CACHE_ENTRIES - 1));

    uint16_t idx = entry_id.record - MESH_OPT_SEQZERO_CACHE_RECORD;
    memcpy(p_entry, &m_seqzero_cache[idx], sizeof(uint16_t));
}

MESH_CONFIG_ENTRY(replay_cache,
                  MESH_OPT_REPLAY_CACHE_EID,
                  REPLAY_CACHE_ENTRIES,
                  sizeof(replay_cache_entry_t),
                  replay_cache_setter,
                  replay_cache_getter,
                  NULL,
                  false);

MESH_CONFIG_ENTRY(seqzero_cache,
                  MESH_OPT_SEQZERO_CACHE_EID,
                  REPLAY_CACHE_ENTRIES,
                  sizeof(uint16_t),
                  seqzero_cache_setter,
                  seqzero_cache_getter,
                  NULL,
                  false);

MESH_CONFIG_FILE(m_replay_cache_file, MESH_OPT_REPLAY_CACHE_FILE_ID, REPLAY_CACHE_STORAGE_STRATEGY);

void replay_cache_init(void)
{ /* Initialization happens before data loading from the flash. */
    static nrf_mesh_evt_handler_t event_handler = {.evt_cb = evt_handler};
    nrf_mesh_evt_handler_add(&event_handler);

    m_is_enabled = false;
    m_entry_count = 0;
    memset(m_replay_cache, 0, sizeof(m_replay_cache));
    memset(m_seqzero_cache, 0, sizeof(m_seqzero_cache));
}

void replay_cache_enable(void)
{
    m_is_enabled = true;
    m_current_iv_index = net_state_beacon_iv_index_get();
}

uint32_t replay_cache_add(uint16_t src, uint32_t seqno, uint32_t iv_index)
{
    return entry_add(src, seqno, iv_index, NULL);
}

uint32_t replay_cache_seqauth_add(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero)
{
    uint32_t status;
    uint32_t entry_index;

    /* There is the assertion in this function because this situation should be handled by transport
     * using checkers replay_cache_has_seqauth and replay_cache_is_seqauth_last.
     * The frame should be silently skipped. If the situation happens here then there is something wrong with transport.  */
    NRF_MESH_ASSERT(seqno >= (uint32_t) seqzero);

    status = entry_add(src, seqno, iv_index, &entry_index);
    if (status == NRF_SUCCESS
        && (m_seqzero_cache[entry_index] == SEQZERO_CACHE_ENTRY_INVALID
            || seqauth_is_new(entry_index, iv_index, seqno, seqzero)))
    {
        mesh_config_entry_id_t id = MESH_OPT_SEQZERO_CACHE_EID;
        id.record += entry_index;
        NRF_MESH_ERROR_CHECK(mesh_config_entry_set(id, &seqzero));
    }

    return status;
}

bool replay_cache_has_elem(uint16_t src, uint32_t seqno, uint32_t iv_index)
{
    for (uint_fast8_t i = 0; i < m_entry_count; ++i)
    {
        if (m_replay_cache[i].src == src)
        {
            return !packet_is_new(&m_replay_cache[i], iv_index, seqno);
        }
    }

    return false;
}

bool replay_cache_has_seqauth(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero)
{
    if (seqno < (uint32_t) seqzero)
    { /* Fault case, we need to drop data. Probably replay attack.
         It returns true to go into the branch where
         replay_cache_is_seqauth_last will filter out the message (return false).
         Finally, transport will skip frame silently. (todo hidden intercomponent dependency should be refactored.) */
        return true;
    }

    for (uint_fast8_t i = 0; i < m_entry_count; ++i)
    {
        if (m_replay_cache[i].src == src)
        {
            return m_seqzero_cache[i] != SEQZERO_CACHE_ENTRY_INVALID
                   && !seqauth_is_new(i, iv_index, seqno, seqzero);
        }
    }

    return false;
}

bool replay_cache_is_seqauth_last(uint16_t src, uint32_t seqno, uint32_t iv_index, uint16_t seqzero)
{
    if (seqno < (uint32_t) seqzero)
    { /* Fault case, we need to drop data. Probably replay attack. */
        return false;
    }

    for (uint_fast8_t i = 0; i < m_entry_count; ++i)
    {
        if (m_replay_cache[i].src == src)
        {
            uint64_t entry_seqauth = transport_sar_seqauth_get(iv_index_get(&m_replay_cache[i]),
                                                               m_replay_cache[i].seqnum,
                                                               m_seqzero_cache[i]);
            uint64_t new_seqauth = transport_sar_seqauth_get(iv_index, seqno, seqzero);

            return m_seqzero_cache[i] != SEQZERO_CACHE_ENTRY_INVALID
                   && new_seqauth == entry_seqauth;
        }
    }

    return false;
}

void replay_cache_clear(void)
{
    m_entry_count = 0;
    memset(m_replay_cache, 0, sizeof(m_replay_cache));
    memset(m_seqzero_cache, 0, sizeof(m_seqzero_cache));
    mesh_config_file_clear(MESH_OPT_REPLAY_CACHE_FILE_ID);
}
