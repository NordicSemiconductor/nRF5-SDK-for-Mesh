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
#include <string.h>
#include <nrf_error.h>

#include "nrf_mesh_defines.h"
#include "nrf_mesh_config_core.h"
#include "replay_cache.h"

typedef struct
{
    uint32_t seqno : NETWORK_SEQNUM_BITS;
    uint16_t src;
} replay_cache_entry_t;

/**
 * @todo Get memory from elsewhere...
 */
static replay_cache_entry_t m_replay_cache[2][REPLAY_CACHE_ENTRIES];

static uint8_t m_cache_index = 0;
static bool m_cache_full = false;

void replay_cache_init(void)
{
    replay_cache_clear();
}

uint32_t replay_cache_add(uint16_t src, uint32_t seqno, uint8_t ivi)
{
    for (uint_fast8_t i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        if (m_replay_cache[ivi][i].src == 0)
        {
            /* Free slot! */
            m_replay_cache[ivi][i].seqno = seqno;
            m_replay_cache[ivi][i].src   = src;
            if (i == (REPLAY_CACHE_ENTRIES - 1))
            {
                m_cache_full = true;
            }
            return NRF_SUCCESS;
        }

        if (m_replay_cache[ivi][i].src == src)
        {
            /* Free slot! */
            m_replay_cache[ivi][i].seqno = seqno;
            return NRF_SUCCESS;
        }
    }

    return NRF_ERROR_NO_MEM;
}


bool replay_cache_has_room(uint16_t src, uint8_t ivi)
{
    if (!m_cache_full)
    {
        return true;
    }
    for (uint32_t i=0; i<REPLAY_CACHE_ENTRIES; i++)
    {
        if (m_replay_cache[ivi][i].src == src)
        {
            return true;
        }
    }
    return false;
}

bool replay_cache_has_elem(uint16_t src, uint32_t seqno, uint8_t ivi)
{
    for (uint_fast8_t i = 0; i < REPLAY_CACHE_ENTRIES; ++i)
    {
        if (m_replay_cache[ivi][i].src == src)
        {
            if (m_replay_cache[ivi][i].seqno < seqno)
            {
                return false;
            }

            return true;
        }
    }

    /* Not to be added to cache unless successful application decrypt! */
    return false;
}

void replay_cache_on_iv_update(void)
{
    /* Clear old index */
    m_cache_index = (m_cache_index + 1) & 0x01;
    memset(m_replay_cache[m_cache_index], 0, sizeof(m_replay_cache[0]));
}

void replay_cache_clear(void)
{
    memset(m_replay_cache, 0, sizeof(m_replay_cache));
    m_cache_index = 0;
    m_cache_full = false;
}
