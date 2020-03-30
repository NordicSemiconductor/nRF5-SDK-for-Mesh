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
#include <stdbool.h>

#include "msg_cache.h"
#include "transport.h"
#include "nrf_error.h"

#include "log.h"

/*****************************************************************************
* Local type definitions
*****************************************************************************/
/** Cache entry for message cache. */
typedef struct
{
    bool allocated;  /**< Whether the entry is in use. */
    uint16_t src;    /**< Source address from packet header. */
    uint32_t seq;    /**< Sequence number from the packet header. */
} msg_cache_entry_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
/** Message cache buffer */
static msg_cache_entry_t m_msg_cache[MSG_CACHE_ENTRY_COUNT];

/** Message cache head index */
static uint32_t m_msg_cache_head = 0;

/*****************************************************************************
* Interface functions
*****************************************************************************/
void msg_cache_init(void)
{
    for (uint32_t i = 0; i < MSG_CACHE_ENTRY_COUNT; ++i)
    {
        m_msg_cache[i].src = NRF_MESH_ADDR_UNASSIGNED;
        m_msg_cache[i].seq = 0;
        m_msg_cache[i].allocated = 0;
    }

    m_msg_cache_head = 0;
}

bool msg_cache_entry_exists(uint16_t src_addr, uint32_t sequence_number)
{
    /* Search backwards from head */
    uint32_t entry_index = m_msg_cache_head;
    for (uint32_t i = 0; i < MSG_CACHE_ENTRY_COUNT; ++i)
    {
        if (entry_index-- == 0) /* compare before subtraction */
        {
            entry_index = MSG_CACHE_ENTRY_COUNT - 1;
        }

        if (!m_msg_cache[entry_index].allocated)
        {
            return false; /* Gone past the last valid entry. */
        }

        if (m_msg_cache[entry_index].src == src_addr &&
            m_msg_cache[entry_index].seq == sequence_number)
        {
            return true;
        }
    }

    return false;
}

void msg_cache_entry_add(uint16_t src, uint32_t seq)
{
    m_msg_cache[m_msg_cache_head].src = src;
    m_msg_cache[m_msg_cache_head].seq = seq;
    m_msg_cache[m_msg_cache_head].allocated = true;

    if ((++m_msg_cache_head) == MSG_CACHE_ENTRY_COUNT)
    {
        m_msg_cache_head = 0;
    }
}

void msg_cache_clear(void)
{
    for (uint32_t i = 0; i < MSG_CACHE_ENTRY_COUNT; ++i)
    {
        m_msg_cache[i].allocated = 0;
    }
}

