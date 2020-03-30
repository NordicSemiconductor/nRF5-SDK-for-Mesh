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

#include "ad_type_filter.h"
#include "filter_engine.h"
#include "packet.h"
#include "nrf_mesh_assert.h"
#include "bitfield.h"

/* AD type is 8 bits , so we need 256 bits to encode all possible values as a bit field,
   we will store them as 32 bit numbers (words), thus we need 256/32 words */
#define FILTER_SIZE BITFIELD_BLOCK_COUNT(256)

typedef struct
{
    filter_t       filter;
    ad_type_mode_t mode;
    uint32_t       adtype_filter[FILTER_SIZE];
    uint32_t       amount_invalid_length_frames;
    uint32_t       amount_filtered_adtype_frames;
} ad_type_filter_t;

static ad_type_filter_t m_adtype_filter;

static bool adtype_filter_handle(scanner_packet_t * p_scan_packet, void * p_data)
{
    (void)p_data;
    packet_t * p_packet = &p_scan_packet->packet;
    bool filtering = m_adtype_filter.mode == AD_FILTER_WHITELIST_MODE ? true : false;

    for (int i = 0; i < packet_payload_size_get(p_packet);)
    {
        uint8_t length = p_packet->payload[i];
        uint8_t type = p_packet->payload[i + 1];

        if (bitfield_get(m_adtype_filter.adtype_filter, type))
        {
            if (length >= packet_payload_size_get(p_packet) - i)
            {
                m_adtype_filter.amount_invalid_length_frames++;
                return true;
            }
            filtering = !filtering;
            break;
        }

        i += length + 1;
    }

    if (filtering)
    {
        m_adtype_filter.amount_filtered_adtype_frames++;
    }

    return filtering;
}

void bearer_adtype_filtering_set(bool onoff)
{
    if (onoff)
    {
        m_adtype_filter.filter.handler = adtype_filter_handle;
        m_adtype_filter.filter.type    = FILTER_TYPE_PRE_PROC;
        fen_filter_start(&m_adtype_filter.filter);
    }
    else
    {
        fen_filter_stop(&m_adtype_filter.filter);
    }
}

void bearer_adtype_add(uint8_t type)
{
    bitfield_set(m_adtype_filter.adtype_filter, type);
}

void bearer_adtype_remove(uint8_t type)
{
    bitfield_clear(m_adtype_filter.adtype_filter, type);
}

void bearer_adtype_clear(void)
{
    bitfield_clear_all(m_adtype_filter.adtype_filter, 256);
}

void bearer_adtype_mode_set(ad_type_mode_t mode)
{
    NRF_MESH_ASSERT(mode == AD_FILTER_WHITELIST_MODE || mode == AD_FILTER_BLACKLIST_MODE);

    m_adtype_filter.mode = mode;
}

uint32_t bearer_invalid_length_amount_get(void)
{
    return m_adtype_filter.amount_invalid_length_frames;
}

uint32_t bearer_adtype_filtered_amount_get(void)
{
    return m_adtype_filter.amount_filtered_adtype_frames;
}
