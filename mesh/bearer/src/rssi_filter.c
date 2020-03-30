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

#include "rssi_filter.h"
#include "filter_engine.h"
#include "scanner.h"

typedef struct
{
    filter_t filter;
    uint32_t packets_dropped_invalid_rssi;
    int8_t   rssi_filter_val;
    uint32_t amount_filtered_rssi_frames;
} rssi_filter_t;

static rssi_filter_t m_rssi_filter;

static bool rssi_filter_handle(scanner_packet_t * p_scan_packet, void * p_data)
{
    (void)p_data;

    if (p_scan_packet->metadata.rssi < m_rssi_filter.rssi_filter_val)
    {
        m_rssi_filter.amount_filtered_rssi_frames++;
        return true;
    }

    return false;
}

void bearer_rssi_filtering_set(int8_t rssi)
{
    if (rssi < 0)
    {
        if (m_rssi_filter.rssi_filter_val == 0)
        {
            m_rssi_filter.filter.handler = rssi_filter_handle;
            m_rssi_filter.filter.type    = FILTER_TYPE_POST_PROC;
            fen_filter_start(&m_rssi_filter.filter);
        }

        m_rssi_filter.rssi_filter_val = rssi;
    }
    else
    {
        m_rssi_filter.rssi_filter_val = 0;
        fen_filter_stop(&m_rssi_filter.filter);
    }
}

uint32_t bearer_rssi_filtered_amount_get(void)
{
    return m_rssi_filter.amount_filtered_rssi_frames;
}
