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

#include "adv_packet_filter.h"
#include "filter_engine.h"
#include "packet.h"
#include "nrf_mesh_assert.h"

typedef struct
{
    filter_t                  filter;
    uint16_t                  adv_filter;
    adv_packet_filter_mode_t  mode;
    uint32_t                  amount_filtered_adv_type_frames;
} adv_packet_filter_t;

static adv_packet_filter_t m_adv_packet_filter;

static bool adv_filter_handle(scanner_packet_t * p_scan_packet, void * p_data)
{
    (void)p_data;
    packet_t * p_packet = &p_scan_packet->packet;
    bool filtering = m_adv_packet_filter.mode == ADV_FILTER_WHITELIST_MODE ? true : false;

    if (p_packet->header.type == BLE_PACKET_TYPE_ADV_NONCONN_IND)
    {
        return false; /* the main BT Mesh packets are not filtered */
    }

    if (m_adv_packet_filter.adv_filter & ((uint16_t)1u << p_packet->header.type))
    {
        filtering = !filtering;
    }

    if (filtering)
    {
        m_adv_packet_filter.amount_filtered_adv_type_frames++;
    }

    return filtering;
}

void bearer_adv_packet_filtering_set(bool onoff)
{
    if (onoff)
    {
        m_adv_packet_filter.filter.handler = adv_filter_handle;
        m_adv_packet_filter.filter.type = FILTER_TYPE_POST_PROC;
        fen_filter_start(&m_adv_packet_filter.filter);
    }
    else
    {
        fen_filter_stop(&m_adv_packet_filter.filter);
    }
}

void bearer_adv_packet_remove(ble_packet_type_t type)
{
    m_adv_packet_filter.adv_filter &= ~(uint16_t)(1u << type);
}

void bearer_adv_packet_add(ble_packet_type_t type)
{
    m_adv_packet_filter.adv_filter |= ((uint16_t)1u << type);
}

void bearer_adv_packet_clear(void)
{
    m_adv_packet_filter.adv_filter = 0;
}

void bearer_adv_packet_filter_mode_set(adv_packet_filter_mode_t mode)
{
    NRF_MESH_ASSERT(mode == ADV_FILTER_WHITELIST_MODE ||
                    mode == ADV_FILTER_BLACKLIST_MODE);

    m_adv_packet_filter.mode = mode;
}

uint32_t bearer_adv_packet_filtered_amount_get(void)
{
    return m_adv_packet_filter.amount_filtered_adv_type_frames;
}
