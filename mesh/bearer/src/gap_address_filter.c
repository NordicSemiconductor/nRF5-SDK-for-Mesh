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

#include "gap_address_filter.h"
#include "filter_engine.h"
#include "nrf_mesh_assert.h"

/** Types of filters for addresses */
typedef enum
{
    ADDR_FILTER_TYPE_NONE,      /**< No filter set. */
    ADDR_FILTER_TYPE_WHITELIST, /**< Whitelist filter, address must be in list to be accepted. */
    ADDR_FILTER_TYPE_BLACKLIST, /**< Blacklist filter, addresses in the list will be rejected. */
    ADDR_FILTER_TYPE_RANGE      /**< Range filter, address must be in range between two addresses. */
} addr_filter_type_t;

typedef struct
{
    filter_t               filter;
    const ble_gap_addr_t * p_gap_addr_list;
    uint16_t               count;
    addr_filter_type_t     type;
    uint32_t               amount_filtered_gap_addr_frames;
} gap_addr_filter_t;

/** Filter for GAP addresses */
static gap_addr_filter_t m_addr_filter;

static bool gap_address_filter_handle(scanner_packet_t * p_scan_packet, void * p_data)
{
    packet_t * p_packet = &p_scan_packet->packet;
    gap_addr_filter_t * p_filter = (gap_addr_filter_t *)p_data;
    bool filtering = p_filter->type == ADDR_FILTER_TYPE_BLACKLIST ? false : true;

    if (p_filter->type == ADDR_FILTER_TYPE_NONE)
    {
        return false; /* No filter set, accept all */
    }

    if (p_filter->type == ADDR_FILTER_TYPE_WHITELIST || p_filter->type == ADDR_FILTER_TYPE_BLACKLIST)
    {
        for (uint32_t i = 0; i < p_filter->count; ++i)
        {
            if (memcmp(p_filter->p_gap_addr_list[i].addr, p_packet->addr, BLE_GAP_ADDR_LEN) == 0 &&
                p_packet->header.addr_type == p_filter->p_gap_addr_list[i].addr_type)
            {
                filtering = !filtering;
                break;
            }
        }
    }
    else
    {
        if ((p_packet->header.addr_type == p_filter->p_gap_addr_list[0].addr_type) &&
            (memcmp(p_filter->p_gap_addr_list[0].addr, p_packet->addr, BLE_GAP_ADDR_LEN) <= 0) &&
            (memcmp(p_filter->p_gap_addr_list[1].addr, p_packet->addr, BLE_GAP_ADDR_LEN) > 0))
        {
            filtering = false;
        }
    }

    if (filtering)
    {
        p_filter->amount_filtered_gap_addr_frames++;
    }

    return filtering;
}

static uint32_t gap_addr_filter_set(const ble_gap_addr_t * const p_addrs,
                                    uint16_t addr_count,
                                    addr_filter_type_t type)
{
    if (m_addr_filter.type != ADDR_FILTER_TYPE_NONE &&
        m_addr_filter.type != type)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_addrs == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (addr_count == 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    m_addr_filter.type            = type;
    m_addr_filter.count           = addr_count;
    m_addr_filter.filter.handler  = gap_address_filter_handle;
    m_addr_filter.filter.p_data   = (void *)&m_addr_filter;
    m_addr_filter.filter.type     = FILTER_TYPE_POST_PROC;

    if (m_addr_filter.p_gap_addr_list != p_addrs)
    {
        m_addr_filter.p_gap_addr_list = p_addrs;
        fen_filter_start(&m_addr_filter.filter);
    }

    return NRF_SUCCESS;
}

uint32_t bearer_filter_gap_addr_whitelist_set(const ble_gap_addr_t * const p_addrs, uint16_t addr_count)
{
    return gap_addr_filter_set(p_addrs, addr_count, ADDR_FILTER_TYPE_WHITELIST);
}

uint32_t bearer_filter_gap_addr_blacklist_set(const ble_gap_addr_t * const p_addrs, uint16_t addr_count)
{
    return gap_addr_filter_set(p_addrs, addr_count, ADDR_FILTER_TYPE_BLACKLIST);
}

uint32_t bearer_filter_gap_addr_range_set(const ble_gap_addr_t * const p_addrs)
{
    if (p_addrs == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_addrs[0].addr_type != p_addrs[1].addr_type ||
        memcmp(p_addrs[0].addr, p_addrs[1].addr, BLE_GAP_ADDR_LEN) > 0)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    return gap_addr_filter_set(p_addrs, 2, ADDR_FILTER_TYPE_RANGE);
}

uint32_t bearer_filter_gap_addr_clear(void)
{
    if (m_addr_filter.type == ADDR_FILTER_TYPE_NONE)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    m_addr_filter.type            = ADDR_FILTER_TYPE_NONE;
    m_addr_filter.p_gap_addr_list = NULL;
    m_addr_filter.count           = 0;
    fen_filter_stop(&m_addr_filter.filter);

    return NRF_SUCCESS;
}

uint32_t bearer_gap_addr_filtered_amount_get(void)
{
    return m_addr_filter.amount_filtered_gap_addr_frames;
}
