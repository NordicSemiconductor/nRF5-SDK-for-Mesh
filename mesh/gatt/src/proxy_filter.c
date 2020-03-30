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
#include "proxy_filter.h"

#include <stddef.h>
#include "nrf_mesh_assert.h"

/**
 * Returns whether the given filter has the given address, ignoring the filter type.
 *
 * @param[in] p_filter Filter to look through.
 * @param[in] addr Address to look for.
 *
 * @returns Whether the address is present in the filter.
 */
static bool proxy_filter_has_addr(const proxy_filter_t * p_filter, uint16_t addr)
{
    for (uint32_t i = 0; i < p_filter->count; ++i)
    {
        if (p_filter->addrs[i] == addr)
        {
            return true;
        }
    }
    return false;
}

void proxy_filter_clear(proxy_filter_t * p_filter)
{
    NRF_MESH_ASSERT(p_filter);
    p_filter->type  = PROXY_FILTER_TYPE_WHITELIST;
    p_filter->count = 0;
}

uint32_t proxy_filter_type_set(proxy_filter_t * p_filter, proxy_filter_type_t type)
{
    NRF_MESH_ASSERT(p_filter);

    if (type >= PROXY_FILTER_TYPE_RFU_START)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    proxy_filter_clear(p_filter);
    p_filter->type = type;
    return NRF_SUCCESS;
}

void proxy_filter_add(proxy_filter_t * p_filter, const uint16_t * p_addrs, uint32_t addr_count)
{
    NRF_MESH_ASSERT(p_filter);
    NRF_MESH_ASSERT(p_addrs);
    for (uint32_t i = 0;
         (i < addr_count && p_filter->count < MESH_GATT_PROXY_FILTER_ADDR_COUNT);
         ++i)
    {
        if (p_addrs[i] != NRF_MESH_ADDR_UNASSIGNED && !proxy_filter_has_addr(p_filter, p_addrs[i]))
        {
            p_filter->addrs[p_filter->count++] = p_addrs[i];
        }
    }
}

void proxy_filter_remove(proxy_filter_t * p_filter, const uint16_t * p_addrs, uint32_t addr_count)
{
    NRF_MESH_ASSERT(p_filter);
    NRF_MESH_ASSERT(p_addrs);
    for (uint32_t i = 0; i < addr_count; ++i)
    {
        for (uint32_t j = 0; j < p_filter->count; ++j)
        {
            if (p_addrs[i] == p_filter->addrs[j])
            {
                /* Replace the address we will remove with the last one, and reduce count by 1,
                 * changing the order, but maintaining the set. */
                p_filter->addrs[j] =
                    p_filter->addrs[--p_filter->count];
                break;
            }
        }
    }
}

bool proxy_filter_accept(const proxy_filter_t * p_filter, uint16_t addr)
{
    NRF_MESH_ASSERT(p_filter);

    bool has_addr = proxy_filter_has_addr(p_filter, addr);

    if (p_filter->type == PROXY_FILTER_TYPE_WHITELIST)
    {
        return has_addr;
    }
    else
    {
        return !has_addr;
    }
}
