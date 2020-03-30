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
#ifndef PROXY_FILTER_H__
#define PROXY_FILTER_H__

#include <stdint.h>
#include <stdbool.h>
#include "nrf_mesh_config_core.h"

/**
 * @defgroup PROXY_FILTER Proxy filters
 * @ingroup PROXY
 * The proxy filter is a list of short-addresses the proxy server maintains for its proxy client to
 * reduce the required throughput.
 * @{
 */

typedef enum
{
    PROXY_FILTER_TYPE_WHITELIST,
    PROXY_FILTER_TYPE_BLACKLIST,
    PROXY_FILTER_TYPE_RFU_START,
} proxy_filter_type_t;

typedef struct
{
    uint16_t addrs[MESH_GATT_PROXY_FILTER_ADDR_COUNT];
    uint16_t count;
    proxy_filter_type_t type;
} proxy_filter_t;

/**
 * Clear all addresses in the filter, and set the filter type to whitelist.
 *
 * @param[in,out] p_filter Filter to clear.
 */
void proxy_filter_clear(proxy_filter_t * p_filter);

/**
 * Set the filter type, clearing the filter.
 *
 * @param[in,out] p_filter Filter to set type of
 * @param[in] type Filter type to set.
 *
 * @retval NRF_SUCCESS The filter type was set successfully.
 * @retval NRF_ERROR_INVALID_DATA The given filter type was invalid.
 */
uint32_t proxy_filter_type_set(proxy_filter_t * p_filter, proxy_filter_type_t type);

/**
 * Add a list of addresses to the filter.
 *
 * Only valid addresses which aren't already present in the filter will be added.
 *
 * @param[in,out] p_filter Filter to add to.
 * @param[in] p_addrs List of addresses with @c addr_count entries.
 * @param[in] addr_count Number of addresses in @c p_addrs.
 */
void proxy_filter_add(proxy_filter_t * p_filter, const uint16_t * p_addrs, uint32_t addr_count);

/**
 * Remove a list of addresses from the filter.
 *
 * Ignores any addresses that aren't in the filter.
 *
 * @param[in,out] p_filter Filter to remove from.
 * @param[in] p_addrs List of addresses to remove.
 * @param[in] addr_count Number of addresses in @c p_addrs.
 */
void proxy_filter_remove(proxy_filter_t * p_filter, const uint16_t * p_addrs, uint32_t addr_count);

/**
 * Check whether the given filter accepts the address.
 *
 * @param[in] p_filter Filter to use.
 * @param[in] addr Address to check.
 *
 * @returns Whether or not the filter accepts the address.
 */
bool proxy_filter_accept(const proxy_filter_t * p_filter, uint16_t addr);

/** @} */

#endif /* PROXY_FILTER_H__ */
