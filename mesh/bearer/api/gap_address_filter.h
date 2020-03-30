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

#ifndef GAP_ADDRESS_FILTER_H__
#define GAP_ADDRESS_FILTER_H__

#include "nrf_mesh.h"

/**
 * @defgroup GAP_ADDRESS_FILTER GAP address filtering
 * @ingroup MESH_API_GROUP_BEARER_FILTER
 * Filtering of advertisement packets based on their GAP address.
 * @{
 */

/**
 * Set a whitelist for GAP addresses. Only packets with a GAP address entry in
 * the whitelist will be passed from the radio to the stack for processing.
 *
 * @note The @p p_addrs parameter must point to a statically allocated list of
 * addresses of at least @p addr_count length.
 *
 * @warning Changing the contents of the whitelist while it's in use may result
 * in unwanted packets being accepted. It is recommended to clear the list
 * before changing it.
 *
 * @param[in] p_addrs List of addresses to accept. Must be statically allocated.
 * @param[in] addr_count The number of addresses in the given list.
 *
 * @retval NRF_SUCCESS The whitelist was successfully set.
 * @retval NRF_ERROR_INVALID_STATE A blacklist is already in place. Clear the
 * filter before changing the type of accept criteria.
 * @retval NRF_ERROR_NULL The @p p_addrs variable was NULL.
 * @retval NRF_ERROR_INVALID_LENGTH The @p addr_count variable was 0.
 */
uint32_t bearer_filter_gap_addr_whitelist_set(const ble_gap_addr_t * const p_addrs, uint16_t addr_count);

/**
 * Set a blacklist for GAP addresses. Packets with a GAP address entry in the
 * blacklist will not be passed from the radio to the upper stack for
 * processing.
 *
 * @note The @p p_addrs parameter must point to a statically allocated list of
 * addresses of at least @p addr_count length.
 *
 * @warning Changing the contents of the blacklist while it's in use may result
 * in unwanted packets being accepted. It is recommended to clear the list
 * before changing it.
 *
 * @param[in] p_addrs List of addresses to accept. Must be statically allocated.
 * @param[in] addr_count The number of addresses in the given list.
 *
 * @retval NRF_SUCCESS The blacklist was successfully set.
 * @retval NRF_ERROR_INVALID_STATE A whitelist is already in place. Clear the
 * filter before changing the type of accept criteria.
 * @retval NRF_ERROR_NULL The @p p_addrs variable was NULL.
 * @retval NRF_ERROR_INVALID_LENGTH The @p addr_count variable was 0.
 */
uint32_t bearer_filter_gap_addr_blacklist_set(const ble_gap_addr_t * const p_addrs, uint16_t addr_count);

/**
 * Set a range for GAP addresses. Packets with a GAP address equal to or higher
 * than the first entry and a GAP address lower than the second entry will
 * pass, ie p_addrs[0] <= addr < p_addrs[1].
 *
 * @note The @p p_addrs parameter must point to a statically allocated list of
 * addresses of at least 2 entries. The addresses must have the same address
 * type.
 *
 * @warning Changing the range filter while it's in use may result in unwanted
 * packets being accepted. It is recommended to clear the filter before
 * changing it.
 *
 * @param[in] p_addrs Statically allocated array of two or more addresses, with
 * the same address type, where the first one is lower than or equal to the
 * second.
 *
 * @retval NRF_SUCCESS The blacklist was successfully set.
 * @retval NRF_ERROR_NULL The @p p_addrs variable was NULL.
 * @retval NRF_ERROR_INVALID_DATA The addresses does not have the same address
 * type.
 * @retval NRF_ERROR_INVALID_STATE Another filter is already in place. Clear it
 * before changing the type of accept criteria.
 */
uint32_t bearer_filter_gap_addr_range_set(const ble_gap_addr_t * const p_addrs);

/**
 * Remove the currently assigned GAP address filter.
 *
 * @retval NRF_SUCCESS The current GAP address filter was removed.
 * @retval NRF_ERROR_INVALID_STATE No GAP address filter was in place.
 */
uint32_t bearer_filter_gap_addr_clear(void);

/**
 * Read out amount filtered packets according the advertisement addresses settings.
 *
 * @return Amount of the filtered packets with unsuitable advertisement addresses.
 */
uint32_t bearer_gap_addr_filtered_amount_get(void);

/** @} GAP_ADDRESS_FILTER */

#endif /* GAP_ADDRESS_FILTER_H__ */
