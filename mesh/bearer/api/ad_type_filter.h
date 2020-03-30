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

#ifndef AD_TYPE_FILTER_H__
#define AD_TYPE_FILTER_H__

#include <stdint.h>
#include <stdbool.h>

/**
 * @defgroup AD_TYPE_FILTER AD type filtering
 * @ingroup MESH_API_GROUP_BEARER_FILTER
 * Filtering of advertisement packets based on their AD types.
 * @{
 */

/** Possible working modes for AD type filter.
 *  @ref AD_FILTER_WHITELIST_MODE is default mode.
 */
typedef enum
{
    AD_FILTER_WHITELIST_MODE, /**< Whitelist mode. */
    AD_FILTER_BLACKLIST_MODE  /**< Blacklist mode. */
} ad_type_mode_t;

/**
 * Enable or disable AD type filtering. If enabled, only the
 * packets with the AD types added via @ref bearer_adtype_add()
 * will be kept and processed by the bearer.
 *
 * @param[in] onoff AD type filtering state; set to true to enable
 *            AD Type filter, and false to disable
 */
void bearer_adtype_filtering_set(bool onoff);

/**
 * Remove the AD type from the list of accepted AD types.
 *
 * @param[in] type The AD type that will be removed from the filter.
 *
 * @note This function has no effect if the AD type has not
 * been added.
 */
void bearer_adtype_remove(uint8_t type);

/**
 * Add the AD type to the list of accepted AD types.
 *
 * @param[in] type The AD type that will be added to the filter.
 *
 * @note Ad type filtering must be enabled (via a call to
 * @ref bearer_adtype_filtering_set) for the added AD types to
 * have any effect.
 */
void bearer_adtype_add(uint8_t type);

/**
 * Remove all set AD types from the list of accepted AD types.
 */
void bearer_adtype_clear(void);

/**
 * Set mode of AD type filter.
 *
 * @param[in] mode  Mode of AD type filter.
 *
 * @note There are two possible modes.
 * @ref AD_FILTER_WHITELIST_MODE (default) mode enables receiving packets with AD in the list.
 * @ref AD_FILTER_BLACKLIST_MODE mode disables receiving packets with AD in the list.
 */
void bearer_adtype_mode_set(ad_type_mode_t mode);

/**
 * Read out amount filtered packets with invalid length.
 *
 * @return Amount of the filtered packets with an invalid length.
 */
uint32_t bearer_invalid_length_amount_get(void);

/**
 * Read out amount filtered packets according AD type settings.
 *
 * @return Amount of the filtered packets with unsuitable AD type.
 */
uint32_t bearer_adtype_filtered_amount_get(void);

/** @} AD_TYPE_FILTER */

#endif /* AD_TYPE_FILTER_H__ */
