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

#ifndef ADV_PACKET_FILTER_H__
#define ADV_PACKET_FILTER_H__

#include <stdint.h>
#include "packet.h"

/**
 * @defgroup ADV_TYPE_FILTER Advertisement packet type filtering
 * @ingroup MESH_API_GROUP_BEARER_FILTER
 * Filtering of advertisement packets based on their advertisement type.
 * @{
 */

/** Possible working modes for BLE advertisement packet type filter.
 *  @ref ADV_FILTER_WHITELIST_MODE is default mode
 */
typedef enum
{
    ADV_FILTER_WHITELIST_MODE, /**< Whitelist mode. */
    ADV_FILTER_BLACKLIST_MODE  /**< Blacklist mode. */
} adv_packet_filter_mode_t;

/**
 * Enable or disable the advertisement type filtering. If enabled, only the
 * packets with the advertisement types added via @ref bearer_adv_packet_add()
 * will be kept and processed by the bearer.
 *
 * @note The main BT Mesh packets (Nonconnectable advertisement type) are not filtered.
 *
 * @param[in] onoff advertisement type filtering state; set to true to enable
 *            the advertisement type filter, and false to disable
 */
void bearer_adv_packet_filtering_set(bool onoff);

/**
 * Remove the advertisement type from the list of accepted advertisement types.
 *
 * @param[in] type The advertisement type that will be removed from the filter.
 *
 * @note This function has no effect if the advertisement type has not
 * been added.
 */
void bearer_adv_packet_remove(ble_packet_type_t type);

/**
 * Add the advertisement type to the list of accepted advertisement types.
 *
 * @param[in] type The advertisement type that will be added to the filter.
 *
 * @note the filtering must be enabled (via a call to
 * @ref bearer_adv_packet_filtering_set) for the added advertisement types to
 * have any effect.
 */
void bearer_adv_packet_add(ble_packet_type_t type);

/**
 * Remove all set advertisement types from the list of accepted advertisement types.
 */
void bearer_adv_packet_clear(void);

/**
 * Set mode of the advertisement type filter.
 *
 * @param[in] mode  Mode of the advertisement type filter.
 *
 * @note There are two possible modes.
 * @ref ADV_FILTER_WHITELIST_MODE (default) mode enables receiving packets
 * with advertisement types in the list.
 * @ref ADV_FILTER_BLACKLIST_MODE mode disables receiving packets
 * with advertisement types in the list.
 */
void bearer_adv_packet_filter_mode_set(adv_packet_filter_mode_t mode);

/**
 * Read out amount filtered packets according the BLE advertisement type settings.
 *
 * @return Amount of the filtered packets with unsuitable BLE advertisement type.
 */
uint32_t bearer_adv_packet_filtered_amount_get(void);

/** @} ADV_TYPE_FILTER */

#endif /* ADV_PACKET_FILTER_H__ */
