/* Copyright (c) 2010 - 2018, Nordic Semiconductor ASA
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
#ifndef CORE_TX_ADV_H__
#define CORE_TX_ADV_H__
#include "core_tx.h"

/**
 * @defgroup CORE_TX_ADV Core TX Advertiser bearer
 * @ingroup CORE_TX
 * Core TX wrapper of the advertiser bearer.
 * @{
 */


/**
 * Initializes the Core TX advertiser bearer, and register it with the Core TX module.
 */
void core_tx_adv_init(void);

/**
 * Sets the number of transmissions to do for a single packet, when transmitted in the given role.
 *
 * @param[in] role Role to configure the TX count for.
 * @param[in] tx_count The number of transmissions to do per packet. Must be larger than 0. Setting
 * this to @ref ADVERTISER_REPEAT_INFINITE causes the packet to be transmitted until a new packet
 * has been added to the queue.
 */
void core_tx_adv_count_set(core_tx_role_t role, uint8_t tx_count);

/**
 * Gets the number of transmissions that will be done per packet when transmitted in the given role.
 *
 * @param[in] role Role to get the configuration for.
 *
 * @returns The configured number of transmissions per packet for the given role.
 */
uint8_t core_tx_adv_count_get(core_tx_role_t role);

/**
 * Sets the advertisement interval for the specific role.
 *
 * @param[in] role Role to set the advertisement interval for.
 * @param[in] interval_ms Advertisement interval in milliseconds.
 */
void core_tx_adv_interval_set(core_tx_role_t role, uint32_t interval_ms);

/**
 * Gets the advertisement interval in milliseconds for the given role.
 *
 * @param[in] role Role to get the advertisement interval for.
 *
 * @returns The advertisement interval for the given role in milliseconds.
 */
uint32_t core_tx_adv_interval_get(core_tx_role_t role);

/**
 * Sets the advertisement address for the specific role.
 *
 * @param[in] role Role to set the advertisement address for.
 * @param[in] p_addr New GAP advertisement address.
 */
void core_tx_adv_address_set(core_tx_role_t role, const ble_gap_addr_t * p_addr);

/** @} */

#endif /* CORE_TX_ADV_H__ */
