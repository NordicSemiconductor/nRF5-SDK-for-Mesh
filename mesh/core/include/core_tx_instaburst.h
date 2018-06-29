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
#ifndef CORE_TX_INSTABURST_H__
#define CORE_TX_INSTABURST_H__
#include "core_tx.h"
#include "radio_config.h"

/**
 * @defgroup CORE_TX_INSTABURST Core TX Instaburst bearer
 * @ingroup INSTABURST
 * Core TX wrapper of the Instaburst bearer.
 * @{
 */

/**
 * Initializes the Core TX Instaburst bearer, and register it with the Core TX module.
 *
 * @warning Should only be called if the Instaburst feature is enabled, and the Core TX advertiser
 * bearer is inactive.
 */
void core_tx_instaburst_init(void);

/**
 * Sets the advertisement interval for the specific role.
 *
 * @param[in] role Role to set the advertisement interval for.
 * @param[in] interval_ms Advertisement interval in milliseconds.
 */
void core_tx_instaburst_interval_set(core_tx_role_t role, uint32_t interval_ms);

/**
 * Gets the advertisement interval in milliseconds for the given role.
 *
 * @param[in] role Role to get the advertisement interval for.
 *
 * @returns The advertisement interval for the given role in milliseconds.
 */
uint32_t core_tx_instaburst_interval_get(core_tx_role_t role);

/**
 * Sets the TX power for the specific role.
 *
 * @param[in] role Role to set the TX power for.
 * @param[in] tx_power TX power.
 */
void core_tx_instaburst_tx_power_set(core_tx_role_t role, radio_tx_power_t tx_power);

/** @} */

#endif /* CORE_TX_INSTABURST_H__ */
