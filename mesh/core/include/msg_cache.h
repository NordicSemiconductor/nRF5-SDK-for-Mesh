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
#ifndef MSG_CACHE_H__
#define MSG_CACHE_H__

#include <stdbool.h>
#include <stdint.h>

#include "packet.h"
#include "nrf_mesh.h"
#include "nrf_mesh_config_core.h"

/**
 * @defgroup MSG_CACHE Message cache system
 * @ingroup MESH_CORE
 * Caches incoming packets, and keeps them from being echoed around the network.
 * @{
 */

/**
 * Initialize the message cache module.
 */
void msg_cache_init(void);

/**
 * Check whether the given packet already exists in message cache.
 *
 * @param[in] src_addr        Source address.
 * @param[in] sequence_number Message sequence number (little endian).
 *
 * @return Returns @c true if the specified packet is in the message cache,
 *         or @c false otherwise.
 */
bool msg_cache_entry_exists(uint16_t src_addr, uint32_t sequence_number);

/**
 * Add the given packet to the message cache.
 *
 * @param[in] src Source address.
 * @param[in] seq Message sequence number (little endian).
 */
void msg_cache_entry_add(uint16_t src, uint32_t seq);

/**
 * Clears all entries from the message cache.
 */
void msg_cache_clear(void);

/** @} */

#endif /* MSG_CACHE_H__ */

