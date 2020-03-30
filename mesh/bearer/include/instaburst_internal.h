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
#ifndef INSTABURST_INTERNAL_H__
#define INSTABURST_INTERNAL_H__

#include "nrf_mesh.h"

/**
 * @defgroup INSTABURST_INTERNAL Instaburst internal functions
 * Common functions for internal Instaburst functionality. All functions defined in instaburst.h
 * @ingroup INSTABURST
 * @{
 */

/**
 * Switch for enabling the Instaburst AID cache. If enabled, the Instaburst RX module will ignore
 * packets with ADIs it has already processed.
 */
#define INSTABURST_CACHE_ENABLED 0

/**
 * Checks Instaburst event id cache.
 *
 * Will add the given id to the cache, and return whether it was present before this call.
 *
 * @param[in] p_id ID to check for.
 *
 * @returns Whether this id was present prior to adding it.
 */
bool instaburst_event_id_cache_has_value(const nrf_mesh_instaburst_event_id_t * p_id);

/**
 * Adds a new event ID to the cache.
 *
 * @param[in] p_id ID to add.
 */
void instaburst_event_id_cache_put(const nrf_mesh_instaburst_event_id_t * p_id);

/** @} */

#endif /* INSTABURST_INTERNAL_H__ */
