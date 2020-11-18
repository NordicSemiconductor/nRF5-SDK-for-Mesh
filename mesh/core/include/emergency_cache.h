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
#ifndef EMERGENCY_CACHE_H_
#define EMERGENCY_CACHE_H_

#include <stdint.h>
#include "mesh_config_entry.h"

/** The emergency cache item. */
typedef struct
{
    mesh_config_entry_id_t id;   /** The cached entry ID. */
    uint8_t body[];              /** The cached entry body. */
} emergency_cache_item_t;

/**
 * Initialize the emergency cache functionality.
 *
 * @note This is a part of the mesh config.
 */
void emergency_cache_item_init(void);

/**
 * Store an entry value within the emergency cache file.
 *
 * @note Emergency cache item is created automatically.
 *
 * @param[in] p_params   The pointer on entry parameters.
 * @param[in] id         The identifier of the stored entry.
 *
 * @retval NRF_SUCCESS The emergency cache item was successfully created and passed to the flash manager.
 * @retval NRF_ERROR_NO_MEM No memory in the flash manager buffer.
 */
uint32_t emergency_cache_item_store(const mesh_config_entry_params_t * p_params,
                                    mesh_config_entry_id_t id);

/**
 * Get an actual entry value from the item of the emergency cache file.
 *
 * @note the mesh config should restore the emergency cache item from the flash already as a regular entry.
 *
 * @param[in] p_ec_item   The pointer on the raw emergency cache item.
 *
 * @retval The pointer on the parsed emergency cache data.
 */
static inline emergency_cache_item_t * emergency_cache_item_get(const uint8_t * p_ec_item)
{
    emergency_cache_item_t * p_item = (emergency_cache_item_t *)p_ec_item;
    return p_item;
}

/**
 * Get an actual length of the restored entry from the emergency cache item.
 *
 * @param[in] ec_item_length   The emergency cache item length.
 *
 * @retval The actual entry length.
 */
static inline uint32_t emergency_cache_restored_item_length_get(uint32_t ec_item_length)
{
    return ec_item_length - sizeof(emergency_cache_item_t);
}

#endif /* EMERGENCY_CACHE_H_ */
