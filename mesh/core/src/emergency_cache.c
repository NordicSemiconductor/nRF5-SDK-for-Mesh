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

#include "emergency_cache.h"

#include "mesh_opt.h"
#include "mesh_config_backend.h"
#include "nrf_mesh_config_core.h"
#include "nrf_mesh_assert.h"

enum
{
    MESH_OPT_EC_FAKE_RECORD = 0x0001,
};

#define EMERGENCY_CACHE_ITEM_MAX_SIZE  ALIGN_VAL((sizeof(fm_header_t) + sizeof(emergency_cache_item_t) + MESH_CONFIG_ENTRY_MAX_SIZE), WORD_SIZE)
#define EMERGENCY_CACHE_FAKE_ITEM_NUMBER ((EMERGENCY_CACHE_RESERVED_PAGE_NUMBER * FLASH_MANAGER_DATA_PER_PAGE) / EMERGENCY_CACHE_ITEM_MAX_SIZE)
#define MESH_OPT_EC_FAKE_EID  MESH_CONFIG_ENTRY_ID(MESH_OPT_EMERGENCY_CACHE_FILE_ID, MESH_OPT_EC_FAKE_RECORD)

NRF_MESH_STATIC_ASSERT(EMERGENCY_CACHE_ITEM_MAX_SIZE <= FLASH_MANAGER_ENTRY_MAX_SIZE);

static uint32_t fake_item_setter(mesh_config_entry_id_t id, const void * p_entry);
static void fake_item_getter(mesh_config_entry_id_t id, void * p_entry);

/* Fake items descriptor is mandatory to calculate right allocated area in mesh_config backend. */
MESH_CONFIG_ENTRY(fake_items_pool,
                  MESH_OPT_EC_FAKE_EID,
                  EMERGENCY_CACHE_FAKE_ITEM_NUMBER,
                  MESH_CONFIG_ENTRY_MAX_SIZE,
                  fake_item_setter,
                  fake_item_getter,
                  NULL,
                  NULL);

MESH_CONFIG_FILE(m_emergency_cache_file, MESH_OPT_EMERGENCY_CACHE_FILE_ID, MESH_CONFIG_STRATEGY_ON_POWER_DOWN);

static uint16_t m_fake_handle = MESH_OPT_EC_FAKE_RECORD;

static uint32_t fake_item_setter(mesh_config_entry_id_t id, const void * p_entry)
{
    return NRF_SUCCESS;
}

static void fake_item_getter(mesh_config_entry_id_t id, void * p_entry)
{

}

uint32_t emergency_cache_item_store(const mesh_config_entry_params_t * p_params, mesh_config_entry_id_t id)
{
    /* The emergency cache has to create its own item. */
    uint8_t buffer[sizeof(emergency_cache_item_t) + MESH_CONFIG_ENTRY_MAX_SIZE] __attribute__((aligned(WORD_SIZE)));
    emergency_cache_item_t * p_item = (emergency_cache_item_t *)buffer;
    mesh_config_entry_id_t ec_item_id =
    {
        .file = MESH_OPT_EMERGENCY_CACHE_FILE_ID,
        .record = m_fake_handle
    };

    m_fake_handle++;

    p_item->id = id;
    p_params->callbacks.getter(p_item->id, p_item->body);

    return mesh_config_backend_store(ec_item_id, buffer, sizeof(emergency_cache_item_t) + p_params->entry_size);
}

void emergency_cache_item_init(void)
{
    m_fake_handle = MESH_OPT_EC_FAKE_RECORD;
}
