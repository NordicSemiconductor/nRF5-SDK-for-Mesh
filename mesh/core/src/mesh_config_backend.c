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

#include "mesh_config_backend_glue.h"
#include "mesh_config_backend.h"
#include "nrf_mesh_assert.h"

static const mesh_config_file_params_t * mp_files;
static uint32_t m_file_count;
static uint32_t m_power_down_time_us;

static void dummy_event(const mesh_config_backend_evt_t * p_evt)
{
    (void)p_evt;
}

static const mesh_config_file_params_t * file_get(uint16_t file_id)
{
    for (uint32_t cnt = 0; cnt < m_file_count; cnt++)
    {
        if (mp_files[cnt].id == file_id)
        {
            mp_files[cnt].p_backend_data->file_id = mp_files[cnt].id;
            return &mp_files[cnt];
        }
    }

    return NULL;
}

/**
 * Gets the next file, sorted by file ID.
 *
 * @param[in] p_params Previous file, or NULL to find the first file.
 *
 * @returns The next file, according to its file ID.
 */
static const mesh_config_file_params_t * next_file_get(const mesh_config_file_params_t * p_params)
{
    uint16_t start_id = ((p_params == NULL) ? 0 : p_params->id + 1);
    p_params = NULL;

    for (uint32_t cnt = 0; cnt < m_file_count; cnt++)
    {
        if (mp_files[cnt].id >= start_id && (p_params == NULL || mp_files[cnt].id < p_params->id))
        {
            p_params = &mp_files[cnt];
        }
    }
    return p_params;
}

void mesh_config_backend_init(const mesh_config_entry_params_t * p_entries,
                              uint32_t entry_count,
                              const mesh_config_file_params_t * p_files,
                              uint32_t file_count,
                              mesh_config_backend_evt_cb_t evt_cb)
{
    NRF_MESH_ASSERT(p_entries != NULL);
    NRF_MESH_ASSERT(entry_count > 0ul);
    NRF_MESH_ASSERT(p_files != NULL);
    NRF_MESH_ASSERT(file_count > 0ul);

    m_power_down_time_us = 0;

    mp_files = p_files;
    m_file_count = file_count;
    mesh_config_backend_glue_init(evt_cb == NULL ? dummy_event : evt_cb);

    for (uint32_t itr = 0; itr < entry_count; itr++)
    {
        const mesh_config_file_params_t * p_file = file_get(p_entries[itr].p_id->file);
        NRF_MESH_ASSERT(p_file != NULL && p_file->p_backend_data != NULL);
        uint32_t size_guard = p_file->p_backend_data->size +
                mesh_config_record_size_calculate(p_entries[itr].entry_size) * p_entries[itr].max_count;
        NRF_MESH_ASSERT(size_guard <= UINT16_MAX);
        p_file->p_backend_data->size = size_guard;
        p_file->p_backend_data->entry_count += p_entries[itr].max_count;
    }

    /* Create files in increasing file ID order */
    const mesh_config_file_params_t * p_file = NULL;

    while ((p_file = next_file_get(p_file)) != NULL)
    {
        if (MESH_CONFIG_STRATEGY_NON_PERSISTENT != p_file->strategy)
        {
            NRF_MESH_ERROR_CHECK(mesh_config_backend_file_create(p_file->p_backend_data));
        }

        m_power_down_time_us += mesh_config_backend_file_power_down_time_get(p_file);
    }
}

uint32_t mesh_config_backend_store(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len)
{
    NRF_MESH_ASSERT(p_entry != NULL);
    NRF_MESH_ASSERT(entry_len > 0);
    const mesh_config_file_params_t * p_file = file_get(id.file);
    NRF_MESH_ASSERT(p_file != NULL);
    NRF_MESH_ASSERT(p_file->p_backend_data->size > 0);

    p_file->p_backend_data->curr_pos = id.record;

    return mesh_config_backend_record_write(p_file->p_backend_data, p_entry, entry_len);
}

uint32_t mesh_config_backend_erase(mesh_config_entry_id_t id)
{
    const mesh_config_file_params_t * p_file = file_get(id.file);
    NRF_MESH_ASSERT(p_file != NULL);
    NRF_MESH_ASSERT(p_file->p_backend_data->size > 0);

    p_file->p_backend_data->curr_pos = id.record;

    return mesh_config_backend_record_erase(p_file->p_backend_data);
}

uint32_t mesh_config_backend_read(mesh_config_entry_id_t id, uint8_t * p_entry, uint32_t * p_entry_len)
{
    NRF_MESH_ASSERT(p_entry != NULL);
    NRF_MESH_ASSERT(p_entry_len != NULL);
    NRF_MESH_ASSERT(*p_entry_len > 0ul);
    const mesh_config_file_params_t * p_file = file_get(id.file);
    NRF_MESH_ASSERT(p_file != NULL);
    NRF_MESH_ASSERT(p_file->p_backend_data->size > 0);

    p_file->p_backend_data->curr_pos = id.record;

    return mesh_config_backend_record_read(p_file->p_backend_data, p_entry, p_entry_len);
}

void mesh_config_backend_read_all(mesh_config_backend_iterate_cb_t cb)
{
    NRF_MESH_ASSERT(cb != NULL);

    for (uint32_t itr = 0; itr < m_file_count; itr++)
    {
        if (MESH_CONFIG_STRATEGY_NON_PERSISTENT != mp_files[itr].strategy)
        {
            mesh_config_backend_records_read(mp_files[itr].p_backend_data, cb);
        }
    }
}

uint32_t mesh_config_backend_power_down_time_get(void)
{
    return m_power_down_time_us;
}
