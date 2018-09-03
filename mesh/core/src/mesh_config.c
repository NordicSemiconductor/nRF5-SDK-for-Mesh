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
#include <stddef.h>
#include <string.h>
#include "mesh_config.h"
#include "mesh_config_entry.h"
#include "mesh_config_backend.h"
#include "mesh_config_listener.h"
#include "utils.h"
#include "event.h"

#include "nrf_section.h"
#include "nrf_error.h"

#define CONFIG_ENTRY_COUNT    (NRF_MESH_SECTION_ITEM_COUNT(mesh_config_entries, const mesh_config_entry_params_t))
#define CONFIG_FILE_COUNT     (NRF_MESH_SECTION_ITEM_COUNT(mesh_config_files, const mesh_config_file_params_t))

#define FOR_EACH_FILE(P_FILE)                                                                      \
    NRF_MESH_SECTION_FOR_EACH(mesh_config_files, const mesh_config_file_params_t, P_FILE)
#define FOR_EACH_ENTRY(P_ENTRY)                                                                    \
    NRF_MESH_SECTION_FOR_EACH(mesh_config_entries, const mesh_config_entry_params_t, P_ENTRY)
#define FOR_EACH_LISTENER(P_LISTENER)                                                              \
    NRF_MESH_SECTION_FOR_EACH(mesh_config_entry_listeners, const mesh_config_listener_t, P_LISTENER)

NRF_MESH_SECTION_DEF_FLASH(mesh_config_files, const mesh_config_file_params_t);
NRF_MESH_SECTION_DEF_FLASH(mesh_config_entries, const mesh_config_entry_params_t);
NRF_MESH_SECTION_DEF_FLASH(mesh_config_entry_listeners, const mesh_config_listener_t);

#if PERSISTENT_STORAGE
static const mesh_config_entry_params_t * entry_params_get(uint32_t i)
{
    return NRF_MESH_SECTION_ITEM_GET(mesh_config_entries, const mesh_config_entry_params_t, i);
}

static const mesh_config_file_params_t * file_params_get(uint32_t i)
{
    return NRF_MESH_SECTION_ITEM_GET(mesh_config_files, const mesh_config_file_params_t, i);
}
#endif

static mesh_config_entry_flags_t * entry_flags_get(const mesh_config_entry_params_t * p_params, mesh_config_entry_id_t id)
{
    uint32_t index = id.record - p_params->p_id->record;
    NRF_MESH_ASSERT(index < p_params->max_count);
    return &p_params->p_state[index];
}

static bool contains_entry(const mesh_config_entry_params_t * p_params, mesh_config_entry_id_t id)
{
    return (id.file == p_params->p_id->file &&
            IS_IN_RANGE(id.record, p_params->p_id->record, p_params->p_id->record + p_params->max_count - 1));
}

static const mesh_config_entry_params_t * entry_params_find(mesh_config_entry_id_t id)
{
    FOR_EACH_ENTRY(p_params)
    {
        if (contains_entry(p_params, id))
        {
            return p_params;
        }
    }
    return NULL;
}

static const mesh_config_file_params_t * file_params_find(uint16_t id)
{
    FOR_EACH_FILE(p_file)
    {
        if (id == p_file->id)
        {
            return p_file;
        }
    }
    return NULL;
}

static void listeners_notify(const mesh_config_entry_params_t * p_params,
                             mesh_config_change_reason_t reason,
                             mesh_config_entry_id_t id,
                             const void * p_entry)
{
    FOR_EACH_LISTENER(p_listener)
    {
        if (contains_entry(p_params, *p_listener->p_id))
        {
            p_listener->callback(reason, id, p_entry);
        }
    }
}

static void dirty_entries_process(mesh_config_strategy_t strategy)
{
#if PERSISTENT_STORAGE
    FOR_EACH_ENTRY(p_params)
    {
        const mesh_config_file_params_t * p_file    = file_params_find(p_params->p_id->file);
        NRF_MESH_ASSERT(p_file);
        if (p_file->strategy == strategy)
        {
            for (uint32_t j = 0; j < p_params->max_count; ++j)
            {
                if ((p_params->p_state[j] & MESH_CONFIG_ENTRY_FLAG_DIRTY) &&
                   !(p_params->p_state[j] & MESH_CONFIG_ENTRY_FLAG_BUSY))
                {
                    mesh_config_entry_id_t id = *p_params->p_id;
                    id.record += j;
                    uint32_t status;

                    if (p_params->p_state[j] & MESH_CONFIG_ENTRY_FLAG_ACTIVE)
                    {
                        /* The backend has to make a copy, as the buffer is on stack! */
                        uint8_t buf[MESH_CONFIG_ENTRY_MAX_SIZE] __attribute__((aligned(WORD_SIZE)));

                        p_params->callbacks.getter(id, buf);

                        status = mesh_config_backend_store(id, buf, p_params->entry_size);
                    }
                    else
                    {
                        status = mesh_config_backend_erase(id);
                    }

                    if (status == NRF_SUCCESS)
                    {
                        p_params->p_state[j] &= (mesh_config_entry_flags_t)~MESH_CONFIG_ENTRY_FLAG_DIRTY;
                        p_params->p_state[j] |= MESH_CONFIG_ENTRY_FLAG_BUSY;
                    }
                    else
                    {
                        /* Back off if the backend call fails, to allow it to free up some resources */
                        return;
                    }
                }
            }
        }
    }
#endif
}

static uint32_t entry_store(const mesh_config_entry_params_t * p_params, mesh_config_entry_id_t id, const void * p_entry)
{
    uint32_t status = p_params->callbacks.setter(id, p_entry);
    if (status == NRF_SUCCESS)
    {
        mesh_config_entry_flags_t * p_flags = entry_flags_get(p_params, id);
        *p_flags |= (mesh_config_entry_flags_t)(MESH_CONFIG_ENTRY_FLAG_DIRTY | MESH_CONFIG_ENTRY_FLAG_ACTIVE);
        const mesh_config_file_params_t * p_file = file_params_find(p_params->p_id->file);
        NRF_MESH_ASSERT(p_file != NULL);
        if (p_file->strategy == MESH_CONFIG_STRATEGY_CONTINUOUS)
        {
            dirty_entries_process(MESH_CONFIG_STRATEGY_CONTINUOUS);
        }

        listeners_notify(p_params, MESH_CONFIG_CHANGE_REASON_SET, id, p_entry);
    }
    else
    {
        status = NRF_ERROR_INVALID_DATA;
    }
    return status;
}

static mesh_config_backend_iterate_action_t restore_callback(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len)
{
    const mesh_config_entry_params_t * p_params = entry_params_find(id);
    mesh_config_load_failure_t load_failure;

    if (p_params == NULL)
    {
        load_failure = MESH_CONFIG_LOAD_FAILURE_INVALID_ID;
    }
    else if (entry_len < p_params->entry_size)
    {
        load_failure = MESH_CONFIG_LOAD_FAILURE_INVALID_LENGTH;
    }
    else if (p_params->callbacks.setter(id, p_entry) != NRF_SUCCESS)
    {
        load_failure = MESH_CONFIG_LOAD_FAILURE_INVALID_DATA;
    }
    else
    {
        *entry_flags_get(p_params, id) = MESH_CONFIG_ENTRY_FLAG_ACTIVE;
        /* Success causes early return */
        return MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE;
    }

    /* No early return, notify user about load failure */
    const nrf_mesh_evt_t load_failure_event = {
        .type = NRF_MESH_EVT_CONFIG_LOAD_FAILURE,
        .params.config_load_failure = {
            .p_data = p_entry,
            .id = id, /*lint !e64 Type mismatch */
            .data_len = entry_len,
            .reason = load_failure
        }
    };
    event_handle(&load_failure_event);

    return MESH_CONFIG_BACKEND_ITERATE_ACTION_CONTINUE;
}

#if PERSISTENT_STORAGE
static void backend_evt_handler(const mesh_config_backend_evt_t * p_evt)
{
    const mesh_config_entry_params_t * p_params = entry_params_find(p_evt->id);
    NRF_MESH_ASSERT(p_params);

    mesh_config_entry_flags_t * p_flags = entry_flags_get(p_params, p_evt->id);
    NRF_MESH_ASSERT_DEBUG(*p_flags & MESH_CONFIG_ENTRY_FLAG_BUSY);
    *p_flags &= (mesh_config_entry_flags_t)~MESH_CONFIG_ENTRY_FLAG_BUSY;

    if (p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE)
    {
        const nrf_mesh_evt_t evt = {
            .type = NRF_MESH_EVT_CONFIG_STORAGE_FAILURE,
            .params.config_storage_failure = {
                .id = p_evt->id, /*lint !e64 Type mismatch */
            }
        };
        event_handle(&evt);
    }

    if (mesh_config_is_busy())
    {
        dirty_entries_process(MESH_CONFIG_STRATEGY_CONTINUOUS);
    }
    else
    {
        nrf_mesh_evt_t evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
        event_handle(&evt);
    }
}
#endif

/**
 * Test whether any entries have invalid IDs.
 */
static void entry_validation(void)
{
#ifndef NDEBUG
    FOR_EACH_ENTRY(p_params_1)
    {
        NRF_MESH_ASSERT((uint32_t) p_params_1->p_id->record + p_params_1->max_count <= UINT16_MAX);

        FOR_EACH_ENTRY(p_params_2)
        {
            if (p_params_1 == p_params_2)
            {
                break;
            }
            if (p_params_1->p_id->file == p_params_2->p_id->file)
            {
                NRF_MESH_ASSERT((p_params_1->p_id->record >= p_params_2->p_id->record + p_params_2->max_count) ||
                                (p_params_2->p_id->record >= p_params_1->p_id->record + p_params_1->max_count));
            }
        }
    }
#endif
}

void mesh_config_init(void)
{
    entry_validation();
#if PERSISTENT_STORAGE
    mesh_config_backend_init(entry_params_get(0), CONFIG_ENTRY_COUNT, file_params_get(0), CONFIG_FILE_COUNT, backend_evt_handler);
#endif
}

void mesh_config_load(void)
{
    mesh_config_backend_read_all(restore_callback);
}

void mesh_config_power_down(void)
{
    dirty_entries_process(MESH_CONFIG_STRATEGY_ON_POWER_DOWN);
}

bool mesh_config_is_busy(void)
{
#if PERSISTENT_STORAGE
    FOR_EACH_ENTRY(p_params)
    {
        for (uint32_t j = 0; j < p_params->max_count; ++j)
        {
            if (p_params->p_state[j] &
               (mesh_config_entry_flags_t)(MESH_CONFIG_ENTRY_FLAG_DIRTY | MESH_CONFIG_ENTRY_FLAG_BUSY))
            {
                return true;
            }
        }
    }
#endif
    return false;
}

bool mesh_config_entry_available_id(mesh_config_entry_id_t * p_base_id)
{
    NRF_MESH_ASSERT(p_base_id);
    const mesh_config_entry_params_t * p_params = entry_params_find(*p_base_id);

    for (uint32_t i = 0; i < p_params->max_count; ++i)
    {
        if (!(p_params->p_state[i] & MESH_CONFIG_ENTRY_FLAG_ACTIVE))
        {
            p_base_id->record = p_params->p_id->record + i;
            return true;
        }
    }
    return false;
}

uint32_t mesh_config_entry_set(mesh_config_entry_id_t id, const void * p_entry)
{
    if (p_entry == NULL)
    {
        return NRF_ERROR_NULL;
    }

    const mesh_config_entry_params_t * p_params = entry_params_find(id);
    if (p_params)
    {
        return entry_store(p_params, id, p_entry);
    }
    else
    {
        return NRF_ERROR_NOT_FOUND;
    }
}

uint32_t mesh_config_entry_get(mesh_config_entry_id_t id, void * p_entry)
{
    if (p_entry == NULL)
    {
        return NRF_ERROR_NULL;
    }

    const mesh_config_entry_params_t * p_params = entry_params_find(id);
    if (p_params)
    {
        if (*entry_flags_get(p_params, id) & MESH_CONFIG_ENTRY_FLAG_ACTIVE)
        {
            p_params->callbacks.getter(id, p_entry);
        }
        else
        {
            if (p_params->p_default)
            {
                memcpy(p_entry, p_params->p_default, p_params->entry_size);
            }
            else
            {
                return NRF_ERROR_INVALID_STATE;
            }
        }
        return NRF_SUCCESS;
    }
    return NRF_ERROR_NOT_FOUND;
}

uint32_t mesh_config_entry_delete(mesh_config_entry_id_t id)
{
    const mesh_config_entry_params_t * p_params = entry_params_find(id);
    if (p_params)
    {
        mesh_config_entry_flags_t * p_flags = entry_flags_get(p_params, id);
        if (*p_flags & MESH_CONFIG_ENTRY_FLAG_ACTIVE)
        {
            *p_flags &= (mesh_config_entry_flags_t)~MESH_CONFIG_ENTRY_FLAG_ACTIVE; /* no longer active */
            *p_flags |= MESH_CONFIG_ENTRY_FLAG_DIRTY;

            if (p_params->callbacks.deleter)
            {
                p_params->callbacks.deleter(id);
            }

            if (file_params_find(p_params->p_id->file)->strategy == MESH_CONFIG_STRATEGY_CONTINUOUS)
            {
                dirty_entries_process(MESH_CONFIG_STRATEGY_CONTINUOUS);
            }
            listeners_notify(p_params, MESH_CONFIG_CHANGE_REASON_DELETE, id, NULL);
            return NRF_SUCCESS;
        }
        else
        {
            return NRF_ERROR_INVALID_STATE;
        }
    }
    return NRF_ERROR_NOT_FOUND;
}

uint32_t mesh_config_power_down_time_get(void)
{
    return mesh_config_backend_power_down_time_get();
}
