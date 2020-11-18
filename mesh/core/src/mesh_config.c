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
#include <stddef.h>
#include <string.h>
#include "mesh_config.h"
#include "mesh_config_entry.h"
#include "mesh_config_backend.h"
#include "mesh_config_listener.h"
#include "mesh_opt.h"
#if PERSISTENT_STORAGE == 0
#include "bearer_event.h"
#endif
#include "utils.h"
#include "event.h"
#include "emergency_cache.h"

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

/* This flag is set only if mesh_config shall fulfill handling all kind of entries in the dirty_entries_process.
 * There is only one emergency action: Power Down. */
static bool m_is_emergency_action;
static bool m_is_emergency_cache_exist;
/* Counter of entities that are in progress with hw part. */
static uint32_t m_entry_in_progress_cnt;
static uint32_t m_file_in_progress_cnt;

#if PERSISTENT_STORAGE == 0
static bearer_event_flag_t m_bearer_event_flag;
#endif

/* This is architectural hook because dsm entries with the same id can have differed size.
 * Otherwise, these entries will be interpreted as invalid length entries.
 * We have to have this hook to be backward compatible with stack version 3.2.0 and lower. */
extern void dsm_legacy_pretreatment_do(mesh_config_entry_id_t * p_id, uint32_t entry_len);

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

#if PERSISTENT_STORAGE
static uint32_t default_file_store(const mesh_config_entry_params_t * p_params,
                                   mesh_config_entry_id_t id)
{
    /* The backend has to make a copy, as the buffer is on stack! */
    uint8_t buf[MESH_CONFIG_ENTRY_MAX_SIZE] __attribute__((aligned(WORD_SIZE)));

    p_params->callbacks.getter(id, buf);

    return mesh_config_backend_store(id, buf, p_params->entry_size);
}
#endif

static void dirty_entries_process(void)
{
#if PERSISTENT_STORAGE
    if (m_file_in_progress_cnt != 0)
    { /* the file metadata might not be ready till the current moment. */
        return;
    }

    FOR_EACH_ENTRY(p_params)
    {
        const mesh_config_file_params_t * p_file = file_params_find(p_params->p_id->file);
        NRF_MESH_ASSERT(p_file);
        if (p_file->strategy == MESH_CONFIG_STRATEGY_CONTINUOUS ||
           (p_file->strategy == MESH_CONFIG_STRATEGY_ON_POWER_DOWN && m_is_emergency_action))
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
                        /* Files with strategy MESH_CONFIG_STRATEGY_ON_POWER_DOWN have the prepared flash area in advance.
                         * They should be stored in a default manner. */
                        if (p_file->strategy == MESH_CONFIG_STRATEGY_CONTINUOUS && m_is_emergency_action)
                        {
                            status = emergency_cache_item_store(p_params, id);
                        }
                        else
                        {
                            status = default_file_store(p_params, id);
                        }
                    }
                    else
                    {
                        status = mesh_config_backend_erase(id);
                    }

                    switch (status)
                    {
                        case NRF_SUCCESS:
                            m_entry_in_progress_cnt++;
                            p_params->p_state[j] &= (mesh_config_entry_flags_t)~MESH_CONFIG_ENTRY_FLAG_DIRTY;
                            p_params->p_state[j] |= MESH_CONFIG_ENTRY_FLAG_BUSY;
                            break;
                        case NRF_ERROR_NOT_FOUND:
                            /* This can only happen with mesh_config_backend_erase() on not written yet entry. */
                            p_params->p_state[j] &= (mesh_config_entry_flags_t)~MESH_CONFIG_ENTRY_FLAG_DIRTY;
                            break;
                        default:
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
        dirty_entries_process();
#if PERSISTENT_STORAGE == 0
        bearer_event_flag_set(m_bearer_event_flag);
#endif
        listeners_notify(p_params, MESH_CONFIG_CHANGE_REASON_SET, id, p_entry);
    }

    return status;
}

static mesh_config_backend_iterate_action_t restore_callback(mesh_config_entry_id_t id, const uint8_t * p_entry, uint32_t entry_len)
{
    bool is_restored_from_ec = false;

    if (id.file == MESH_OPT_EMERGENCY_CACHE_FILE_ID)
    { /* restore real entry from the emergency cache item. */
        emergency_cache_item_t * p_ec_item = emergency_cache_item_get(p_entry);
        id = p_ec_item->id;
        p_entry = p_ec_item->body;
        entry_len = emergency_cache_restored_item_length_get(entry_len);
        is_restored_from_ec = true;
        m_is_emergency_cache_exist = true;
    }
    else
    { /* Version with legacy entries didn't support emergency cache functionality. */
        dsm_legacy_pretreatment_do(&id, entry_len);
    }

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
        *entry_flags_get(p_params, id) = is_restored_from_ec ?
                (mesh_config_entry_flags_t)(MESH_CONFIG_ENTRY_FLAG_ACTIVE | MESH_CONFIG_ENTRY_FLAG_DIRTY) :
                MESH_CONFIG_ENTRY_FLAG_ACTIVE;
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
static void backend_entry_evt_handler(const mesh_config_backend_evt_t * p_evt)
{
    mesh_config_entry_id_t id;

    /* Restore actual entry id in case of the emergency cache entry. */
    if (p_evt->id.file == MESH_OPT_EMERGENCY_CACHE_FILE_ID)
    { /* restore real entry from emergency cache item. */
        NRF_MESH_ASSERT(p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_STORE_COMPLETE);
        emergency_cache_item_t * p_ec_item = emergency_cache_item_get(p_evt->p_data);
        id = p_ec_item->id;
    }
    else
    {
        id = p_evt->id;
    }

    const mesh_config_entry_params_t * p_params = entry_params_find(id);
    NRF_MESH_ASSERT(p_params);

    mesh_config_entry_flags_t * p_flags = entry_flags_get(p_params, id);
    NRF_MESH_ASSERT_DEBUG(*p_flags & MESH_CONFIG_ENTRY_FLAG_BUSY);
    *p_flags &= (mesh_config_entry_flags_t)~MESH_CONFIG_ENTRY_FLAG_BUSY;
    NRF_MESH_ASSERT(m_entry_in_progress_cnt != 0);
    m_entry_in_progress_cnt--;

    if (p_evt->type != MESH_CONFIG_BACKEND_EVT_TYPE_STORAGE_MEDIUM_FAILURE)
    {
        return;
    }

    if (m_is_emergency_action && p_evt->id.file != MESH_OPT_EMERGENCY_CACHE_FILE_ID)
    {
        const mesh_config_file_params_t * p_file = file_params_find(p_evt->id.file);
        if (p_file->strategy == MESH_CONFIG_STRATEGY_CONTINUOUS &&
            (*p_flags & MESH_CONFIG_ENTRY_FLAG_ACTIVE))
        { /* This is the failed writing of the entry because
           * the action was put in the flash manager queue before the power down happened.
           * Defragmentation has been frozen and there is a lack of place for the entry.
           * Try one more time in the emergency cache. */
            *p_flags |= (mesh_config_entry_flags_t)MESH_CONFIG_ENTRY_FLAG_DIRTY;
            return;
        }
    }

    const nrf_mesh_evt_t evt =
    {
        .type = NRF_MESH_EVT_CONFIG_STORAGE_FAILURE,
        .params.config_storage_failure =
        {
            .id = id, /*lint !e64 Type mismatch */
        }
    };
    event_handle(&evt);
}

static void backend_file_evt_handler(const mesh_config_backend_evt_t * p_evt)
{
    (void)p_evt;
    NRF_MESH_ASSERT(m_file_in_progress_cnt != 0);
    m_file_in_progress_cnt--;
 }

static void backend_evt_handler(const mesh_config_backend_evt_t * p_evt)
{
    if (p_evt->type == MESH_CONFIG_BACKEND_EVT_TYPE_FILE_CLEAN_COMPLETE)
    {
        backend_file_evt_handler(p_evt);
    }
    else
    {
        backend_entry_evt_handler(p_evt);
    }

    if (m_file_in_progress_cnt != 0)
    { /* fulfill the file logic first. */
        return;
    }

    dirty_entries_process();

    if (m_entry_in_progress_cnt != 0)
    { /* fulfill the entry logic second. */
        return;
    }

    nrf_mesh_evt_t evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
    event_handle(&evt);
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

#if PERSISTENT_STORAGE == 0
static bool bearer_event_cb(void)
{
    nrf_mesh_evt_t evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
    event_handle(&evt);
    return true;
}
#endif

void mesh_config_init(void)
{
    m_entry_in_progress_cnt = 0;
    m_file_in_progress_cnt = 0;
    m_is_emergency_action = false;
    m_is_emergency_cache_exist = false;

    entry_validation();
#if PERSISTENT_STORAGE
    mesh_config_backend_init(entry_params_get(0), CONFIG_ENTRY_COUNT, file_params_get(0), CONFIG_FILE_COUNT, backend_evt_handler);
#else
    (void)m_is_emergency_action;
    m_bearer_event_flag = bearer_event_flag_add(bearer_event_cb);
#endif
}

void mesh_config_load(void)
{
    mesh_config_backend_read_all(restore_callback);

    if (m_is_emergency_cache_exist)
    {
        m_is_emergency_cache_exist = false;
        mesh_config_file_clear(MESH_OPT_EMERGENCY_CACHE_FILE_ID);
    }

    FOR_EACH_FILE(p_file)
    {
        if (p_file->strategy == MESH_CONFIG_STRATEGY_ON_POWER_DOWN)
        {
            if (p_file->id != MESH_OPT_EMERGENCY_CACHE_FILE_ID &&
                    !mesh_config_backend_is_there_power_down_place(p_file->p_backend_data))
            {
                mesh_config_file_clear(p_file->id);
            }
        }
    }
}

void mesh_config_power_down(void)
{
    m_is_emergency_action = true;
    mesh_config_backend_power_down();
    dirty_entries_process();

    if (!mesh_config_is_busy())
    { // there was no data for emergency storage
        nrf_mesh_evt_t evt = {.type = NRF_MESH_EVT_CONFIG_STABLE};
        event_handle(&evt);
    }
}

bool mesh_config_is_busy(void)
{
    return m_entry_in_progress_cnt != 0 || m_file_in_progress_cnt != 0;
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
        if (p_params->has_default || (*entry_flags_get(p_params, id) & MESH_CONFIG_ENTRY_FLAG_ACTIVE))
        {
            p_params->callbacks.getter(id, p_entry);
        }
        else
        {
            return NRF_ERROR_INVALID_STATE;
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

            dirty_entries_process();
#if PERSISTENT_STORAGE == 0
            bearer_event_flag_set(m_bearer_event_flag);
#endif
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

void mesh_config_file_clear(uint16_t file_id)
{
    const mesh_config_file_params_t * p_file = file_params_find(file_id);

    if (p_file == NULL)
    {
        /* If certain file is not found, there is no use for the clear call, silently return. */
        return;
    }

    FOR_EACH_ENTRY(p_params)
    {
        if (p_params->p_id->file != file_id)
        {
            continue;
        }

        for (uint32_t j = 0; j < p_params->max_count; ++j)
        {
            if (p_params->p_state[j] & MESH_CONFIG_ENTRY_FLAG_ACTIVE)
            {
                mesh_config_entry_id_t id = *p_params->p_id;
                id.record += j;

                if (p_params->callbacks.deleter)
                {
                    p_params->callbacks.deleter(id);
                }
                listeners_notify(p_params, MESH_CONFIG_CHANGE_REASON_DELETE, id, NULL);

                p_params->p_state[j] &=  /* no longer active and request to the entry processor does not have sense. */
                                    (mesh_config_entry_flags_t)~(MESH_CONFIG_ENTRY_FLAG_ACTIVE | MESH_CONFIG_ENTRY_FLAG_DIRTY);
            }
        }
    }

#if PERSISTENT_STORAGE
    if (p_file->strategy != MESH_CONFIG_STRATEGY_NON_PERSISTENT)
    {
        m_file_in_progress_cnt++;
        mesh_config_backend_file_clean(p_file->p_backend_data);
    }
#endif
}

void mesh_config_clear(void)
{
    FOR_EACH_FILE(p_file)
    {
        if (MESH_OPT_FIRST_FREE_ID > p_file->id)
        {
            mesh_config_file_clear(p_file->id);
        }
    }
}
